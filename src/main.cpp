#include <Arduino.h>

#include "../include/base.h"

#include "./bluetooth.h"
#include "./controller.h"
#include "./flight_log.h"
#include "./gps.h"
#include "./imu.h"
#include "./kalman_filter.h"
#include "./servos.h"

#define I2C_SDA 1
#define I2C_SCL 0

static f32 imu_dt_default = 1.0f / 104.0f;
static f32 dt = 1.0f / 104.0f;

static f32 g_acceleration = -9.82;

#define LIFTOFF_ACCELERATION_TRIGGER 4

#define LOG_PERIOD_MS 100
#define FLIGHT_LOG_TIME_MS (8UL * 60UL * 1000UL) // 8 minutes

static b32 g_log_closed = false;

static imu_measurement_t imu_measurement;
static gps_measurement_t gps_measurement;

static controller_t controller;

typedef enum {
    IDLE = 0,
    CALIBRATION = 1,
    READY = 2,
    LIFTOFF = 3,
    CONTROL = 4,
    APOGEE = 5,
    TOUCHDOWN = 6
} flight_state_t;

static flight_state_t g_state = IDLE;
static u32 g_state_enter_ms = 0;

static const matrix *g_last_kf_state = NULL;
static u32 g_last_kf_update_us = 0;

static void set_fins_neutral(void) {
    f32 neutral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    servos_write(neutral);
}

static const char *state_name(flight_state_t s) {
    switch (s) {
    case IDLE:
        return "IDLE";
    case CALIBRATION:
        return "CALIBRATION_CHECKS";
    case READY:
        return "READY";
    case LIFTOFF:
        return "LIFTOFF";
    case CONTROL:
        return "CONTROL";
    case APOGEE:
        return "APOGEE";
    case TOUCHDOWN:
        return "TOUCHDOWN";
    default:
        return "UNKNOWN";
    }
}

static void enter_state(flight_state_t next) {
    const flight_state_t prev = g_state;
    g_state = next;
    g_state_enter_ms = millis();

    if (next == IDLE) {
        set_fins_neutral();
    }

    if (next == LIFTOFF) {
        flog_init();
        g_log_closed = false;
    }

    if (next == CONTROL) {
        // Re-prime dt timing at mode boundary to avoid transition spikes.
        g_last_kf_update_us = 0;
    }

    // When leaving CONTROL
    if (prev == CONTROL && next != CONTROL) {
        flog_close();
        set_fins_neutral();
    }

    ble_sendf("STATE,%d,%d,%s", (int)prev, (int)next, state_name(next));
}

static void handle_cmd(const char *cmd) {
    if (strcmp(cmd, "CALIBRATE") == 0) {
        if (g_state == IDLE) {
            g_last_kf_state = NULL;
            g_last_kf_update_us = 0;
            enter_state(CALIBRATION);
        } else {
            ble_send("ERROR: CALIBRATE only valid in IDLE");
        }

    } else if (strcmp(cmd, "RESET") == 0) {
        g_last_kf_state = NULL;
        g_last_kf_update_us = 0;
        dt = 0.0f;
        controller_reset(&controller);
        gps_reset_origin();
        enter_state(IDLE);

    } else if (strcmp(cmd, "DUMP") == 0) {
        if (!flog_dump_ble()) {
            ble_send("ERROR: DUMP failed");
        }

    } else {
        ble_send("ERROR: Unknown command");
    }
}

static b32 init_kalman_from_current_imu(void) {
    if (!get_imu_data(&imu_measurement)) {
        return false;
    }

    // Gravity reference value
    g_acceleration = imu_measurement.ax;

    // Build init state matrix
    quat init_q = {1.0f, 0.0f, 0.0f, 0.0f};

    f32 init_state_buf[state_dim] = {0};
    matrix init_state;
    mat_create(&init_state, state_dim, 1, init_state_buf);

    init_state.data[a_x] = imu_measurement.ax;
    init_state.data[a_y] = imu_measurement.ay;
    init_state.data[a_z] = imu_measurement.az;

    if (!kalman_filter_init(&init_state, init_q, 1.0f)) {
        return false;
    }

    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    ble_init();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    servos_setup();
    gps_setup();

    // i2c_scan();

    // PID tune
    // TODO: Tune gains according to simulations in simulink
    controller.pid_roll = (pid){.kp = 5.0f, .ki = 0.0f, .kd = 0.4f};
    controller.pid_pitch = (pid){.kp = 5.0f, .ki = 0.0f, .kd = 0.4f};
    controller.pid_yaw = (pid){.kp = 3.0f, .ki = 0.0f, .kd = 0.2f};
    controller.pid_roll.integral_limit = 1.0f;
    controller.pid_pitch.integral_limit = 1.0f;
    controller.pid_yaw.integral_limit = 1.0f;

    // Desired attitude: fixed straight up (world/body aligned).
    // NOTE: This is what we want for a first launch
    controller.q_desired = (quat){1.0f, 0.0f, 0.0f, 0.0f};
    quat_normalise(&controller.q_desired);
    controller_reset(&controller);

    set_fins_neutral();
    g_state = IDLE;
    g_state_enter_ms = millis();

    ble_send("STATE,0,0,IDLE");
}

void loop() {
    static u32 last_log_ms = 0;
    char cmd[64];

    if (ble_poll_cmd(cmd, (u8)sizeof(cmd))) {
        handle_cmd(cmd);
    }

    switch (g_state) {
    case IDLE:
        break;

    case CALIBRATION:
        if (!imu_init()) {
            ble_send("ERROR: IMU init failed");
            enter_state(IDLE);
            break;
        }

        if (imu_sample_rate_hz > 0.0f) {
            imu_dt_default = 1.0f / imu_sample_rate_hz;
            dt = imu_dt_default;
        }

        if (!imu_calibrate()) {
            ble_send("ERROR: IMU calibration failed");
            enter_state(IDLE);
            break;
        }

        if (!init_kalman_from_current_imu()) {
            ble_send("ERROR: Kalman init failed");
            enter_state(IDLE);
            break;
        }

        enter_state(READY);

        break;

    case READY: {
        const u32 now_ms = millis();
        const u32 now_us = micros();

        if (gps_read_local_enu(&gps_measurement) && gps_measurement.fresh &&
            gps_measurement.valid) {
            kalman_filter_set_gps_enu(gps_measurement.x_east_m,
                                      gps_measurement.y_north_m,
                                      gps_measurement.z_up_m);
        }

        if (get_imu_data(&imu_measurement)) {
            if (imu_measurement.ax >
                g_acceleration + LIFTOFF_ACCELERATION_TRIGGER) {
                enter_state(LIFTOFF);
                break;
            }

            if (g_last_kf_update_us == 0) {
                g_last_kf_update_us = now_us;
            } else {
                dt = (f32)(now_us - g_last_kf_update_us) * 1e-6f;
                g_last_kf_update_us = now_us;
				if (dt < 0.001f || dt > 0.1f) {
					ble_send("Unusual dt, falling back to imu_dt_default");
					dt = imu_dt_default;
				}

                kalman_filter_rotate_accel(
                    imu_measurement.ax, imu_measurement.ay, imu_measurement.az);
                g_last_kf_state = kalman_filter_update(dt, imu_measurement.wx,
                                                       imu_measurement.wy,
                                                       imu_measurement.wz);
            }

            // Periodic sensor health report over BLE
            static u32 last_ready_log_ms = 0;
            if (now_ms - last_ready_log_ms >= 500) {
                last_ready_log_ms = now_ms;

                ble_sendf("IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                          imu_measurement.ax, imu_measurement.ay,
                          imu_measurement.az, imu_measurement.wx,
                          imu_measurement.wy, imu_measurement.wz);

                gps_measurement_t gps;
                if (gps_read_local_enu(&gps) && gps.fresh) {
                    if (gps.valid) {
                        ble_sendf("GPS,%.2f,%.2f,%.2f", gps.x_east_m,
                                  gps.y_north_m, gps.z_up_m);
                    } else {
                        ble_send("GPS,NO_FIX");
                    }
                }
            }

            static u32 last_ready_kalman_log_ms = 0;
            if (now_ms - last_ready_kalman_log_ms >= LOG_PERIOD_MS &&
                g_last_kf_state) {
                last_ready_kalman_log_ms = now_ms;
                const quat *q_now = kalman_filter_get_quat();
                ble_sendf(
                    "%lu,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%."
                    "6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f",
                    (unsigned long)now_ms, imu_measurement.wx,
                    imu_measurement.wy, imu_measurement.wz,
                    g_last_kf_state->data[p_x], g_last_kf_state->data[p_y],
                    g_last_kf_state->data[p_z], g_last_kf_state->data[v_x],
                    g_last_kf_state->data[v_y], g_last_kf_state->data[v_z],
                    g_last_kf_state->data[a_x], g_last_kf_state->data[a_y],
                    g_last_kf_state->data[a_z], g_last_kf_state->data[d_theta],
                    g_last_kf_state->data[d_alpha],
                    g_last_kf_state->data[d_beta], q_now->w, q_now->x, q_now->y,
                    q_now->z);
            }
        }

        break;
    }
    case LIFTOFF:
        // NOTE: We do not want to attempt to control the rocket during
        // this high acceleration phase.
        //       Checking for velocity or altitude is unreliable.
        //       Therefore we hardcode a timer. This minimizes the risk
        //       of the statelock.

        if (millis() - g_state_enter_ms >= 1500) {
            enter_state(CONTROL);
        }

        break;

    // FIX: Dont read every loop call, use interupt pin
    case CONTROL:
        const u32 now_us = micros();
        const u32 now_ms = millis();

        if (millis() - g_state_enter_ms >= FLIGHT_LOG_TIME_MS) {
            if (!g_log_closed) {
                flog_close();
                g_log_closed = true;
            }
            break;
        }

        if (gps_read_local_enu(&gps_measurement) && gps_measurement.fresh &&
            gps_measurement.valid) {
            kalman_filter_set_gps_enu(gps_measurement.x_east_m,
                                      gps_measurement.y_north_m,
                                      gps_measurement.z_up_m);
        }

        if (get_imu_data(&imu_measurement)) {
            if (g_last_kf_update_us == 0) {
                g_last_kf_update_us = now_us;
                break;
            }

            dt = (f32)(now_us - g_last_kf_update_us) * 1e-6f;
            g_last_kf_update_us = now_us;
			if (dt < 0.001f || dt > 0.1f) {  // reject anything under 1ms or over 100ms
				ble_send("Unusual dt, falling back to imu_dt_default");
				dt = imu_dt_default;
			}

            kalman_filter_rotate_accel(imu_measurement.ax, imu_measurement.ay,
                                       imu_measurement.az);
            const matrix *s = kalman_filter_update(
                dt, imu_measurement.wx, imu_measurement.wy, imu_measurement.wz);
            const quat *q = kalman_filter_get_quat();
            controller_update(&controller, dt, (quat *)q, (matrix *)s);
            g_last_kf_state = s;

            servos_write(controller.fin_angle_deg);
        }

        if (now_ms - last_log_ms >= LOG_PERIOD_MS) {
            last_log_ms = now_ms;

            if (g_last_kf_state) {
                const quat *q_now = kalman_filter_get_quat();
                char line[384];
                int w = snprintf(
                    line, sizeof(line),
                    "%lu,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%."
                    "6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    (unsigned long)now_ms, imu_measurement.wx,
                    imu_measurement.wy, imu_measurement.wz,
                    g_last_kf_state->data[p_x], g_last_kf_state->data[p_y],
                    g_last_kf_state->data[p_z], g_last_kf_state->data[v_x],
                    g_last_kf_state->data[v_y], g_last_kf_state->data[v_z],
                    g_last_kf_state->data[a_x], g_last_kf_state->data[a_y],
                    g_last_kf_state->data[a_z], g_last_kf_state->data[d_theta],
                    g_last_kf_state->data[d_alpha],
                    g_last_kf_state->data[d_beta], q_now->w, q_now->x, q_now->y,
                    q_now->z, controller.fin_angle_deg[0],
                    controller.fin_angle_deg[1], controller.fin_angle_deg[2],
                    controller.fin_angle_deg[3]);
                if (w > 0)
                    flog_write(line);
            }
        }

        break;

        // case APOGEE:
        //   break;
        //
        // case TOUCHDOWN:
        //   break;
        //
        // default:
        //   break;
    }
}
