#include <Arduino.h>

#include "../include/base.h"

#include "./controller.h"
#include "./imu.h"
#include "./kalman_filter.h"
#include "./servos.h"
#include "./utils.h"

#define I2C_SDA 1
#define I2C_SCL 0

const f32 imu_dt_default = 1.0f / imu_sample_rate_hz;
f32 dt = imu_dt_default;

#define CONTROL_LOG_PERIOD_MS 100

static imu_m imu_measurement;

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
static flight_state_t g_prev_state = IDLE;
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
  g_prev_state = g_state;
  g_state = next;
  g_state_enter_ms = millis();

  if (next == IDLE || next == READY || next == TOUCHDOWN) {
    set_fins_neutral();
  }

  Serial.print("STATE,");
  Serial.print((int)g_prev_state);
  Serial.print(",");
  Serial.print((int)g_state);
  Serial.print(",");
  Serial.println(state_name(g_state));
}

static b32 cmd_is(const char *cmd, const char *word) {
  return strcmp(cmd, word) == 0;
}

static b32 init_kalman_from_current_imu(void) {
  if (!get_imu_data(&imu_measurement)) {
    return false;
  }

  // Build init state matrix
  quat init_q = {1.0f, 0.0f, 0.0f, 0.0f};

  f32 init_state_buf[state_dim] = {0};
  matrix init_state;
  mat_create(&init_state, state_dim, 1, init_state_buf);

  f32 a_body[3] = {imu_measurement.ax, imu_measurement.ay, imu_measurement.az};
  f32 a_world[3];
  quat_rotate(&init_q, a_body, a_world);
  init_state.data[a_x] = a_world[0];
  init_state.data[a_y] = a_world[1];
  init_state.data[a_z] = a_world[2];

  if (!kalman_filter_init(&init_state, init_q, 1.0f)) {
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  servos_setup();

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

  set_fins_neutral();
  g_state = IDLE;
  g_prev_state = IDLE;
  g_state_enter_ms = millis();

  Serial.println("STATE,0,0,IDLE");
}

void loop() {
  static u32 last_log_ms = 0;
  char cmd[64];

  if (serial_read_line(cmd, (int)sizeof(cmd))) {
    if (cmd_is(cmd, "CALIBRATE")) {
      if (g_state == IDLE || g_state == TOUCHDOWN) {
        g_last_kf_state = NULL;
        g_last_kf_update_us = 0;
        enter_state(CALIBRATION);
      } else {
        Serial.println("ERROR: CALIBRATE only valid in IDLE or TOUCHDOWN");
      }
    } else if (cmd_is(cmd, "RESET")) {
      g_last_kf_state = NULL;
      g_last_kf_update_us = 0;
      enter_state(IDLE);
    } else {
      Serial.println("ERROR: Unknown command");
    }
  }

  switch (g_state) {
  case IDLE:
    break;

  // FIX: Decide where to start recording flight data
  case CALIBRATION:
    if (!imu_init()) {
      Serial.println("ERROR: IMU init failed");
      enter_state(IDLE);
      break;
    }

    if (!imu_calibrate()) {
      Serial.println("ERROR: IMU calibration failed");
      enter_state(IDLE);
      break;
    }

    if (!init_kalman_from_current_imu()) {
      Serial.println("ERROR: Kalman init failed");
      enter_state(IDLE);
      break;
    }

    enter_state(READY);

    break;

  case READY:
    // NOTE: Just go to next state for now
    enter_state(LIFTOFF);

    // TODO: Detect liftoff via gps i suppose, then go to liftoff state

    break;
  case LIFTOFF:
    // NOTE: Just go to next state for now
    enter_state(CONTROL);

    break;

  // FIX: Dont read every loop call, use interupt pin
  case CONTROL:
    const u32 now_us = micros();
    const u32 now_ms = millis();

    if (get_imu_data(&imu_measurement)) {
      if (g_last_kf_update_us == 0) {
        g_last_kf_update_us = now_us;
        break;
      }

      dt = (f32)(now_us - g_last_kf_update_us) * 1e-6f;
      g_last_kf_update_us = now_us;
      if (dt <= 0.0f || dt > 0.1f || dt <= 1e-6f) {
        Serial.print("Unusual calculated dt occured, if this happens too often "
                     "something is wrong");
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

    if (now_ms - last_log_ms >= CONTROL_LOG_PERIOD_MS) {
      last_log_ms = now_ms;

      f32 roll, pitch, yaw;
      kalman_filter_get_euler_deg(&roll, &pitch, &yaw);

      Serial.print("Roll: ");
      Serial.print(roll, 4);
      Serial.print(" Pitch: ");
      Serial.print(pitch, 4);
      Serial.print(" Yaw: ");
      Serial.println(yaw, 4);

      servos_debug_write_angles(controller.fin_angle_deg);

      kalman_filter_debug_print_csv_row(now_ms, imu_measurement.wx,
                                        imu_measurement.wy, imu_measurement.wz,
                                        g_last_kf_state);
      controller_debug_print_csv_row(now_ms, &controller);
    }

    break;

    // case APOGEE:
    // 	set_fins_neutral();
    // 	break;
    //
    // case TOUCHDOWN:
    // 	set_fins_neutral();
    // 	break;
    //
    // default:
    // 	enter_state(IDLE);
    // 	break;
  }
}
