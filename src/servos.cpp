#include "servos.h"

// angle is in [-40, +40] degrees
static int us_to_duty(int pulse_us) {
    pulse_us = constrain(pulse_us, SERVO_MIN_US, SERVO_MAX_US);
    return (int)(((int64_t)pulse_us * 16383) / 20000);
}

static int angle_to_us(f32 angle) {
    angle = constrain(angle, -40.0f, 40.0f);
    return SERVO_CENTER_US + (int)(angle * 10.0f);
}

static int angle_to_duty(f32 angle) { return us_to_duty(angle_to_us(angle)); }

void servos_setup() {
    ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                 .duty_resolution = LEDC_TIMER_14_BIT,
                                 .timer_num = LEDC_TIMER_0,
                                 .freq_hz = 50,
                                 .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = (uint32_t)angle_to_duty(0.0f), // 0 = center
            .hpoint = 0};
        ledc_channel_config(&ch);
        Serial.printf("Servo %d attached to pin %d\n", i, servo_pins[i]);
    }
}

static void servo_write_angle(int channel, f32 angle) {
    uint32_t duty = (uint32_t)angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void servos_write(f32 fin_servo_angle[4]) {
    for (int i = 0; i < 4; i++) {
        const int servo_channel = fin_to_servo_channel[i];
        servo_write_angle(servo_channel, fin_servo_angle[i]);
    }
}

void servos_test() {
    servo_write_angle(0, -40.0f);
    servo_write_angle(1, -40.0f);
    servo_write_angle(2, -40.0f);
    servo_write_angle(3, -40.0f);
    delay(800);
    servo_write_angle(0, 40.0f);
    servo_write_angle(1, 40.0f);
    servo_write_angle(2, 40.0f);
    servo_write_angle(3, 40.0f);
    delay(800);
    servo_write_angle(0, 0.0f);
    servo_write_angle(1, 0.0f);
    servo_write_angle(2, 0.0f);
    servo_write_angle(3, 0.0f);
    delay(800);
}

void servos_debug_write_angles(f32 fin_servo_angle[4]) {
    for (int i = 0; i < 4; i++) {
        Serial.printf("Fin %d -> Servo %d: %.1f deg\n", i,
                      fin_to_servo_channel[i], fin_servo_angle[i]);
    }
}
