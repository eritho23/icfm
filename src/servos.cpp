#include "servos.h"

Servo servos[4];

void servos_setup() {
	servos[0].attach(servo_pins[0]); // 1KHz 10 bits
	servos[1].attach(servo_pins[1]);
	servos[2].attach(servo_pins[2]);
	servos[3].attach(servo_pins[3]);
}

void test_servos() {
    servos[0].write(80);
    delay(800);
    servos[0].write(100);
    delay(800);
}

void servos_write(f32 fin_servo_angle[4]) {
    for (int i = 0; i < 4; i++) {
        f32 servo_pos = 90.0f + fin_servo_angle[i];
        servos[i].write((int)servo_pos);
    }
}

void servos_debug_write_angles(f32 fin_servo_angle[4]) {
    for (int i = 0; i < 4; i++) {
        f32 servo_pos = 90.0f + fin_servo_angle[i];
		Serial.printf("Servo %d: ", i); Serial.println((int)servo_pos);
    }
}
