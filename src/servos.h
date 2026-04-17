#ifndef ICFM_SERVOS_H
#define ICFM_SERVOS_H

#include "driver/ledc.h"
#include <Arduino.h>

#include "../include/base.h"

static const int servo_pins[4] = {2, 3, 10, 11};

// Logical fin index (controller) -> physical servo channel.
// Controller assumes fins 0/2 are opposite and 1/3 are opposite.
// Adjust this map to match the actual wiring/layout around the body.
static const int fin_to_servo_channel[4] = {0, 2, 3, 1};

// Corona CS238MG: +-40deg travel with 400us per side from 1520us center.
// At 50Hz, full period is 20000us.
static const int SERVO_CENTER_US = 1520;
static const int SERVO_TRAVEL_US = 400;
static const int SERVO_MIN_US = SERVO_CENTER_US - SERVO_TRAVEL_US; // 1120us
static const int SERVO_MAX_US = SERVO_CENTER_US + SERVO_TRAVEL_US; // 1920us

void servos_setup();
void servos_test();
void servos_write(f32 fin_angle_deg[4]);
void servos_debug_write_angles(f32 fin_servo_angle[4]);

#endif
