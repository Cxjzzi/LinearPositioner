#pragma once

// Pins for A4988 stepper motor driver
#define MS1 5
#define MS2 6
#define MS3 7
#define STEP 9
#define DIR 10

// Motor steps per revolution (200 for 1.8-degree stepper)
#define FULL_STEPS_PER_REV 200

// Pin (analog) for IR distance sensor
#define IR_DIST_READ A0

// Pin (analog) for current sensor
#define CURRENT_READ A1

// Pin for light sensor digital output
#define LIGHT_READ 2
