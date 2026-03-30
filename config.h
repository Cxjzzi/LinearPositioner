#pragma once

// Stepper driver pins (A4988)
#define MS1 5
#define MS2 6
#define MS3 7
#define STEP 9
#define DIR 10

// Motor parameters
#define FULL_STEPS_PER_REV 200  // Steps per revolution
#define DIST_PER_REV 1.0        // Distance moved per revolution in cm

// Sensors
#define CURRENT_READ A1
#define FLEX_READ A0
#define IR_DIST_READ A3
#define LIGHT_READ 2

// LEDs
#define GREEN_LED 3
#define RED_LED 4

// Push button
#define PUSH_BUTTON 8
