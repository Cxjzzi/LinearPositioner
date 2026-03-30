#pragma once

// Stepper driver pins (A4988)
#define MS1 5
#define MS2 6
#define MS3 7
#define STEP 9
#define DIR 10

// Motor steps per revolution (1.8° stepper)
#define FULL_STEPS_PER_REV 200

// Sensors
#define IR_DIST_READ A3
#define CURRENT_READ A1
#define LIGHT_READ 2
#define flex A0

// LEDs
#define goLed 3
#define stopLed 4

// NEW: Emergency stop pushbutton
#define estopPin 8