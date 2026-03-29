#include "config.h"
#include "motor.h"
#include "sensors.h"

// Global variables/objects; accessible in both setup() and loop()
CurrentSensor* currentSensor = nullptr;
IRDistanceSensor* irSensor = nullptr;
LightSensor* lightSensor = nullptr;
Motor* motor = nullptr;

void setup() {
    // Initialize sensors (pins defined in config.h)
    currentSensor = new CurrentSensor(CURRENT_READ);
    irSensor = new IRDistanceSensor(IR_DIST_READ);
    lightSensor = new LightSensor(LIGHT_READ);

    // Initialize motor (control pins and parameters defined in config.h)
    motor = new Motor(MS1, MS2, MS3, STEP, DIR, FULL_STEPS_PER_REV);

    // Start with motor direction set to clockwise (i.e. forward)
    motor->setDirection(true);

    // Set initial micro-stepping mode and delay between steps
    // (adjust as needed to configure speed and "smoothness")
    motor->setStepMode(half);
    motor->setHalfDelay(1);
}

void loop() {}
