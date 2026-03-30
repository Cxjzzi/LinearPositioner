#include "config.h"
#include "motor.h"
#include "sensors.h"

// Adjusted by user -------------------------------------------------
// ------------------------------------------------------------------

// Set whether to run continuously or to a specific position
bool continuousMode = false;

// Set target position for non-continuous mode (in cm, from end)
float targetPosition = 30.0;

// ------------------------------------------------------------------
// ------------------------------------------------------------------

// Global variables/objects; accessible in both setup() and loop()
CurrentSensor* currentSensor = nullptr;
FlexSensor* flexSensor = nullptr;
IRDistanceSensor* irSensor = nullptr;
LightSensor* lightSensor = nullptr;
Motor* motor = nullptr;
bool isStopped = false;
bool positionReached = false;

void setup() {
    // Start serial communication for debugging
    Serial.begin(9600);

    // Configure LED and push button pins
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(PUSH_BUTTON, INPUT);

    // Initialize sensors (pins defined in config.h)
    currentSensor = new CurrentSensor(CURRENT_READ);
    flexSensor = new FlexSensor(FLEX_READ);
    irSensor = new IRDistanceSensor(IR_DIST_READ);
    lightSensor = new LightSensor(LIGHT_READ);

    // Initialize motor (control pins and parameters defined in config.h)
    motor = new Motor(MS1, MS2, MS3, STEP, DIR, FULL_STEPS_PER_REV);

    // Start with motor direction set to forward
    motor->setDirection(forward);

    // Set initial micro-stepping mode and delay between steps
    // (adjust as needed to configure speed and "smoothness")
    motor->setStepMode(half);
    motor->setHalfDelay(1);
}

void loop() {
    // Push button E-stop
    if (digitalRead(PUSH_BUTTON)) {
        isStopped = true;
    }

    // Check if motor is E-stopped or if it's dark
    if (isStopped || !lightSensor->isLight()) {
        // Red LED on
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    } else {
        // Green LED on
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(RED_LED, LOW);

        // Check whether to run continuously or to a specific position
        if (continuousMode) {
            // Read distance and set motor direction accordingly
            float distance = irSensor->readDistance();
            if (distance < 10.0) {
                motor->setDirection(reverse);
            } else if (distance > 30.0) {
                motor->setDirection(forward);
            }
            // Run motor
            motor->rotate();
            // Print current reading
            float current = currentSensor->readCurrent();
            Serial.println(current);

        } else if (!continuousMode && !positionReached) {
            // Measure starting error
            float error = irSensor->readDistance() - targetPosition;
            // Set motor direction based on error sign
            if (error >= 0) {
                motor->setDirection(forward);
            } else {
                motor->setDirection(reverse);
            }
            // Attempt to reach target position
            int revsToMove = (int)(abs(error) / DIST_PER_REV);
            for (int i = 0; i < revsToMove; i++) {
                if (digitalRead(PUSH_BUTTON)) {
                    isStopped = true;
                }
                if (!isStopped) {
                    motor->rotate();
                } else {
                    break;
                }
            }
            // Adjust until within target position threshold
            while (abs(error) > 0.5) {
                if (digitalRead(PUSH_BUTTON)) {
                    isStopped = true;
                }
                if (!isStopped) {
                    // Read distance and calculate error from target position
                    error = irSensor->readDistance() - targetPosition;
                    // Set motor direction based on error sign
                    if (error >= 0) {
                        motor->setDirection(forward);
                    } else {
                        motor->setDirection(reverse);
                    }
                    // Run motor one revolution
                    motor->rotate();
                } else {
                    break;
                }
            }
            positionReached = true;
        }
    }
}
