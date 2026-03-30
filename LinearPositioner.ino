#include "config.h"
#include "motor.h"
#include "sensors.h"

Motor* motor;
CurrentSensor* currentSensor;
IRDistanceSensor* irSensor;
LightSensor* lightSensor;

unsigned long stopStartTime = 0;
bool stopDelayActive = false;
bool reversingUntilFlex = false;

void setup() {
    Serial.begin(9600);

    motor = new Motor(MS1, MS2, MS3, STEP, DIR, FULL_STEPS_PER_REV);
    motor->setStepMode(half);
    motor->setAcceleration(300);
    motor->setTargetSpeed(800);

    currentSensor = new CurrentSensor(CURRENT_READ);
    irSensor      = new IRDistanceSensor(IR_DIST_READ);
    lightSensor   = new LightSensor(LIGHT_READ);

    pinMode(goLed, OUTPUT);
    pinMode(stopLed, OUTPUT);
    pinMode(estopPin, INPUT_PULLUP);
}

void loop() {
    motor->update();

    float dist = irSensor->readDistance();
    int flexValue = analogRead(flex);
    bool dark = lightSensor->isDark();
    bool running = motor->isRunning();

    // -----------------------------
    // DEBUG OUTPUT
    // -----------------------------
    Serial.print("Dist=");
    Serial.print(dist);
    Serial.print(" | Flex=");
    Serial.print(flexValue);
    Serial.print(" | Dark=");
    Serial.print(dark);
    Serial.print(" | Running=");
    Serial.print(running);
    Serial.print(" | State=");
    Serial.print(motor->getState());
    Serial.print(" | Dir=");
    Serial.print(motor->getDirection());
    Serial.print(" | Speed=");
    Serial.println(motor->getCurrentSpeed());

    // -----------------------------
    // PUSHBUTTON EMERGENCY STOP
    // -----------------------------
    if (digitalRead(estopPin) == LOW) {
        motor->emergencyStop();
        digitalWrite(stopLed, HIGH);
        digitalWrite(goLed, LOW);
        return;
    }

    // -----------------------------
    // FLEX EMERGENCY STOP
    // -----------------------------
    if (flexValue > 800) {
        motor->emergencyStop();
        digitalWrite(stopLed, HIGH);
        digitalWrite(goLed, LOW);
        return;
    }

    // -----------------------------
    // STOP → WAIT 5 SEC → REVERSE UNTIL FLEX > 20
    // -----------------------------
    if (!running && !reversingUntilFlex) {
        if (!stopDelayActive) {
            stopDelayActive = true;
            stopStartTime = millis();
        }

        digitalWrite(stopLed, HIGH);
        digitalWrite(goLed, LOW);

        if (millis() - stopStartTime < 5000) return;

        stopDelayActive = false;
        reversingUntilFlex = true;

        motor->reverse();
        motor->setTargetSpeed(300);
    }

    if (reversingUntilFlex) {
        digitalWrite(stopLed, HIGH);
        digitalWrite(goLed, LOW);

        if (flexValue > 20) {
            reversingUntilFlex = false;
        } else {
            return;
        }
    }

    // -----------------------------
    // NORMAL LOGIC
    // -----------------------------
    if (!reversingUntilFlex) {
        if (dist < 10) {
            motor->reverse();
            motor->setTargetSpeed(400);
        }

        if (dark) motor->setTargetSpeed(200);
        else      motor->setTargetSpeed(800);
    }

    // -----------------------------
    // LED LOGIC
    // -----------------------------
    if (!running || dist > 7 || reversingUntilFlex) {
        digitalWrite(stopLed, HIGH);
        digitalWrite(goLed, LOW);
    } else {
        digitalWrite(stopLed, LOW);
        digitalWrite(goLed, HIGH);
    }
}