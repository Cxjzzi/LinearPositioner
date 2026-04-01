#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "sensors.h"

// ------------------------------------------------------------
// Operating Modes
// ------------------------------------------------------------
enum OperatingMode {
   MODE_CONTINUOUS = 1,
   MODE_TARGET = 2,
   MODE_SEQUENCE = 3
};

OperatingMode currentMode = MODE_TARGET;

// ------------------------------------------------------------
// User-adjustable variables
// ------------------------------------------------------------
float targetPosition = 30.0;

// ------------------------------------------------------------
// PID Gains — tune these on hardware
// ------------------------------------------------------------
float Kp = 20.0;
float Ki = 0.00;
float Kd = 0.03;

// ------------------------------------------------------------
// PID state variables
// ------------------------------------------------------------
float pidIntegral = 0.0;
float pidPrevError = 0.0;
unsigned long pidLastTime = 0;

// ------------------------------------------------------------
// Target mode state
// ------------------------------------------------------------
bool targetActive = false;
Direction currentDir = forward;

// ------------------------------------------------------------
// Sequence mode state
// ------------------------------------------------------------
const int MAX_SEQUENCE_TARGETS = 10;
float sequenceTargets[MAX_SEQUENCE_TARGETS];
int sequenceCount = 0;
int currentSequenceIndex = 0;
bool sequenceActive = false;

bool dwellActive = false;
unsigned long dwellStartTime = 0;
const unsigned long DWELL_TIME_MS = 1500;   // 1.5 seconds

// ------------------------------------------------------------
// Tuning constants
// ------------------------------------------------------------
const float POSITION_TOLERANCE_CM = 0.15;
const float MIN_OUTPUT = 0.5;
const float MAX_OUTPUT = 50.0;
const float INTEGRAL_LIMIT = 20.0;

// Optional soft limits for entered targets
const float MIN_TARGET_CM = 5.0;
const float MAX_TARGET_CM = 35.0;

// ------------------------------------------------------------
// Global objects
// ------------------------------------------------------------
CurrentSensor* currentSensor = nullptr;
FlexSensor* flexSensor = nullptr;
IRDistanceSensor* irSensor = nullptr;
LightSensor* lightSensor = nullptr;
Motor* motor = nullptr;

bool isStopped = false;
bool inToleranceBand = false;
unsigned long toleranceStartTime = 0;
const unsigned long SETTLE_TIME_MS = 200;

// ------------------------------------------------------------
// Exponential Moving Average Filter
// ------------------------------------------------------------
float filteredDistance = 0.0;
bool filterInitialized = false;
const float alpha = 0.3;

// ------------------------------------------------------------
// Serial input helpers
// ------------------------------------------------------------
int readIntFromSerial() {
   while (Serial.available() == 0) {
   }
   return Serial.parseInt();
}

void clearSerialBuffer() {
   while (Serial.available() > 0) {
      Serial.read();
   }
}

// ------------------------------------------------------------
// PID helpers
// ------------------------------------------------------------
void resetPID() {
   pidIntegral = 0.0;
   pidPrevError = 0.0;
   pidLastTime = millis();
}

int outputToHalfDelay(float absOutput) {
   absOutput = constrain(absOutput, MIN_OUTPUT, MAX_OUTPUT);

   float halfDelayFloat = 5.0 - ((absOutput - MIN_OUTPUT) * (4.0 / (MAX_OUTPUT - MIN_OUTPUT)));

   int halfDelay = (int)halfDelayFloat;
   halfDelay = constrain(halfDelay, 1, 5);

   return halfDelay;
}

bool targetIsValid(float target) {
   return (target >= MIN_TARGET_CM && target <= MAX_TARGET_CM);
}

// ------------------------------------------------------------
// Sensor filtering helper
// ------------------------------------------------------------
float getFilteredDistance() {
   float rawDistance = irSensor->readDistance();

   if (!filterInitialized) {
      filteredDistance = rawDistance;
      filterInitialized = true;
   } else {
      filteredDistance = alpha * rawDistance + (1 - alpha) * filteredDistance;
   }

   return filteredDistance;
}

// ------------------------------------------------------------
// Target input helper
// ------------------------------------------------------------
void promptForTarget() {
   Serial.print("Enter target distance in cm: ");

   while (Serial.available() == 0) {
   }

   String input = Serial.readStringUntil('\n');
   input.trim();

   if (input.length() == 0) {
      return;
   }

   float newTarget = input.toFloat();

   if (newTarget == 0.0 && input != "0" && input != "0.0") {
      Serial.println("Invalid input. Please enter a number.");
      return;
   }

   if (!targetIsValid(newTarget)) {
      Serial.print("Invalid target. Enter a value between ");
      Serial.print(MIN_TARGET_CM);
      Serial.print(" and ");
      Serial.print(MAX_TARGET_CM);
      Serial.println(" cm.");
      return;
   }

   targetPosition = newTarget;
   targetActive = true;
   inToleranceBand = false;
   resetPID();

   Serial.print("Target set to: ");
   Serial.println(targetPosition);
}

// ------------------------------------------------------------
// Sequence input helper
// ------------------------------------------------------------
void promptForSequence() {
   Serial.println("Enter target sequence in cm, separated by spaces:");
   Serial.println("Example: 10 15 20 12");
   Serial.print("Sequence: ");

   while (Serial.available() == 0) {
   }

   String input = Serial.readStringUntil('\n');
   input.trim();

   if (input.length() == 0) {
      return;
   }

   sequenceCount = 0;
   currentSequenceIndex = 0;
   sequenceActive = false;

   int start = 0;
   while (start < input.length() && sequenceCount < MAX_SEQUENCE_TARGETS) {
      int spaceIndex = input.indexOf(' ', start);
      if (spaceIndex == -1) {
         spaceIndex = input.length();
      }

      String token = input.substring(start, spaceIndex);
      token.trim();

      if (token.length() > 0) {
         float value = token.toFloat();

         if (value == 0.0 && token != "0" && token != "0.0") {
            Serial.print("Skipping invalid entry: ");
            Serial.println(token);
         }
         else if (!targetIsValid(value)) {
            Serial.print("Skipping out-of-bounds target: ");
            Serial.println(value);
         }
         else {
            sequenceTargets[sequenceCount] = value;
            sequenceCount++;
         }
      }

      start = spaceIndex + 1;
   }

   if (sequenceCount == 0) {
      Serial.println("No valid targets entered.");
      return;
   }

   targetPosition = sequenceTargets[0];
   targetActive = true;
   sequenceActive = true;
   inToleranceBand = false;
   dwellActive = false;
   resetPID();

   Serial.println("Sequence loaded:");
   for (int i = 0; i < sequenceCount; i++) {
      Serial.print("  Target ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(sequenceTargets[i]);
   }

   Serial.print("Moving to first target: ");
   Serial.println(targetPosition);
}

// ------------------------------------------------------------
// Advance to next sequence target
// ------------------------------------------------------------
void advanceSequenceTarget() {
   currentSequenceIndex++;

   if (currentSequenceIndex >= sequenceCount) {
      sequenceActive = false;
      targetActive = false;
      sequenceCount = 0;
      currentSequenceIndex = 0;

      Serial.println("Sequence complete.");
      Serial.println();
      return;
   }

   targetPosition = sequenceTargets[currentSequenceIndex];
   targetActive = true;
   inToleranceBand = false;
   dwellActive = false;
   resetPID();

   Serial.print("Moving to next target: ");
   Serial.println(targetPosition);
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
   Serial.begin(115200);
   delay(500);

   Serial.println("=== Linear Positioner System ===");
   Serial.println("Select Mode:");
   Serial.println("1 = Continuous Mode");
   Serial.println("2 = Target Position Mode");
   Serial.println("3 = Sequence Mode");
   Serial.print("Enter choice: ");

   int mode = readIntFromSerial();

   if (mode == 1) {
      currentMode = MODE_CONTINUOUS;
      Serial.println("Continuous Mode Selected.");
   }
   else if (mode == 2) {
      currentMode = MODE_TARGET;
      Serial.println("Target Position Mode Selected.");
   }
   else if (mode == 3) {
      currentMode = MODE_SEQUENCE;
      Serial.println("Sequence Mode Selected.");
   }
   else {
      Serial.println("Invalid choice. Defaulting to Continuous Mode.");
      currentMode = MODE_CONTINUOUS;
   }

   clearSerialBuffer();

   // -------------------------------
   // Hardware Initialization
   // -------------------------------
   pinMode(GREEN_LED, OUTPUT);
   pinMode(RED_LED, OUTPUT);
   pinMode(PUSH_BUTTON, INPUT);

   currentSensor = new CurrentSensor(CURRENT_READ);
   flexSensor = new FlexSensor(FLEX_READ);
   irSensor = new IRDistanceSensor(IR_DIST_READ);
   lightSensor = new LightSensor(LIGHT_READ);

   motor = new Motor(MS1, MS2, MS3, STEP, DIR, FULL_STEPS_PER_REV);
   motor->setDirection(forward);
   motor->setStepMode(half);
   motor->setHalfDelay(5);

   currentDir = forward;
   resetPID();
}

// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------
void loop() {
   if (digitalRead(PUSH_BUTTON)) {
      isStopped = true;
   }

   if (isStopped || !lightSensor->isLight()) {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      return;
   }

   digitalWrite(GREEN_LED, HIGH);
   digitalWrite(RED_LED, LOW);

   // --------------------------------------------------------
   // CONTINUOUS MODE
   // --------------------------------------------------------
   if (currentMode == MODE_CONTINUOUS) {
      float currentDistance = getFilteredDistance();

      if (currentDistance < 10.0) {
         motor->setDirection(reverse);
      }
      else if (currentDistance > 30.0) {
         motor->setDirection(forward);
      }

      motor->rotate();

      float current = currentSensor->readCurrent();
      Serial.print("Current: ");
      Serial.println(current);

      return;
   }

   // --------------------------------------------------------
   // TARGET MODE + SEQUENCE MODE
   // --------------------------------------------------------
   if (currentMode == MODE_TARGET || currentMode == MODE_SEQUENCE) {

      if (!targetActive) {
         if (currentMode == MODE_TARGET) {
            promptForTarget();
         }
         else if (currentMode == MODE_SEQUENCE) {
            promptForSequence();
         }
         return;
      }

      if (currentMode == MODE_SEQUENCE && dwellActive) {
         if (millis() - dwellStartTime >= DWELL_TIME_MS) {
            dwellActive = false;
            advanceSequenceTarget();
         }
         return;
      }

      float currentDistance = getFilteredDistance();
      float error = targetPosition - currentDistance;

      // Stop when close enough
      if (abs(error) <= POSITION_TOLERANCE_CM) {
         if (!inToleranceBand) {
            inToleranceBand = true;
            toleranceStartTime = millis();
         }

         if (millis() - toleranceStartTime >= SETTLE_TIME_MS) {
            inToleranceBand = false;
            motor->setHalfDelay(5);

            Serial.print("Target reached at: ");
            Serial.print(currentDistance);
            Serial.print(" cm (Target: ");
            Serial.print(targetPosition);
            Serial.println(" cm)");

            if (currentMode == MODE_SEQUENCE && sequenceActive) {
               if (!dwellActive) {
               dwellActive = true;
               dwellStartTime = millis();
            }
            } else {
               targetActive = false;
               Serial.println();
            }

            return;
         }
      }
      else {
         inToleranceBand = false;
      }

      // Time step
      unsigned long now = millis();
      float dt = (now - pidLastTime) / 1000.0;

      if (dt <= 0.0) {
         dt = 0.001;
      }

      pidLastTime = now;

      // PID terms
      pidIntegral += error * dt;
      pidIntegral = constrain(pidIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      float derivative = (error - pidPrevError) / dt;
      pidPrevError = error;

      float output = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
      float absOutput = abs(output);

      if (absOutput < MIN_OUTPUT) {
         return;
      }

      // Speed control via half-delay
      int halfDelay = outputToHalfDelay(absOutput);
      motor->setHalfDelay(halfDelay);

      // Direction control
      Direction newDir = (output >= 0) ? reverse : forward;

      if (newDir != currentDir) {
         motor->setDirection(newDir);
         currentDir = newDir;
      }

      // Single PID-controlled step
      motor->step();

      // Serial monitoring
      static unsigned long lastPrintTime = 0;
      unsigned long nowPrint = millis();

      if (nowPrint - lastPrintTime >= 100) {
         Serial.print("Current: ");
         Serial.print(currentDistance);
         Serial.print(" | Target: ");
         Serial.print(targetPosition);
         Serial.print(" | Error: ");
         Serial.println(error);

         lastPrintTime = nowPrint;
      }

      return;
   }
}