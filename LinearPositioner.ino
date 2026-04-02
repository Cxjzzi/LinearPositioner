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
   MODE_SEQUENCE = 3,
   MODE_FORCE = 4
};

OperatingMode currentMode = MODE_TARGET;

// ------------------------------------------------------------
// User-adjustable variables
// ------------------------------------------------------------
float targetPosition = 30.0;
float targetForce = 2.0;

// ------------------------------------------------------------
// Position-control PID Gains
// ------------------------------------------------------------
float Kp = 20.0;
float Ki = 0.00;
float Kd = 0.00;

// ------------------------------------------------------------
// Force-control PID Gains
// ------------------------------------------------------------
float KpForce = 12.0;
float KiForce = 0.00;
float KdForce = 0.00;

// ------------------------------------------------------------
// PID state variables (position)
// ------------------------------------------------------------
float pidIntegral = 0.0;
float pidPrevError = 0.0;
unsigned long pidLastTime = 0;

// ------------------------------------------------------------
// PID state variables (force)
// ------------------------------------------------------------
float forceIntegral = 0.0;
float forcePrevError = 0.0;
unsigned long forceLastTime = 0;

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
// Force mode state
// ------------------------------------------------------------
bool forceActive = false;
bool contactDetected = false;

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

// Force control constants
const float MIN_FORCE_TARGET_N = 0.2;
const float MAX_FORCE_TARGET_N = 8.0;
const float FORCE_TOLERANCE_N = 0.10;
const float CONTACT_FORCE_THRESHOLD_N = 0.40;
const float FORCE_MIN_OUTPUT = 0.5;
const float FORCE_MAX_OUTPUT = 35.0;
const float FORCE_INTEGRAL_LIMIT = 10.0;

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
// Exponential Moving Average Filter (position)
// ------------------------------------------------------------
float filteredDistance = 0.0;
bool filterInitialized = false;
const float alpha = 0.3;

// ------------------------------------------------------------
// Exponential Moving Average Filter (force)
// ------------------------------------------------------------
float filteredForce = 0.0;
bool forceFilterInitialized = false;
const float forceAlpha = 0.2;

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

void resetForcePID() {
   forceIntegral = 0.0;
   forcePrevError = 0.0;
   forceLastTime = millis();
}

int outputToHalfDelay(float absOutput) {
   absOutput = constrain(absOutput, MIN_OUTPUT, MAX_OUTPUT);

   float halfDelayFloat = 5.0 - ((absOutput - MIN_OUTPUT) * (4.0 / (MAX_OUTPUT - MIN_OUTPUT)));

   int halfDelay = (int)halfDelayFloat;
   halfDelay = constrain(halfDelay, 1, 5);

   return halfDelay;
}

int forceOutputToHalfDelay(float absOutput) {
   absOutput = constrain(absOutput, FORCE_MIN_OUTPUT, FORCE_MAX_OUTPUT);

   float halfDelayFloat = 6.0 - ((absOutput - FORCE_MIN_OUTPUT) * (5.0 / (FORCE_MAX_OUTPUT - FORCE_MIN_OUTPUT)));
   int halfDelay = (int)halfDelayFloat;
   halfDelay = constrain(halfDelay, 1, 6);

   return halfDelay;
}

bool targetIsValid(float target) {
   return (target >= MIN_TARGET_CM && target <= MAX_TARGET_CM);
}

bool forceTargetIsValid(float force) {
   return (force >= MIN_FORCE_TARGET_N && force <= MAX_FORCE_TARGET_N);
}

// ------------------------------------------------------------
// Sensor filtering helpers
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

float getFilteredForce() {
   float rawForce = flexSensor->readForce();

   if (!forceFilterInitialized) {
      filteredForce = rawForce;
      forceFilterInitialized = true;
   } else {
      filteredForce = forceAlpha * rawForce + (1.0 - forceAlpha) * filteredForce;
   }

   return filteredForce;
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
// Force input helper
// ------------------------------------------------------------
void promptForForceTarget() {
   Serial.print("Enter target force in N: ");

   while (Serial.available() == 0) {
   }

   String input = Serial.readStringUntil('\n');
   input.trim();

   if (input.length() == 0) {
      return;
   }

   float newForce = input.toFloat();

   if (newForce == 0.0 && input != "0" && input != "0.0") {
      Serial.println("Invalid input. Please enter a number.");
      return;
   }

   if (!forceTargetIsValid(newForce)) {
      Serial.print("Invalid target force. Enter a value between ");
      Serial.print(MIN_FORCE_TARGET_N);
      Serial.print(" and ");
      Serial.print(MAX_FORCE_TARGET_N);
      Serial.println(" N.");
      return;
   }

   targetForce = newForce;
   forceActive = true;
   contactDetected = false;
   resetForcePID();

   Serial.print("Target force set to: ");
   Serial.print(targetForce);
   Serial.println(" N");
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
   Serial.println("4 = Force Control Mode");
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
   else if (mode == 4) {
      currentMode = MODE_FORCE;
      Serial.println("Force Control Mode Selected.");
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
   resetForcePID();
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

      motor->setHalfDelay(2);
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
            motor->setHalfDelay(3);

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


   // --------------------------------------------------------
   // FORCE CONTROL MODE
   // --------------------------------------------------------
   if (currentMode == MODE_FORCE) {

      if (!forceActive) {
         promptForForceTarget();
         return;
      }

      float currentForce = getFilteredForce();

      // Stage 1: approach until contact is detected
      if (!contactDetected) {
         if (currentForce >= CONTACT_FORCE_THRESHOLD_N) {
            contactDetected = true;
            resetForcePID();
            Serial.println("Contact detected. Starting force regulation.");
            return;
         }

         motor->setHalfDelay(6);

         Direction newDir = reverse; // approach direction
         if (newDir != currentDir) {
            motor->setDirection(newDir);
            currentDir = newDir;
         }

         motor->step();

         static unsigned long lastApproachPrintTime = 0;
         unsigned long nowApproachPrint = millis();

         if (nowApproachPrint - lastApproachPrintTime >= 100) {
            Serial.print("Approaching... Force: ");
            Serial.print(currentForce);
            Serial.print(" N | Target: ");
            Serial.print(targetForce);
            Serial.println(" N");

            lastApproachPrintTime = nowApproachPrint;
         }

         return;
      }

      // Stage 2: closed-loop force regulation
      float forceError = targetForce - currentForce;

      if (abs(forceError) <= FORCE_TOLERANCE_N) {
         motor->setHalfDelay(6);

         Serial.print("Force reached at: ");
         Serial.print(currentForce);
         Serial.print(" N (Target: ");
         Serial.print(targetForce);
         Serial.println(" N)");
         Serial.println();

         forceActive = false;
         contactDetected = false;
         inToleranceBand = false;
         return;
      }
      else {
         inToleranceBand = false;
      }

      unsigned long now = millis();
      float dt = (now - forceLastTime) / 1000.0;

      if (dt <= 0.0) {
         dt = 0.001;
      }

      forceLastTime = now;

      forceIntegral += forceError * dt;
      forceIntegral = constrain(forceIntegral, -FORCE_INTEGRAL_LIMIT, FORCE_INTEGRAL_LIMIT);

      float forceDerivative = (forceError - forcePrevError) / dt;
      forcePrevError = forceError;

      float forceOutput = (KpForce * forceError) + (KiForce * forceIntegral) + (KdForce * forceDerivative);
      float absForceOutput = abs(forceOutput);

      if (absForceOutput < FORCE_MIN_OUTPUT) {
         return;
      }

      int halfDelay = forceOutputToHalfDelay(absForceOutput);
      motor->setHalfDelay(halfDelay);

      // If more force is needed, move deeper into contact.
      // If force is too high, back off.
      Direction newDir = (forceOutput >= 0) ? reverse : forward;

      if (newDir != currentDir) {
         motor->setDirection(newDir);
         currentDir = newDir;
      }

      motor->step();

      static unsigned long lastPrintTimeForce = 0;
      unsigned long nowPrintForce = millis();

      if (nowPrintForce - lastPrintTimeForce >= 100) {
         Serial.print("Force: ");
         Serial.print(currentForce);
         Serial.print(" N | Target: ");
         Serial.print(targetForce);
         Serial.print(" N | Error: ");
         Serial.println(forceError);

         lastPrintTimeForce = nowPrintForce;
      }

      return;
   }

}