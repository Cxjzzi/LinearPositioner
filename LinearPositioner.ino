#include "config.h"
#include "motor.h"
#include "sensors.h"

// ------------------------------------------------------------
// User-adjustable variables
// ------------------------------------------------------------
bool continuousMode = false;
float targetPosition = 30.0;

// ------------------------------------------------------------
// Global objects
// ------------------------------------------------------------
CurrentSensor* currentSensor = nullptr;
FlexSensor* flexSensor = nullptr;
IRDistanceSensor* irSensor = nullptr;
LightSensor* lightSensor = nullptr;
Motor* motor = nullptr;

bool isStopped = false;
bool positionReached = false;

// ------------------------------------------------------------
// Serial input helpers
// ------------------------------------------------------------
int readIntFromSerial() {
   while (!Serial.available()) {
   }
   return Serial.parseInt();
}

float readFloatFromSerial() {
   while (!Serial.available()) {
   }
   return Serial.parseFloat();
}

// NEW: Clears leftover characters like \r and \n
void clearSerialBuffer() {
   while (Serial.available() > 0) {
      Serial.read();
   }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
   Serial.begin(9600);
   delay(500);

   Serial.println("=== Linear Positioner System ===");
   Serial.println("Select Mode:");
   Serial.println("1 = Continuous Mode");
   Serial.println("2 = Target Position Mode");
   Serial.print("Enter choice: ");

   int mode = readIntFromSerial();

   if (mode == 1) {
      continuousMode = true;
      Serial.println("Continuous Mode Selected.");
   } else if (mode == 2) {
      continuousMode = false;
      Serial.println("Target Position Mode Selected.");

      // *** FIX: Clear leftover characters BEFORE asking for target ***
      clearSerialBuffer();

      Serial.print("Enter target distance in cm: ");
      targetPosition = readFloatFromSerial();

      Serial.print("Target set to: ");
      Serial.println(targetPosition);

      positionReached = false;
      isStopped = false;
   } else {
      Serial.println("Invalid choice. Defaulting to Continuous Mode.");
      continuousMode = true;
   }

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
   motor->setHalfDelay(1);
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
   if (continuousMode) {
      float distance = irSensor->readDistance();

      if (distance < 10.0) {
         motor->setDirection(reverse);
      } else if (distance > 30.0) {
         motor->setDirection(forward);
      }

      motor->rotate();

      float current = currentSensor->readCurrent();
      Serial.print("Current: ");
      Serial.println(current);

      return;
   }

   // --------------------------------------------------------
   // TARGET POSITION MODE
   // --------------------------------------------------------
   if (!positionReached) {
      float error = irSensor->readDistance() - targetPosition;

      if (error >= 0)
         motor->setDirection(forward);
      else
         motor->setDirection(reverse);

      int revsToMove = (int)(abs(error) / DIST_PER_REV);

      for (int i = 0; i < revsToMove; i++) {
         if (digitalRead(PUSH_BUTTON)) {
            isStopped = true;
            break;
         }
         if (!isStopped) motor->rotate();
      }

      while (abs(error) > 0.5) {
         if (digitalRead(PUSH_BUTTON)) {
            isStopped = true;
            break;
         }

         if (!isStopped) {
            error = irSensor->readDistance() - targetPosition;

            if (error >= 0)
               motor->setDirection(forward);
            else
               motor->setDirection(reverse);

            motor->rotate();
         }
      }

      positionReached = true;
   }
}
