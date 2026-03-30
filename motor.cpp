#include "motor.h"

// Constructor (initializes motor control pins and parameters)
// Must be called in the setup() function
Motor::Motor(int pinMS1, int pinMS2, int pinMS3, int pinSTEP, int pinDIR,
             int fullStepsPerRev)
    : m_pinMS1(pinMS1)
    , m_pinMS2(pinMS2)
    , m_pinMS3(pinMS3)
    , m_pinSTEP(pinSTEP)
    , m_pinDIR(pinDIR)
    , m_fullStepsPerRev(fullStepsPerRev) {
    // Configure pin modes
    pinMode(m_pinMS1, OUTPUT);
    pinMode(m_pinMS2, OUTPUT);
    pinMode(m_pinMS3, OUTPUT);
    pinMode(m_pinSTEP, OUTPUT);
    pinMode(m_pinDIR, OUTPUT);
    // Ensure STEP pin starts LOW
    digitalWrite(m_pinSTEP, LOW);
    // Set default micro-stepping mode
    setStepMode(full);
    // Set default half-delay
    setHalfDelay(1);
    // Set default direction
    setDirection(forward);
}

// Sets micro-stepping mode
void Motor::setStepMode(StepMode mode) {
    switch (mode) {
    case full:
        digitalWrite(m_pinMS1, LOW);
        digitalWrite(m_pinMS2, LOW);
        digitalWrite(m_pinMS3, LOW);
        m_stepsPerRev = m_fullStepsPerRev;
        break;
    case half:
        digitalWrite(m_pinMS1, HIGH);
        digitalWrite(m_pinMS2, LOW);
        digitalWrite(m_pinMS3, LOW);
        m_stepsPerRev = m_fullStepsPerRev * 2;
        break;
    case quarter:
        digitalWrite(m_pinMS1, LOW);
        digitalWrite(m_pinMS2, HIGH);
        digitalWrite(m_pinMS3, LOW);
        m_stepsPerRev = m_fullStepsPerRev * 4;
        break;
    case eighth:
        digitalWrite(m_pinMS1, HIGH);
        digitalWrite(m_pinMS2, HIGH);
        digitalWrite(m_pinMS3, LOW);
        m_stepsPerRev = m_fullStepsPerRev * 8;
        break;
    case sixteenth:
        digitalWrite(m_pinMS1, HIGH);
        digitalWrite(m_pinMS2, HIGH);
        digitalWrite(m_pinMS3, HIGH);
        m_stepsPerRev = m_fullStepsPerRev * 16;
        break;
    default:  // Default to full step
        digitalWrite(m_pinMS1, LOW);
        digitalWrite(m_pinMS2, LOW);
        digitalWrite(m_pinMS3, LOW);
        m_stepsPerRev = m_fullStepsPerRev;
    }
}

// Sets direction of rotation
void Motor::setDirection(Direction dir) {
    delay(250);  // Short delay to allow motor to stop before changing direction
    digitalWrite(m_pinDIR, dir == forward ? LOW : HIGH);
}

// Sets delay between steps (controls speed)
void Motor::setHalfDelay(int halfDelay) { m_halfDelay = halfDelay; }

// Rotates motor one full revolution
void Motor::rotate() {
    for (int i = 0; i < m_stepsPerRev; i++) {
        digitalWrite(m_pinSTEP, HIGH);
        delay(m_halfDelay);
        digitalWrite(m_pinSTEP, LOW);
        delay(m_halfDelay);
    }
}
