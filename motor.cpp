#include "motor.h"

Motor::Motor(int pinMS1, int pinMS2, int pinMS3, int pinSTEP, int pinDIR,
             int fullStepsPerRev)
    : m_pinMS1(pinMS1), m_pinMS2(pinMS2), m_pinMS3(pinMS3),
      m_pinSTEP(pinSTEP), m_pinDIR(pinDIR),
      m_fullStepsPerRev(fullStepsPerRev),
      m_stepsPerRev(fullStepsPerRev),
      m_enabled(false),
      m_direction(true),
      m_targetSpeed(0),
      m_currentSpeed(0),
      m_accel(200),
      m_lastStepMicros(0),
      m_stepInterval(0),
      m_state(STOPPED)
{
    pinMode(m_pinMS1, OUTPUT);
    pinMode(m_pinMS2, OUTPUT);
    pinMode(m_pinMS3, OUTPUT);
    pinMode(m_pinSTEP, OUTPUT);
    pinMode(m_pinDIR, OUTPUT);

    digitalWrite(m_pinSTEP, LOW);
    applyStepMode();
}

void Motor::applyStepMode() {
    digitalWrite(m_pinMS1, LOW);
    digitalWrite(m_pinMS2, LOW);
    digitalWrite(m_pinMS3, LOW);
}

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
    }
}

void Motor::setDirection(bool clockwise) {
    m_direction = clockwise;
    digitalWrite(m_pinDIR, clockwise ? LOW : HIGH);
}

void Motor::setTargetSpeed(float stepsPerSec) {
    m_targetSpeed = stepsPerSec;

    if (stepsPerSec > 0) {
        m_enabled = true;
        m_state = ACCELERATING;
    }
}

void Motor::setAcceleration(float accel) {
    m_accel = accel;
}

void Motor::enable(bool state) {
    m_enabled = state;
    if (!state) {
        m_state = STOPPED;
        m_currentSpeed = 0;
    }
}

void Motor::emergencyStop() {
    m_state = EMERGENCY_STOP;
    m_currentSpeed = 0;
    m_enabled = false;
}

void Motor::reverse() {
    // Begin safe reverse sequence:
    // 1. Decelerate to zero
    // 2. Flip direction
    // 3. Accelerate back up
    m_state = REVERSING_DECEL;
}

bool Motor::isRunning() const {
    return (m_state != STOPPED && m_state != EMERGENCY_STOP);
}

void Motor::step() {
    digitalWrite(m_pinSTEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(m_pinSTEP, LOW);
}

void Motor::update() {
    if (!m_enabled) return;

    unsigned long now = micros();

    // -----------------------------
    // SAFE REVERSE: DECELERATE
    // -----------------------------
    if (m_state == REVERSING_DECEL) {
        m_currentSpeed -= m_accel * 0.001;
        if (m_currentSpeed <= 0) {
            m_currentSpeed = 0;
            // Flip direction
            setDirection(!m_direction);
            // Begin accelerating in new direction
            m_state = REVERSING_ACCEL;
        }
    }

    // -----------------------------
    // SAFE REVERSE: ACCELERATE
    // -----------------------------
    else if (m_state == REVERSING_ACCEL) {
        m_currentSpeed += m_accel * 0.001;
        if (m_currentSpeed >= m_targetSpeed) {
            m_currentSpeed = m_targetSpeed;
            m_state = CRUISING;
        }
    }

    // -----------------------------
    // NORMAL ACCELERATION
    // -----------------------------
    else if (m_state == ACCELERATING) {
        m_currentSpeed += m_accel * 0.001;
        if (m_currentSpeed >= m_targetSpeed) {
            m_currentSpeed = m_targetSpeed;
            m_state = CRUISING;
        }
    }

    // -----------------------------
    // NORMAL DECELERATION
    // -----------------------------
    else if (m_state == DECELERATING) {
        m_currentSpeed -= m_accel * 0.001;
        if (m_currentSpeed <= 0) {
            m_currentSpeed = 0;
            m_state = STOPPED;
        }
    }

    if (m_currentSpeed <= 0) return;

    m_stepInterval = 1e6 / m_currentSpeed;

    if (now - m_lastStepMicros >= m_stepInterval) {
        m_lastStepMicros = now;
        step();
    }
}