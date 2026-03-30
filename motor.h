#pragma once
#include <Arduino.h>

enum StepMode { full, half, quarter, eighth, sixteenth };

// Expanded state machine for safe reversing
enum MotorState {
    STOPPED,
    ACCELERATING,
    CRUISING,
    DECELERATING,
    REVERSING_DECEL,   // NEW: decelerating before reversing
    REVERSING_ACCEL,   // NEW: accelerating after direction flip
    EMERGENCY_STOP
};

class Motor {
public:
    Motor(int pinMS1, int pinMS2, int pinMS3, int pinSTEP, int pinDIR,
          int fullStepsPerRev);

    void setStepMode(StepMode mode);
    void setDirection(bool clockwise);
    void setTargetSpeed(float stepsPerSec);
    void setAcceleration(float accel);
    void enable(bool state);
    void emergencyStop();
    void reverse();     // now triggers safe reverse sequence
    void update();      // non‑blocking stepper engine
    bool isRunning() const;

    // NEW: Getters for debugging
    float getTargetSpeed() const { return m_targetSpeed; }
    float getCurrentSpeed() const { return m_currentSpeed; }
    bool getDirection() const { return m_direction; }
    MotorState getState() const { return m_state; }

private:
    void step();
    void applyStepMode();

    int m_pinMS1, m_pinMS2, m_pinMS3;
    int m_pinSTEP, m_pinDIR;

    int m_fullStepsPerRev;
    int m_stepsPerRev;

    MotorState m_state;

    bool m_enabled;
    bool m_direction;

    float m_targetSpeed;
    float m_currentSpeed;
    float m_accel;

    unsigned long m_lastStepMicros;
    unsigned long m_stepInterval;
};