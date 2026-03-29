#pragma once

#include <Arduino.h>

// Micro-stepping modes
enum StepMode { full, half, quarter, eighth, sixteenth };

// Functions for controlling stepper motor
class Motor {
public:
    Motor(int pinMS1, int pinMS2, int pinMS3, int pinSTEP, int pinDIR,
          int fullStepsPerRev);
    void setStepMode(StepMode mode);
    void setHalfDelay(int halfDelay);
    void setDirection(bool clockwise);
    void rotate();

private:
    int m_pinMS1;
    int m_pinMS2;
    int m_pinMS3;
    int m_pinSTEP;
    int m_pinDIR;
    int m_fullStepsPerRev;
    int m_stepsPerRev;
    int m_halfDelay;
};
