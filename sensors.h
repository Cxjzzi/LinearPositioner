#pragma once

#include <Arduino.h>

// Functions for using current sensor
class CurrentSensor {
public:
    CurrentSensor(int outputPin);
    float readCurrent();

private:
    int m_outputPin;
};

// Functions for using IR distance sensor
class IRDistanceSensor {
public:
    IRDistanceSensor(int outputPin);
    float readDistance();

private:
    int m_outputPin;
};

// Functions for using light sensor
class LightSensor {
public:
    LightSensor(int outputPin);
    bool isDark();

private:
    int m_outputPin;
};
