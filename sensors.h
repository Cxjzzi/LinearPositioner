#pragma once
#include <Arduino.h>

class CurrentSensor {
public:
    CurrentSensor(int outputPin);
    float readCurrent();
private:
    int m_outputPin;
};

class IRDistanceSensor {
public:
    IRDistanceSensor(int outputPin);
    float readDistance();
private:
    int m_outputPin;
};

class LightSensor {
public:
    LightSensor(int outputPin);
    bool isDark();
private:
    int m_outputPin;
};