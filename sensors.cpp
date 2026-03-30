#include "sensors.h"

CurrentSensor::CurrentSensor(int outputPin)
    : m_outputPin(outputPin)
{
    pinMode(m_outputPin, INPUT);
}

float CurrentSensor::readCurrent() {
    float voltage = analogRead(m_outputPin) * (5.0 / 1023.0);
    return (voltage - 2.5) / 0.185;
}

IRDistanceSensor::IRDistanceSensor(int outputPin)
    : m_outputPin(outputPin)
{
    pinMode(m_outputPin, INPUT);
}

float IRDistanceSensor::readDistance() {
    float voltage = analogRead(m_outputPin) * (5.0 / 1023.0);
    if (voltage <= 0.01) return 9999.0;
    return 13.0 / voltage;
}

LightSensor::LightSensor(int outputPin)
    : m_outputPin(outputPin)
{
    pinMode(m_outputPin, INPUT);
}

bool LightSensor::isDark() {
    return digitalRead(m_outputPin) == LOW;
}