#include "sensors.h"

// Constructors (initialize pins)
// These be called in the setup() function

CurrentSensor::CurrentSensor(int outputPin) : m_outputPin(outputPin) {
    // Configure pin mode (output from sensor; analog input to Arduino)
    pinMode(m_outputPin, INPUT);
}

FlexSensor::FlexSensor(int outputPin) : m_outputPin(outputPin) {
    // Configure pin mode (output from sensor; analog input to Arduino)
    pinMode(m_outputPin, INPUT);
}

IRDistanceSensor::IRDistanceSensor(int outputPin) : m_outputPin(outputPin) {
    // Configure pin mode (output from sensor; analog input to Arduino)
    pinMode(m_outputPin, INPUT);
}

LightSensor::LightSensor(int outputPin) : m_outputPin(outputPin) {
    // Configure pin mode (output from sensor; digital input to Arduino)
    pinMode(m_outputPin, INPUT);
}

// Functions to take readings from sensors

float CurrentSensor::readCurrent() {
    // Read analog voltage (ADC gives value 0-1023 corresponding to 0-5 V)
    float voltage = analogRead(m_outputPin) * (5.0 / 1023.0);
    // Convert voltage to current in amps using sensor-specific formula
    float current = (voltage - 2.5) / 0.185;
    // Output current in amps
    return current;
}

int FlexSensor::readValue() {
    // Output raw value from ADC (0-1023)
    return analogRead(m_outputPin);
}

float IRDistanceSensor::readDistance() {
    // Read analog voltage (ADC gives value 0-1023 corresponding to 0-5 V)
    float voltage = analogRead(m_outputPin) * (5.0 / 1023.0);
    // Convert voltage to distance in cm using sensor-specific formula
    float distance = 13.877 * pow(voltage, -0.921);
    // Output distance in cm
    return distance;
}

float IRDistanceSensor::readAverageVoltage(int numSamples) {
    float totalVoltage = 0.0;
    for (int i = 1; i <= numSamples; i++) {
        totalVoltage += analogRead(m_outputPin) * (5.0 / 1023.0);
        delay(10);  // Short delay between samples
    }
    return totalVoltage / numSamples;
}

bool LightSensor::isLight() {
    // Read digital value (0 or 1)
    int value = digitalRead(m_outputPin);
    // Return true if it's light (sensor outputs LOW when light, HIGH when dark)
    return value == LOW;
}
