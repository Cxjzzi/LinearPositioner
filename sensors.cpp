#include "sensors.h"

// Constructors (initialize pins)
// These be called in the setup() function

CurrentSensor::CurrentSensor(int outputPin) : m_outputPin(outputPin) {
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

float IRDistanceSensor::readDistance() {
    // Read analog voltage (ADC gives value 0-1023 corresponding to 0-5 V)
    float voltage = analogRead(m_outputPin) * (5.0 / 1023.0);
    // Convert voltage to distance in cm using sensor-specific formula
    float distance = 30 * pow(voltage, -1.173);
    // Output distance in cm
    return distance;
}

bool LightSensor::isDark() {
    // Read digital value (0 or 1)
    int value = digitalRead(m_outputPin);
    // Return true if it's dark (assuming sensor outputs LOW when dark)
    return (value == LOW);
}
