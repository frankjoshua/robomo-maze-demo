#include "lineSensor.h"

LineSensor::LineSensor() {
    // Constructor - nothing to initialize here since init() will be called separately
}

void LineSensor::init() {
    sensors.initFiveSensors();
}

void LineSensor::calibrate() {
    // Perform 100 calibration readings
    for(uint16_t i = 0; i < 100; i++) {
        sensors.calibrate();
        delay(10);  // Short delay between readings
    }
}

void LineSensor::readRaw(SensorValues& values) {
    sensors.read(sensorValues, QTR_EMITTERS_ON);
    values.left = sensorValues[0];
    values.leftMid = sensorValues[1];
    values.center = sensorValues[2];
    values.rightMid = sensorValues[3];
    values.right = sensorValues[4];
}

void LineSensor::readCalibrated(SensorValues& values) {
    sensors.readCalibrated(sensorValues);
    values.left = sensorValues[0];
    values.leftMid = sensorValues[1];
    values.center = sensorValues[2];
    values.rightMid = sensorValues[3];
    values.right = sensorValues[4];
}

float LineSensor::getLinePosition() {
    // Read position will return a value between 0 and 4000
    // Convert to range -1.0 to +1.0 where 0 is centered
    return (sensors.readLine(sensorValues) - 2000) / 2000.0f;
}

bool LineSensor::isLineDetected() {
    sensors.readCalibrated(sensorValues);
    // Check if any sensor is above threshold
    for(uint8_t i = 0; i < 5; i++) {
        if(sensorValues[i] > LINE_THRESHOLD) {
            return true;
        }
    }
    return false;
}