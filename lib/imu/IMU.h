#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "MadgwickAHRS.h"

class IMU {
public:
    struct Vector {
        float x;
        float y;
        float z;
    };

    struct IMUData {
        Vector accelerometer;
        Vector gyroscope;
        Vector magnetometer; 
    };

    IMU();
    bool init();
    void calibrateGyro();
    void calibrateAccel();
    void calibrateMag();
    void getRollPitchYaw(float* r, float* p, float* y);
    void readSensorData();

private:
    float roll;
    float pitch;
    float yaw;
    Madgwick madgwickFilter;

    // Calibration offsets
    Vector gyroOffset;
    Vector accelOffset;
    Vector magOffset;  
    
};

#endif // IMU_H