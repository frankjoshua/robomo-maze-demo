#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

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
    void readIMU(IMUData& data);
    void readAccelerometer(Vector& accel);
    void readGyroscope(Vector& gyro);
    void readMagnetometer(Vector& mag);
    void calibrateGyro();
    void calibrateAccel();
    void calibrateMag();
    void setGyroRange(int range);
    void setAccelRange(int range);
    void setMagRange(int range);

private:
    // Calibration offsets
    Vector gyroOffset;
    Vector accelOffset;
    Vector magOffset;  
    
    // Scale factors
    float gyroScale;
    float accelScale;
    float magScale;
    
    // Status flags
    bool initialized;
    bool calibrated;
    bool magCalibrated; 
};

#endif // IMU_H