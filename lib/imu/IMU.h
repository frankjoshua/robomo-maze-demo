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
    };

    IMU();
    bool init();
    void readIMU(IMUData& data);
    void readAccelerometer(Vector& accel);
    void readGyroscope(Vector& gyro);
    void calibrateGyro();
    void calibrateAccel();
    void setGyroRange(int range);
    void setAccelRange(uint8_t range);

private:
    // Calibration offsets
    Vector gyroOffset;
    Vector accelOffset;
    
    // Scale factors
    float gyroScale;
    float accelScale;
    
    // Status flags
    bool initialized;
    bool calibrated;
};

#endif // IMU_H