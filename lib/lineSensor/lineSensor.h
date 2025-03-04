#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Zumo32U4.h>

class LineSensor {
public:
    // Structure to hold sensor values
    struct SensorValues {
        uint16_t left;      // Leftmost sensor
        uint16_t leftMid;   // Left-middle sensor
        uint16_t center;    // Center sensor
        uint16_t rightMid;  // Right-middle sensor
        uint16_t right;     // Rightmost sensor
    };

    LineSensor();
    
    // Initialize the sensors
    void init();
    
    // Calibrate the sensors
    void calibrate();
    
    // Read raw sensor values
    void readRaw(SensorValues& values);
    
    // Read calibrated sensor values (0-1000)
    void readCalibrated(SensorValues& values);
    
    // Get line position (-1 to +1, where 0 is centered)
    float getLinePosition();  // Changed from int16_t to float
    
    // Check if any sensor detects the line
    bool isLineDetected();

private:
    Zumo32U4LineSensors sensors;
    unsigned int sensorValues[5];
    const uint16_t LINE_THRESHOLD = 500;  // Threshold for detecting line (calibrated value)
};

#endif // LINE_SENSOR_H