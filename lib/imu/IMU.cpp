#include "IMU.h"


Zumo32U4IMU imuWrapper; // Use the global IMU object

IMU::IMU() {
    initialized = false;
    calibrated = false;
    gyroScale = 8.75f / 1000.0f;  // Default to ±245 dps
    accelScale = 0.061f;          // Default to ±2g
    gyroOffset = {0.0, 0.0, 0.0}; 
    accelOffset = {0.0, 0.0, 0.0};
}

bool IMU::init() {
    Serial.begin(9600);
    Wire.begin();
    if (!imuWrapper.init()) {
        return false;
    }
    
    imuWrapper.enableDefault();
    imuWrapper.configureForTurnSensing();
    
    setGyroRange(2000);
    setAccelRange(8);
    
    initialized = true;
    return true;
}

void IMU::readIMU(IMUData& data) {
    if (!initialized) return;
    
    readAccelerometer(data.accelerometer);
    readGyroscope(data.gyroscope);
}

void IMU::readAccelerometer(Vector& accel) {
    imuWrapper.readAcc();
    accel.x = (imuWrapper.a.x - accelOffset.x) * accelScale;
    accel.y = (imuWrapper.a.y - accelOffset.y) * accelScale;
    accel.z = (imuWrapper.a.z - accelOffset.z) * accelScale;

    // Serial.print("Accel X: ");
    // Serial.print(accel.x);
    // Serial.print(", Accel Y: ");
    // Serial.print(accel.y);
    // Serial.print(", Accel Z: ");
    // Serial.println(accel.z);

}

void IMU::readGyroscope(Vector& gyro) {
    imuWrapper.readGyro();
    
    // Debug raw values
    float raw_z = imuWrapper.g.z;
    float offset_z = raw_z;// - gyroOffset.z; // Cast to float before subtraction
    float scaled_z = offset_z * gyroScale;
    float final_z = -scaled_z * (PI / 180.0);
    
    // Serial.print("gyroOffset.z: "); Serial.println(gyroOffset.z);

    // Serial.print("Raw Z: ");
    // Serial.print(raw_z);
    // Serial.print(", Offset Z: ");
    // Serial.print(offset_z);
    // Serial.print(", Scaled Z: ");
    // Serial.print(scaled_z);
    // Serial.print(", Final Z: ");
    // Serial.println(final_z);
    // Serial.println(imuWrapper.g.x);
    // Serial.println(imuWrapper.g.y);

    gyro.z = final_z;
}

void IMU::calibrateGyro() {
    const int samples = 100;
    Vector sum = {0.0, 0.0, 0.0};
    
    // Take multiple samples
    for(int i = 0; i < samples; i++) {
        imuWrapper.readGyro();
        sum.x += imuWrapper.g.x;
        sum.y += imuWrapper.g.y;
        sum.z += imuWrapper.g.z;
        delay(10);
    }
    
    // Calculate average offset
    gyroOffset.x = sum.x / samples;
    gyroOffset.y = sum.y / samples;
    gyroOffset.z = sum.z / samples;
    
    calibrated = true;
}

void IMU::calibrateAccel() {
    // Simple calibration assuming level surface
    imuWrapper.readAcc();
    accelOffset.x = imuWrapper.a.x;
    accelOffset.y = imuWrapper.a.y;
    accelOffset.z = imuWrapper.a.z; // Offset for 1g (assuming ±2g range)
}

void IMU::setGyroRange(int range) {
    // Scale factors for degrees per second
    switch(range) {
        case 245:  gyroScale = 8.75f/1000.0f; break;  // ±245 dps
        case 500:  gyroScale = 17.5f/1000.0f; break;  // ±500 dps
        case 2000: gyroScale = 70.0f/1000.0f; break;  // ±2000 dps
        default:   gyroScale = 8.75f/1000.0f; break;  // Default ±245 dps
    }
}

void IMU::setAccelRange(int range) {
    // Set accelerometer range and update scale factor
    switch(range) {
        case 2:  accelScale = 0.061f/1000.0f; break;  // ±2g
        case 4:  accelScale = 0.122f/1000.0f; break;  // ±4g
        case 8:  accelScale = 0.244f/1000.0f; break;  // ±8g
        case 16: accelScale = 0.488f/1000.0f; break;  // ±16g
        default: accelScale = 0.061f/1000.0f; break;  // Default ±2g
    }
}