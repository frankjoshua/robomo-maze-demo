#include "IMU.h"
#include "MadgwickAHRS.h"

Madgwick madgwickFilter;

Zumo32U4IMU imuWrapper; // Use the global IMU object

const int HZ = 100;
const unsigned long interval_us = 1000000 / HZ;

IMU::IMU() {
    gyroOffset = {0.0, 0.0, 0.0}; 
    accelOffset = {0.0, 0.0, 0.0};
    magOffset = {0.0, 0.0, 0.0};
}

bool IMU::init() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Wire.begin();
    if (!imuWrapper.init()) {
        return false;
    }
    
    imuWrapper.enableDefault();
    imuWrapper.configureForTurnSensing();

    madgwickFilter.begin(HZ); 
    return true;
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
        delay(5);
    }
    
    // Calculate average offset
    gyroOffset.x = sum.x / samples;
    gyroOffset.y = sum.y / samples;
    gyroOffset.z = sum.z / samples;
    printf("Gyro offset: %f, %f, %f\n", gyroOffset.x, gyroOffset.y, gyroOffset.z);
}

void IMU::calibrateAccel() {
    const int samples = 200;
    Vector sum = {0.0, 0.0, 0.0};
    
    // Take multiple samples
    for(int i = 0; i < samples; i++) {
        imuWrapper.readAcc();
        sum.x += imuWrapper.a.x;
        sum.y += imuWrapper.a.y;
        sum.z += imuWrapper.a.z;
        delay(10);
    }
    accelOffset.x = sum.x / samples;
    accelOffset.y = sum.y / samples;
    accelOffset.z = sum.z / samples;

    printf("Accel offset: %f, %f, %f\n", accelOffset.x, accelOffset.y, accelOffset.z);
}

void IMU::calibrateMag() {
    const int samples = 100;
    Vector min = {32767, 32767, 32767};
    Vector max = {-32768, -32768, -32768};
    
    // Collect samples to find min/max values
    for(int i = 0; i < samples; i++) {
        imuWrapper.readMag();
        min.x = min(min.x, (float)imuWrapper.m.x);
        min.y = min(min.y, (float)imuWrapper.m.y);
        min.z = min(min.z, (float)imuWrapper.m.z);
        max.x = max(max.x, (float)imuWrapper.m.x);
        max.y = max(max.y, (float)imuWrapper.m.y);
        max.z = max(max.z, (float)imuWrapper.m.z);
        delay(10);
    }
    
    // Calculate offsets (center point of min/max)
    magOffset.x = (min.x + max.x) / 2.0f;
    magOffset.y = (min.y + max.y) / 2.0f;
    magOffset.z = (min.z + max.z) / 2.0f;

    printf("Mag offset: %f, %f, %f\n", magOffset.x, magOffset.y, magOffset.z);
}

static unsigned long lastTime = 0;

void IMU::readSensorData() {

    if ((micros() - lastTime) < interval_us) return; 
    
    lastTime += interval_us;

    imuWrapper.read();
    
    // Get raw sensor data
    float ax_raw = imuWrapper.a.x - accelOffset.x;
    float ay_raw = imuWrapper.a.y - accelOffset.y;
    float az_raw = imuWrapper.a.z - accelOffset.z;
    
    float gx_raw = imuWrapper.g.x - gyroOffset.x;
    float gy_raw = imuWrapper.g.y - gyroOffset.y;
    float gz_raw = imuWrapper.g.z - gyroOffset.z;

    float mx_raw = imuWrapper.m.x - magOffset.x;
    float my_raw = imuWrapper.m.y - magOffset.y;
    float mz_raw = imuWrapper.m.z - magOffset.z;

    // Convert raw values to engineering units:
    float ax = ax_raw * (8.0f / 32768.0f);    // ±2g full scale => 1 LSB = 2/32768 g&#8203;:contentReference[oaicite:16]{index=16}
    float ay = ay_raw * (8.0f / 32768.0f);
    float az = az_raw * (8.0f / 32768.0f);
    float gx = gx_raw * (2000.0f / 32768.0f);  // ±250°/s => 1 LSB = 250/32768 °/s&#8203;:contentReference[oaicite:17]{index=17}
    float gy = gy_raw * (2000.0f / 32768.0f);
    float gz = gz_raw * (2000.0f / 32768.0f);
    // Magnetometer (±4 gauss full-scale by default on LIS3MDL -> 1 LSB = 4/32768 gauss):
    float mx = mx_raw * (4.0f / 32768.0f);
    float my = my_raw * (4.0f / 32768.0f);
    float mz = mz_raw * (4.0f / 32768.0f);

    // Update Madgwick filter
    madgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    // Get Euler angles in degrees
    float roll = madgwickFilter.getRoll();   // Already converted to degrees
    float pitch = madgwickFilter.getPitch(); // Already converted to degrees 
    float yaw = madgwickFilter.getYaw();     // Already converted to degrees

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Yaw: ");
    Serial.println(yaw);
}

