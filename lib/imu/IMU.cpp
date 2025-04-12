#include "IMU.h"
#include "MadgwickAHRS.h"

Madgwick madgwickFilter;

Zumo32U4IMU imuWrapper; // Use the global IMU object

const int HZ = 20;
const unsigned long interval_us = 1000000 / HZ;

IMU::IMU() {
    gyroOffset = {0.0, 0.0, 0.0}; 
    accelOffset = {0.0, 0.0, 0.0};
    magOffset = {0.0, 0.0, 0.0};
}

bool IMU::init() {
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
        // Serial.print(F("Accel: "));
        // Serial.print(imuWrapper.a.x);
        // Serial.print(F(", "));
        // Serial.print(imuWrapper.a.y);
        // Serial.print(F(", "));
        // Serial.println(imuWrapper.a.z);
        delay(10);
    }
    accelOffset.x = sum.x / samples;
    accelOffset.y = sum.y / samples;
    accelOffset.z = sum.z / samples;
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
}

static unsigned long lastTime = 0;

// float getYawDeg(float mx, float my) {
//     float yaw = atan2(-my, mx);  // Note the negative sign on my
//     float yaw_deg = yaw * (180.0f / M_PI);
//     if (yaw_deg < 0) {
//         yaw_deg += 360.0f;
//     }
//     return yaw_deg;
// }

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

    // Serial.print(F(", Yaw: "));

    // Serial.println(getYawDeg(mx, my));

    // Accelerometer: X = forward, Y = right, Z = down
    float ax_ned =  ax;
    float ay_ned = -ay;  // flip Y
    float az_ned = -az;  // flip Z

    // Gyroscope: X = forward, Y = right, Z = down
    float gx_ned =  gx;
    float gy_ned = -gy;  // flip Y
    float gz_ned = -gz;  // flip Z

    // Magnetometer: X = forward, Y = right, Z = down
    float mx_ned =  mx;
    float my_ned = -my;  // flip Y
    float mz_ned = -mz;  // flip Z

    // ------------------------------
    // Update Madgwick filter
    // ------------------------------

    madgwickFilter.update(gx_ned, gy_ned, gz_ned, ax_ned, ay_ned, az_ned, 0, 0, 0);

    
    // Get Euler angles in degrees
    roll = madgwickFilter.getRoll();   // Already converted to degrees
    pitch = madgwickFilter.getPitch(); // Already converted to degrees 
    yaw = (madgwickFilter.getYaw() + 180.0);     // Already converted to degrees
    if(yaw > 360) {
        yaw -= 360.0;
    }

    // Serial.print(F("Roll: "));
    // Serial.print(roll);
    // Serial.print(F(", Pitch: "));
    // Serial.print(pitch);
    // Serial.print(F(", Yaw: "));
    // Serial.println(yaw);
}

void IMU::getRollPitchYaw(float* r, float* p, float* y) {
    *r = roll;
    *p = pitch;
    *y = yaw;
}




