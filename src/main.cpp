#include <Arduino.h>
#include <Zumo32U4.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "Odometry.h"
#include <IMU.h>

// Define constants in meters
const float wheelDiameter_m = 0.032;  // 32mm in meters
const double wheelRadius_m = wheelDiameter_m / 2.0;
const float wheelCircumference_m = PI * wheelDiameter_m;
const int motorMaxRPM = 325;  // Actual max RPM for Zumo32U4
const float wheelsXDistance_m = 0.098;  // 98mm in meters
const float wheelsYDistance_m = 0.098;  // 98mm in meters
const long ticksPerRevolution = 900;  // 12 counts per revolution * 75:1 gear ratio

// Instantiate objects
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, 
                     motorMaxRPM, 
                     wheelDiameter_m,
                     wheelsXDistance_m, 
                     wheelsYDistance_m);
Encoder encoder(wheelRadius_m, ticksPerRevolution);
Odometry odometry;
IMU robotIMU;
float high_imu = 0;
float high_odom = 0;
Kinematics::velocities fusedVel;
Kinematics::velocities imuVel;
long lastLCDUpdate = 0;

void setup() {
    // Initialize the motors
    motors.setSpeeds(0, 0);

    // Initialize the LCD
    lcd.init();
    lcd.clear();
    lcd.print(F("Hello"));
    lcd.gotoXY(0, 1);
    lcd.print(F("Ready"));
    delay(1000);

    if (!robotIMU.init()) {
        lcd.clear();
        lcd.print(F("IMU Failed"));
        while(1);
    }
    robotIMU.calibrateAccel();
    robotIMU.calibrateGyro();
}

void loop() {
    // Read encoder data
    Encoder::EncoderData encoderData;
    encoder.readEncoders(encoderData);

    // Get IMU data
    IMU::IMUData imuData;
    robotIMU.readIMU(imuData);
    kinematics.fuseIMU(
      imuData.accelerometer.x, imuData.accelerometer.y, imuData.accelerometer.z,
      imuData.gyroscope.x, imuData.gyroscope.y, imuData.gyroscope.z,
      0, 0, 0,
      &imuVel);
    // Kinematics::velocities imuVel = kinematics.getVelocitiesFromIMU(imuData.accelerometer.x, imuData.accelerometer.y, imuData.gyroscope.z);
    
    // Calculate velocities using both encoder and IMU data
    Kinematics::velocities vel = kinematics.getVelocities(
        encoderData.rpm.left,
        encoderData.rpm.right,
        0,  // Only using 2 motors
        0   // Only using 2 motors
    );


    // Update position
    Odometry::Position pos = odometry.calculatePosition(imuVel.linear_x, imuVel.angular_z);

    if(millis() > 4000){
        motors.setSpeeds(0, 0);
    } else {
        motors.setSpeeds(200, -200);
    }
    
    if(imuVel.angular_z > high_imu){
        high_imu = imuVel.angular_z;
    }
    if(vel.angular_z > high_odom){
        high_odom = vel.angular_z;
    }

    kinematics.fuseVelocities(0.98, vel, imuVel, fusedVel);
    
    if(millis() - lastLCDUpdate > 500){
        lastLCDUpdate = millis();
        lcd.clear();
        lcd.print(F("I:"));
        lcd.print(imuVel.angular_z);
        lcd.gotoXY(0, 19);
        lcd.print(F("O:"));
        lcd.print(fusedVel.angular_z);
    }
    
    // Delay before the next loop iteration
    delay(50);
}