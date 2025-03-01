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

// Define loop period (100Hz = 10ms)
const unsigned long LOOP_PERIOD_MS = 10;

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
Zumo32U4ButtonA buttonA;


void callibrate(){
    // Stop motors during calibration
    motors.setSpeeds(0, 0);
    
    // Display calibration message
    lcd.clear();
    lcd.print("Calibrating");
    lcd.gotoXY(0, 1);
    lcd.print("IMU...");
    
    // Run the calibration routine
    robotIMU.calibrateAccel();
    robotIMU.calibrateGyro();
    robotIMU.calibrateMag();
    
    // Confirmation message
    lcd.clear();
    lcd.print("Calibration");
    lcd.gotoXY(0, 1);
    lcd.print("Complete!");
    
    delay(1000); // Show message for 1 second
}

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
    callibrate();
}

void loop() {
    unsigned long start_time = millis();  // Get start time

    // Read encoder data
    Encoder::EncoderData encoderData;
    encoder.readEncoders(encoderData);

    robotIMU.readSensorData();
    // // Get IMU data
    // IMU::IMUData imuData;
    // robotIMU.readIMU(imuData);
    // kinematics.fuseIMU(
    //   imuData.accelerometer.x, imuData.accelerometer.y, imuData.accelerometer.z,
    //   imuData.gyroscope.x, imuData.gyroscope.y, imuData.gyroscope.z,
    //   0, 0, 0,
    //   &imuVel);
    // // Kinematics::velocities imuVel = kinematics.getVelocitiesFromIMU(imuData.accelerometer.x, imuData.accelerometer.y, imuData.gyroscope.z);
    
    // Calculate velocities using both encoder and IMU data
    Kinematics::velocities vel = kinematics.getVelocities(
        encoderData.rpm.left,
        encoderData.rpm.right,
        0,  // Only using 2 motors
        0   // Only using 2 motors
    );


    // Update position
    Odometry::Position pos = odometry.calculatePosition(imuVel.linear_x, imuVel.angular_z);

    // Check if A button is pressed to trigger calibration
    if (buttonA.getSingleDebouncedPress()) {
        callibrate();
    }

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
    
    // if(millis() - lastLCDUpdate > 500){
    //     lastLCDUpdate = millis();
    //     lcd.clear();
    //     lcd.print(F("I:"));
    //     lcd.print(imuVel.angular_z);
    //     lcd.gotoXY(0, 19);
    //     lcd.print(F("O:"));
    //     lcd.print(fusedVel.angular_z);
    // }
    
    unsigned long elapsed_time = millis() - start_time;  // Calculate execution time

    // Only delay if we have time remaining
    if (elapsed_time < LOOP_PERIOD_MS) {
        // delay(LOOP_PERIOD_MS - elapsed_time);
    } else {
        // If we don't have time, print a warning
        Serial.print(F("Loop time exceeded! "));
        Serial.println(elapsed_time); 
    }
}

