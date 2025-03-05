#include <Arduino.h>
#include <Zumo32U4.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "Odometry.h"
#include <IMU.h>
#include <lineSensor.h>
#include <Map.h>
#include <Motor.h>

// Define constants in meters
const float wheelDiameter_m = 0.032;  // 32mm in meters
const double wheelRadius_m = wheelDiameter_m / 2.0;
const float wheelCircumference_m = PI * wheelDiameter_m;
const int motorMaxRPM = 325;  // Actual max RPM for Zumo32U4
const float wheelsXDistance_m = 0.098;  // 98mm in meters
const float wheelsYDistance_m = 0.098;  // 98mm in meters
const long ticksPerRevolution = 900;  // 12 counts per revolution * 75:1 gear ratio

const int mapWidth = 200;
const int mapHeight = 200;
const int startX = 100;
const int startY = 100;
const float cellSize = 0.01;

// Define loop period (100Hz = 10ms)
const unsigned long LOOP_PERIOD_MS = 10;

// Instantiate objects
Motor motors;
// Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, 
                     motorMaxRPM, 
                     wheelDiameter_m,
                     wheelsXDistance_m, 
                     wheelsYDistance_m);
Encoder encoder(wheelRadius_m, ticksPerRevolution);
Odometry odometry;
IMU robotIMU;
Kinematics::velocities fusedVel;
Kinematics::velocities imuVel;
Kinematics::velocities encoderVel;
LineSensor lineSensor;
Motor::VelocityCommand motorGoalVel;
Motor::VelocityCommand motorCurrentVel;
Map mapInstance;
long lastLCDUpdate = 0;
LineSensor::SensorValues lineValues;
// Zumo32U4ButtonA buttonA;


void callibrate(){
    // Stop motors during calibration
    // motors.setSpeeds(0, 0);
    
    // Display calibration message
    // lcd.clear();
    // lcd.print(F("Cal..."));
    // lcd.gotoXY(0, 1);
    // lcd.print(F("IMU..."));
    
    // Run the calibration routine
    robotIMU.calibrateAccel();
    robotIMU.calibrateGyro();
    robotIMU.calibrateMag();
    
    // Confirmation message
    // lcd.clear();
    // lcd.print(F("Calibration"));
    // lcd.gotoXY(0, 1);
    // lcd.print(F("Complete!"));
    
    delay(1000); // Show message for 1 second
}

void setup() {
    motorGoalVel.linear_x = 0.25;
    motorGoalVel.angular_z = 0;
    // Initialize the LCD
    // lcd.init();
    // lcd.clear();
    // lcd.print(F("Hello"));
    // lcd.gotoXY(0, 1);
    // lcd.print(F("Ready"));
    // delay(1000);

    if (!robotIMU.init()) {
        // lcd.clear();
        // lcd.print(F("IMU Failed"));
        while(1);
    }
    // callibrate();
    lineSensor.init();
    lineSensor.calibrate();
    mapInstance.init(mapWidth, mapHeight);
}

void loop() {
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
    kinematics.getVelocities(
        encoderVel,
        encoderData.rpm.left,
        encoderData.rpm.right,
        0,  // Only using 2 motors
        0   // Only using 2 motors
    );

    // Check if A button is pressed to trigger calibration
    // if (buttonA.getSingleDebouncedPress()) {
    //     callibrate();
    // }

    kinematics.fuseVelocities(0.98, encoderVel, encoderVel, fusedVel);
    Odometry::Position pos = odometry.calculatePosition(fusedVel.linear_x, fusedVel.angular_z);
    motorCurrentVel.linear_x = fusedVel.linear_x;
    motorCurrentVel.angular_z = fusedVel.angular_z;
    Serial.print("Motor Vel: ");
    Serial.print(motorCurrentVel.linear_x);
    Serial.print(", ");
    Serial.println(motorCurrentVel.angular_z);
    Serial.print("Motor Goal Vel: ");
    Serial.print(motorGoalVel.linear_x);
    Serial.print(", ");
    Serial.println(motorGoalVel.angular_z);
    
    lineSensor.readRaw(lineValues);
    // convert pos to map location
    int x = (int) (pos.x / cellSize) + startX;
    int y = (int) (pos.y / cellSize) + startY;
    if(lineValues.center > 100){
        mapInstance.set(x, y, 1);
    } else {
        mapInstance.set(x, y, 0);
    }
    
    if(millis() - lastLCDUpdate > 500){
        lastLCDUpdate = millis();
        motors.updateVelocities(motorCurrentVel, motorGoalVel);
        // Serial.print(x);
        // Serial.print(", ");
        // Serial.print(y);
        // Serial.print(", ");
        // Serial.println(mapInstance.get(x, y));
    }

}

