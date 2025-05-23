#include <Arduino.h>
#include <Zumo32U4.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "Odometry.h"
#include <IMU.h>
#include <lineSensor.h>
#include <Map.h>
#include <Motor.h>
#include <LocalPlanner.h>
#include <GlobalPlanner.h>

// Define constants in meters
const float wheelDiameter_m = 0.032;  // 32mm in meters
const double wheelRadius_m = wheelDiameter_m / 2.0;
const float wheelCircumference_m = PI * wheelDiameter_m;
const int motorMaxRPM = 325;  // Actual max RPM for Zumo32U4
const float wheelsXDistance_m = 0.098;  // 98mm in meters
const float wheelsYDistance_m = 0.098;  // 98mm in meters
const long ticksPerRevolution = 900;  // 12 counts per revolution * 75:1 gear ratio

const int mapWidth = 45;
const int mapHeight = 45;
const int startX = 0;
const int startY = 0;
const float cellSize = 0.02;

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
Kinematics::velocities goalVel;
LineSensor lineSensor;
LineSensor::SensorValues lineValues;
Motor::VelocityCommand motorGoalVel;
Motor::VelocityCommand motorCurrentVel;
unsigned char grid[(mapWidth * mapHeight) / 8];
Map mapInstance(grid, mapWidth, mapHeight);
LocalPlanner localPlanner(0.2, 1.5, 0.025);
LocalPlanner::Pose currentPose;
LocalPlanner::Pose goalPose;
GlobalPlanner globalPlanner;


LocalPlanner::VelocityCommand localPlannerVel;
long lastLCDUpdate = 0;

Zumo32U4ButtonA buttonA;


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
    delay(1000);
    Serial.begin(115200);
    
    bool a = true;
    while (a)
    {
        Serial.println(F("Move the robot in a figure 8 to calibrate the IMU"));
    Serial.println(F("Then set on flat surface and press A to calibrate"));
        if(buttonA.getSingleDebouncedPress()){
            a = false;
        }
        delay(100);
    }
    
    
    // motorGoalVel.linear_x = 0.1;
    // motorGoalVel.angular_z = 0;
    goalPose.x = 0.0;
    goalPose.y = 0.0;

    float distance = 0.25;
    globalPlanner.addWaypoint({distance, 0.0, 0});
    globalPlanner.addWaypoint({distance, distance, 0});
    globalPlanner.addWaypoint({0.0, distance, 0});
    globalPlanner.addWaypoint({0.0, 0.0, 0});
    globalPlanner.addWaypoint({distance, 0.0, 0});
    globalPlanner.addWaypoint({distance, distance, 0});
    globalPlanner.addWaypoint({0.0, distance, 0});
    globalPlanner.addWaypoint({0.0, 0.0, 0});
    globalPlanner.addWaypoint({distance, 0.0, 0});
    globalPlanner.addWaypoint({distance, distance, 0});
    globalPlanner.addWaypoint({0.0, distance, 0});
    globalPlanner.addWaypoint({0.0, 0.0, 0});
    // globalPlanner.planPath(mapInstance, {0.0, 0.0, 0}, {distance, 0.0, 0});
    // Add waypoints to create a grid pattern
    // for (int i = 0; i < mapWidth; i++) {
    //     globalPlanner.addWaypoint({(double) i * cellSize, mapHeight * cellSize, 0});
    //     globalPlanner.addWaypoint({(double) 0, 0, 0});
    // }

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
        // Serial.println("IMU Failed");
        while(1);
    }
    callibrate();
    lineSensor.init();
    lineSensor.calibrate();
    mapInstance.init(mapWidth, mapHeight);
    // delay(3000);

    // // create diagonal line
    // for (int i = 0; i < mapWidth; i++) {
    //     mapInstance.set(i, i, 1);
    // }


    
}

void loop() {
    // Serial.println("Looping");
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

    goalVel.linear_x = motorGoalVel.linear_x;
    goalVel.angular_z = motorGoalVel.angular_z;
    kinematics.fuseVelocities(0.0, encoderVel, goalVel, fusedVel);
    // Odometry::Position pos = odometry.calculatePosition(fusedVel.linear_x, fusedVel.angular_z);
    float yaw = 0;
    robotIMU.getRollPitchYaw(0, 0, &yaw);
    // convert yaw to radians
    yaw = yaw * (PI / 180.0);
    // convert yaw to right hand rule
    yaw = -yaw;
    // Serial.print("Yaw: ");
    // Serial.println(yaw);

    Odometry::Position pos = odometry.calculatePositionWithYaw(fusedVel.linear_x, yaw);
    Serial.print("Pos: ");
    Serial.print(pos.x);
    Serial.print(", ");
    Serial.println(pos.y);
    motorCurrentVel.linear_x = fusedVel.linear_x;
    motorCurrentVel.angular_z = fusedVel.angular_z;
    // Serial.print("Motor Vel: ");
    // Serial.print(motorCurrentVel.linear_x);
    // Serial.print(", ");
    // Serial.println(motorCurrentVel.angular_z);
    // Serial.print("Motor Goal Vel: ");
    // Serial.print(motorGoalVel.linear_x);
    // Serial.print(", ");
    // Serial.println(motorGoalVel.angular_z);
    
    lineSensor.readRaw(lineValues);
    // convert pos to map location
    int x = (int) (pos.x / cellSize) + startX;
    int y = (int) (pos.y / cellSize) + startY;
    if(lineValues.center > 100){
        mapInstance.set(x, y, 1);
    } else {
        mapInstance.set(x, y, 0);
    }

    currentPose.x = pos.x;
    currentPose.y = pos.y;
    currentPose.theta = pos.theta;
    bool atGoal = localPlanner.computeVelocity(currentPose, goalPose, localPlannerVel);
    if(atGoal){
        Serial.println("At goal");
        motors.stop();
        delay(1000);
                // Serial print the map
                // Serial.println("Map:");
                // for (int y = 0; y < 200; y++) {
                //     for (int x = 0; x < 200; x++) {
                //         Serial.print(mapInstance.get(x, y));
                //     }
                //     Serial.println();
                // }
        bool moreWaypoints = globalPlanner.getNextWaypoint(goalPose);
        if(!moreWaypoints){
            // Serial.println("Finished");
            motors.stop();
            while (1)
            {
                if(buttonA.getSingleDebouncedPress()){
        
                // Serial.println("Map:");
                // for (int y = 0; y < mapHeight; y++) {
                //     for (int x = 0; x < mapWidth; x++) {
                //         Serial.print(mapInstance.get(x, y));
                //     }
                //     Serial.println();
                // }
            }
                delay(10);
            }        
        }
    }
    motorGoalVel.linear_x = localPlannerVel.linear;
    motorGoalVel.angular_z = localPlannerVel.angular;
    
    motors.updateVelocities(motorCurrentVel, motorGoalVel);
    if(millis() - lastLCDUpdate > 100){
        lastLCDUpdate = millis();
        // Print current position to lcd
        // lcd.clear();
        // lcd.gotoXY(0, 0);
        // lcd.print("X: ");
        // lcd.print(currentPose.x);
        // lcd.gotoXY(0, 1);
        // lcd.print("Y: ");
        // lcd.print(currentPose.y);
        
        // Serial.print("Motor current vel: ");
        // Serial.print(motorCurrentVel.linear_x);
        // Serial.print(", ");
        // Serial.println(motorCurrentVel.angular_z);
        // Serial.print("Motor goal vel: ");
        // Serial.print(motorGoalVel.linear_x);
        // Serial.print(", ");
        // Serial.println(motorGoalVel.angular_z);
        // Serial.print(x);
        // Serial.print(", ");
        // Serial.print(y);
        // Serial.print(", ");
        // Serial.println(mapInstance.get(x, y));
    }

}

