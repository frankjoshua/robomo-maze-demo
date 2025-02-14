#include <Arduino.h>
#include <Zumo32U4.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "Odometry.h"

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

// Add state machine enum
enum DriveState {
    DRIVE_FORWARD,
    TURN_LEFT,
    COMPLETE_TURN,
    RESET_STATE
};

DriveState currentState = DRIVE_FORWARD;
const float targetDistance = 0.2; // meters
const float turnAngle = PI/2; // 90 degrees in radians
int sideCount = 0;

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
    odometry.reset();
}

void loop() {
    // Read encoder data
    Encoder::EncoderData encoderData;
    encoder.readEncoders(encoderData);

    // Calculate velocities from encoder readings
    Kinematics::velocities vel = kinematics.getVelocities(
        encoderData.rpm.left,
        encoderData.rpm.right,
        0,  // Only using 2 motors
        0   // Only using 2 motors
    );

    // Update position
    Odometry::Position pos = odometry.calculatePosition(vel.linear_x, vel.angular_z);

    // State machine for square movement
    switch(currentState) {
        case DRIVE_FORWARD:
            motors.setSpeeds(100, 100);
            if (pos.x >= targetDistance) {
                motors.setSpeeds(0, 0);
                currentState = TURN_LEFT;
                odometry.reset();
            }
            break;

        case TURN_LEFT:
            motors.setSpeeds(-100, 100);
            if (pos.theta >= turnAngle) {
                motors.setSpeeds(0, 0);
                currentState = COMPLETE_TURN;
                odometry.reset();
            }
            break;

        case COMPLETE_TURN:
            sideCount++;
            if (sideCount >= 4) {
                currentState = RESET_STATE;
                sideCount = 0;
            } else {
                currentState = DRIVE_FORWARD;
            }
            break;

        case RESET_STATE:
            motors.setSpeeds(0, 0);
            delay(1000);
            odometry.reset();
            currentState = DRIVE_FORWARD;
            break;
    }

    // Display position on LCD
    lcd.clear();
    lcd.print(F("X:"));
    lcd.print(pos.x);
    lcd.gotoXY(0, 1);
    lcd.print(F("Y:"));
    lcd.print(pos.y);

    // Delay before the next loop iteration
    delay(50);
}