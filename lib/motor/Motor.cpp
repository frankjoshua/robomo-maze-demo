#include "Motor.h"
#include <Arduino.h>

Motor::Motor() 
{

}

void Motor::init() {
    motors.setSpeeds(0, 0);
}


void Motor::calculateMotorSpeeds(float linearOutput, float angularOutput, MotorSpeeds& speeds) {
    // Convert PID outputs to motor speeds
    float leftSpeed = linearOutput - angularOutput;
    float rightSpeed = linearOutput + angularOutput;
    
    // Scale to motor speed range
    leftSpeed = constrain(leftSpeed * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
    
    speeds.left = static_cast<int16_t>(speeds.left + leftSpeed);
    speeds.right = static_cast<int16_t>(speeds.right + rightSpeed);
}

void Motor::updateVelocities(const VelocityCommand& current, const VelocityCommand& desired) {
    // Calculate errors
    float linearError = desired.linear_x - current.linear_x;
    float angularError = desired.angular_z - current.angular_z;
    Serial.print("Linear Error: ");
    Serial.print(linearError);
    Serial.print(", Angular Error: ");
    Serial.println(angularError);
    
    // Simple proportional control
    // You might want to tune these constants based on your robot's performance
    const float KP_LINEAR = 0.1;
    const float KP_ANGULAR = 0.05;
    
    // Calculate outputs
    float linearOutput = KP_LINEAR * linearError;
    float angularOutput = KP_ANGULAR * angularError;
    Serial.print("Linear Output: ");
    Serial.print(linearOutput);
    Serial.print(", Angular Output: ");
    Serial.println(angularOutput);
    
    
    // Convert PID outputs to motor speeds
    calculateMotorSpeeds(linearOutput, angularOutput, speeds);
    
    // Apply speeds to motors
    motors.setSpeeds(speeds.left, speeds.right);
    Serial.print("Left Speed: ");
    Serial.print(speeds.left);
    Serial.print(", Right Speed: ");
    Serial.println(speeds.right);
}