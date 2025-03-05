#include "Motor.h"

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
    leftSpeed = constrain(leftSpeed * MAX_SPEED / MAX_LINEAR, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed * MAX_SPEED / MAX_LINEAR, -MAX_SPEED, MAX_SPEED);
    
    speeds.left = static_cast<int16_t>(leftSpeed);
    speeds.right = static_cast<int16_t>(rightSpeed);
}

void Motor::updateVelocities(const VelocityCommand& current, const VelocityCommand& desired) {
    // Calculate errors
    float linearError = desired.linear_x - current.linear_x;
    float angularError = desired.angular_z - current.angular_z;
    
    // Simple proportional control
    // You might want to tune these constants based on your robot's performance
    const float KP_LINEAR = 0.1;
    const float KP_ANGULAR = 0.1;
    
    // Calculate outputs
    float linearOutput = KP_LINEAR * linearError;
    float angularOutput = KP_ANGULAR * angularError;
    
    // Limit the outputs to maximum values
    linearOutput = constrain(linearOutput, -MAX_LINEAR, MAX_LINEAR);
    angularOutput = constrain(angularOutput, -MAX_ANGULAR, MAX_ANGULAR);
    
    // Convert PID outputs to motor speeds
    calculateMotorSpeeds(linearOutput, angularOutput, speeds);
    
    // Apply speeds to motors
    motors.setSpeeds(speeds.left, speeds.right);
}