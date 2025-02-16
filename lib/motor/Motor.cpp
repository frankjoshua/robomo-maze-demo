#include "Motor.h"

Motor::Motor() : 
    linearErrorSum(0.0f),
    angularErrorSum(0.0f),
    lastLinearError(0.0f),
    lastAngularError(0.0f),
    lastUpdateTime(0)
{
    // Default PID gains
    linearPID = {1.0f, 0.1f, 0.05f};   // Kp, Ki, Kd
    angularPID = {1.0f, 0.1f, 0.05f};  // Kp, Ki, Kd
}

void Motor::init() {
    motors.setSpeeds(0, 0);
    lastUpdateTime = millis();
}

void Motor::setLinearPIDGains(float kp, float ki, float kd) {
    linearPID = {kp, ki, kd};
}

void Motor::setAngularPIDGains(float kp, float ki, float kd) {
    angularPID = {kp, ki, kd};
}

void Motor::updateVelocities(const VelocityCommand& current, const VelocityCommand& desired) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f; // Convert to seconds
    lastUpdateTime = currentTime;

    // Calculate errors
    float linearError = desired.linear_x - current.linear_x;
    float angularError = desired.angular_z - current.angular_z;

    // Update PID controllers
    float linearOutput = updatePID(linearError, linearErrorSum, lastLinearError, linearPID, dt);
    float angularOutput = updatePID(angularError, angularErrorSum, lastAngularError, angularPID, dt);

    // Calculate and apply motor speeds
    MotorSpeeds speeds;
    calculateMotorSpeeds(linearOutput, angularOutput, speeds);
    motors.setSpeeds(speeds.left, speeds.right);
}

float Motor::updatePID(float error, float& errorSum, float& lastError, const PIDGains& gains, float dt) {
    // Integrate error
    errorSum += error * dt;
    
    // Calculate derivative
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    // PID output
    return gains.kp * error + gains.ki * errorSum + gains.kd * derivative;
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

Motor::MotorSpeeds Motor::getMotorSpeeds() const {
    // Return current motor speeds
    MotorSpeeds speeds;
    // Note: Zumo32U4Motors doesn't provide a way to read current speeds
    // This would need to be tracked separately if needed
    return speeds;
}