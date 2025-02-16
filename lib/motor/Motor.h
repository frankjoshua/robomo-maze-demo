#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Zumo32U4.h>

class Motor {
public:
    struct PIDGains {
        float kp;
        float ki;
        float kd;
    };

    struct MotorSpeeds {
        int16_t left;
        int16_t right;
    };

    struct VelocityCommand {
        float linear_x;    // m/s
        float angular_z;   // rad/s
    };

    Motor();
    void init();
    
    // Set PID gains for velocity control
    void setLinearPIDGains(float kp, float ki, float kd);
    void setAngularPIDGains(float kp, float ki, float kd);
    
    // Update motor speeds based on current and desired velocities
    void updateVelocities(const VelocityCommand& current, const VelocityCommand& desired);
    
    // Get current motor speeds
    MotorSpeeds getMotorSpeeds() const;
    
private:
    Zumo32U4Motors motors;
    
    // PID controllers for linear and angular velocity
    PIDGains linearPID;
    PIDGains angularPID;
    
    // Error tracking
    float linearErrorSum;
    float angularErrorSum;
    float lastLinearError;
    float lastAngularError;
    
    // Last update time
    unsigned long lastUpdateTime;
    
    // Maximum values
    const int16_t MAX_SPEED = 400;    // Maximum motor speed
    const float MAX_LINEAR = 0.5;      // m/s
    const float MAX_ANGULAR = PI;      // rad/s
    
    // Helper functions
    void calculateMotorSpeeds(float linearOutput, float angularOutput, MotorSpeeds& speeds);
    float updatePID(float error, float& errorSum, float& lastError, const PIDGains& gains, float dt);
};

#endif // MOTOR_H