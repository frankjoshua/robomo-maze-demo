#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Zumo32U4.h>

class Motor {
public:

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
    
    // Update motor speeds based on current and desired velocities
    void updateVelocities(const VelocityCommand& current, const VelocityCommand& desired);
    
private:
    Zumo32U4Motors motors;
    MotorSpeeds speeds;
    
    // Maximum values
    const int16_t MAX_SPEED = 400;    // Maximum motor speed
    
    // Helper functions
    void calculateMotorSpeeds(float linearOutput, float angularOutput, MotorSpeeds& speeds);
};

#endif // MOTOR_H