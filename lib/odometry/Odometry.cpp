#include "Odometry.h"
#include "Arduino.h"

Odometry::Position position;
long lastUpdate = 0;

Odometry::Odometry() 
{
    // Initialize member variables
    position.x = 0.0;
    position.y = 0.0;
    position.theta = 0.0;
    lastUpdate = millis();
}

Odometry::~Odometry()
{
}

Odometry::Position Odometry::calculatePosition(double xLinearVelocity, double zAngularVelocity)
{
    double millisSinceLastUpdate = millis() - lastUpdate;
    lastUpdate = millis();
    double deltaTime = millisSinceLastUpdate / 1000.0;

    double dx = xLinearVelocity;
    double dtheta = zAngularVelocity;
    
    // Update position
    position.x += cos(position.theta) * dx * deltaTime;
    position.y += sin(position.theta) * dx * deltaTime;
    position.theta += dtheta * deltaTime;
    
    // Normalize angle between -PI and PI
    if (position.theta > PI)
        position.theta -= 2 * PI;
    else if (position.theta < -PI)
        position.theta += 2 * PI;

    return position;
}

void Odometry::reset()
{
    position.theta = 0.0;
    position.x = 0.0;
    position.y = 0.0;
    lastUpdate = millis();
}