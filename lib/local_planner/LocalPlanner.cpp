#include "LocalPlanner.h"
#include <math.h>
#include <Arduino.h>


LocalPlanner::LocalPlanner(double max_linear_vel, double max_angular_vel)
    : max_linear_velocity_(max_linear_vel), max_angular_velocity_(max_angular_vel) {
    // Constructor implementation
}

void LocalPlanner::computeVelocity(const Pose& current_pose, const Pose& goal_pose, VelocityCommand& cmd) {
    // Calculate the difference in position
    double dx = goal_pose.x - current_pose.x;
    double dy = goal_pose.y - current_pose.y;

    // Calculate the distance to the goal
    double distance = sqrt(dx * dx + dy * dy);

    // Calculate the angle to the goal
    double angle_to_goal = atan2(dy, dx);

    // Calculate the difference in angle
    double dtheta = angle_to_goal - current_pose.theta;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(", Angle: ");
    Serial.print(dtheta);
    Serial.print("Current Pose: ");
    Serial.print(current_pose.x);
    Serial.print(", ");
    Serial.print(current_pose.y);
    Serial.print(", ");
    Serial.println(current_pose.theta);

    // Stop if close enough to the goal
    if (distance < 0.01) {
        cmd.linear = 0;
        cmd.angular = 0;
        return;
    }

    const double angle_threshold = 0.1; // adjust as needed

    // Normalize the angle to the range [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;
    
    if (fabs(dtheta) > angle_threshold) {
        // If not sufficiently aligned, turn in place
        cmd.linear = 0;
        cmd.angular = max(-max_angular_velocity_, min(dtheta, max_angular_velocity_));
    } else {
        // Once nearly aligned, move forward while adjusting angular velocity
        cmd.linear = min(distance, max_linear_velocity_);
        cmd.angular = max(-max_angular_velocity_, min(dtheta, max_angular_velocity_));
    }
}