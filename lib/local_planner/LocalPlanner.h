#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H



class LocalPlanner {

public:
    struct Pose {
        double x;     // meters
        double y;     // meters
        double theta; // radians
    };

    struct VelocityCommand {
        double linear;  // m/s
        double angular; // rad/s
    };
    
    LocalPlanner(double max_linear_vel, double max_angular_vel);

    // Compute velocity directly into provided command
    void computeVelocity(const Pose& current_pose, const Pose& goal_pose, VelocityCommand& cmd);

private:
    double max_linear_velocity_;
    double max_angular_velocity_;
};

#endif // LOCAL_PLANNER_H
