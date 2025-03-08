#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "LocalPlanner.h"
#include "Map.h"

class GlobalPlanner {
public:
    // Use the same Pose structure as LocalPlanner for consistency
    using Pose = LocalPlanner::Pose;
    
    // Maximum number of waypoints
    static const int MAX_WAYPOINTS = 20;
    
    // Constructor
    GlobalPlanner();

    void planPath(const Map& map, const Pose& start, const Pose& goal);
    
    // Add a waypoint to the path
    // Returns true if waypoint was added successfully, false if array is full
    bool addWaypoint(const Pose& waypoint);
    
    // Get the next waypoint
    // Returns true if there is a next waypoint, false otherwise
    // The waypoint is copied to the provided reference
    bool getNextWaypoint(Pose& waypoint);
    
    // Check if we have more waypoints after the current one
    bool hasMoreWaypoints() const;
    

private:
    Pose waypoints_[MAX_WAYPOINTS];
    int waypointCount_;
    int currentWaypointIndex_;
};

#endif // GLOBAL_PLANNER_H