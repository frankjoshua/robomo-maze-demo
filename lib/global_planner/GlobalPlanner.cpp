#include "GlobalPlanner.h"

GlobalPlanner::GlobalPlanner()
    : waypointCount_(0), currentWaypointIndex_(0)
{
    // Initialize with empty waypoint list
}

void GlobalPlanner::planPath(const Map& map, const Pose& start, const Pose& goal)
{
    // Placeholder for path planning algorithm
    // For now, just add the start and goal as waypoints
    addWaypoint(start);
    addWaypoint(goal);
}

bool GlobalPlanner::addWaypoint(const Pose& waypoint)
{
    if (waypointCount_ >= MAX_WAYPOINTS) {
        return false;  // Array is full
    }
    
    // Add the waypoint to the array
    waypoints_[waypointCount_] = waypoint;
    waypointCount_++;
    
    return true;  // Successfully added
}

bool GlobalPlanner::getNextWaypoint(Pose& waypoint)
{
    // Check if there are any waypoints and if we haven't exceeded the array bounds
    if (waypointCount_ == 0 || currentWaypointIndex_ >= waypointCount_) {
        return false;  // No waypoint available
    }
    
    // Get the current waypoint
    waypoint = waypoints_[currentWaypointIndex_];
    
    // Move to the next waypoint for the next call
    currentWaypointIndex_++;
    
    return true;  // Successfully got waypoint
}

bool GlobalPlanner::hasMoreWaypoints() const
{
    // Return true if there are more waypoints after the current index
    return (waypointCount_ > 0) && (currentWaypointIndex_ < waypointCount_);
}