#pragma once

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <control_tree/core/utility.h>

class BehaviorBase;

class BehaviorManager
{

public:
    BehaviorManager();

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void plan();
    std::vector<nav_msgs::Path> get_trajectories() const;

    void register_behavior(const std::string & key, const std::shared_ptr< BehaviorBase > & behavior)
    {
        behaviors_[key] = behavior;
    }

    void set_current_behavior(const std::string & key)
    {
        current_behavior_ = behaviors_[key];
    }

    OdometryState odometry() const
    {
        return odometry_;
    }

    double cost() const { return cost_evaluator.average(); }
    double planning_time() const { return time_evaluator.average(); }
    double velocity() const { return velocity_evaluator.average(); }

private:
    // behaviors
    std::map< std::string, std::shared_ptr< BehaviorBase > > behaviors_;
    std::shared_ptr< BehaviorBase > current_behavior_;

    // state
    bool odo_received_;
    OdometryState odometry_;

    // evaluation
    Evaluator cost_evaluator;
    Evaluator time_evaluator;
    Evaluator velocity_evaluator;
};
