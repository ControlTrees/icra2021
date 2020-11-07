#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <chrono>

#include <common/utility.h>

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

    double cost() const { return cost_evaluator_.average(); }
    double planning_time() const { return time_evaluator_.average(); }
    double velocity() const { return velocity_evaluator_.average(); }

private:
    // behaviors
    std::map< std::string, std::shared_ptr< BehaviorBase > > behaviors_;
    std::shared_ptr< BehaviorBase > current_behavior_;

    // state
    bool odo_received_;
    OdometryState odometry_;
    uint n_plan_ = 0;
    std::chrono::high_resolution_clock::time_point last_ = std::chrono::high_resolution_clock::now();

    // evaluation
    ContinuousEvaluator cost_evaluator_;
    ContinuousEvaluator velocity_evaluator_;
    Evaluator time_evaluator_;
};
