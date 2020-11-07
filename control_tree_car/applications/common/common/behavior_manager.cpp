#include <math.h>
#include <chrono>
#include <memory>

#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <common/behavior_manager.h>
#include <common/behavior_base.h>


BehaviorManager::BehaviorManager()
    : odo_received_(false)
{
}

void BehaviorManager::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("odometry_callback..");

    // retrieve pose
    odometry_ = odometry_state_from_msg(msg);

    odo_received_ = true;

    //ROS_ERROR(" x, y, yaw: [%f] [%f] [%f]", odometry_.x, odometry_.y, odometry_.yaw);
    //ROS_ERROR(" v, omega: [%f] [%f]", odometry_.v, odometry_.omega);
}


void BehaviorManager::plan()
{
    //ROS_INFO( "plan.." );

    if(!odo_received_)
    {
        ROS_INFO( "no odometry received, abort planning.." );
        return;
    }

    const auto & time_cost = current_behavior_->plan();

    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_).count() / 1e6;
    last_ = now;

    //std::cout << "dt:" << dt << std::endl;

    ++n_plan_; // discard very few iterations since the vehicle may not have attained cruise speed which can bias costs
    if(n_plan_ >=100)
    {
        cost_evaluator_.acc(time_cost.cost, dt);
        velocity_evaluator_.acc(odometry_.v, dt);
        time_evaluator_.acc(time_cost.planning_time);
    }
}

std::vector<nav_msgs::Path> BehaviorManager::get_trajectories() const
{
    //ROS_INFO( "get_trajectories.." );

    return current_behavior_->get_trajectories();
}

