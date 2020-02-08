#include <math.h>
#include <chrono>
#include <memory>

#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <control_tree/core/behavior_manager.h>
#include <control_tree/core/behavior_base.h>


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

    cost_evaluator.acc(time_cost.cost);
    time_evaluator.acc(time_cost.planning_time);
    velocity_evaluator.acc(odometry_.v);
}

std::vector<nav_msgs::Path> BehaviorManager::get_trajectories() const
{
    //ROS_INFO( "get_trajectories.." );

    return current_behavior_->get_trajectories();
}

