#pragma once

#include <vector>

#include <nav_msgs/Path.h>

class BehaviorManager;

struct TimeCostPair
{
    double planning_time;
    double cost;
};

class BehaviorBase
{
public:
    BehaviorBase(BehaviorManager & manager)
        : manager_(manager)
        , steps_(4)
    {};

    virtual TimeCostPair plan() = 0;
    virtual std::vector<nav_msgs::Path> get_trajectories() = 0;

protected:
    // params
    const uint steps_;

    // state
    BehaviorManager & manager_;
};



