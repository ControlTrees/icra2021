#pragma once

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>

class Objective;

struct OdometryState
{
    double x{0};
    double y{0};
    double yaw{0};
    double v{0};
    double omega{0};
};

struct Position2D
{
    double x;
    double y;
};

class Evaluator
{
public:
    void acc(double val)
    {
        if(!std::isnan(val))
        {
            avg = (avg * N + val) / (N+1);
            N++;
        }
    }

    double average() const { return avg; };

private:
    double avg = 0;
    int N = 0;
};

double rand_01();
double rand_m11();
OdometryState odometry_state_from_msg(const nav_msgs::Odometry::ConstPtr& msg);
double get_yaw_from_quaternion(const geometry_msgs::Quaternion & quat);
double calculateDifferenceBetweenAngles(double target, double source);
