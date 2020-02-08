#include <control_tree/core/utility.h>


double rand_01()
{
    return (double(rand()) / RAND_MAX);
}

double rand_m11()
{
    return 2 * (double(rand()) / RAND_MAX - 0.5);
}

OdometryState odometry_state_from_msg(const nav_msgs::Odometry::ConstPtr& msg)
{
    OdometryState odometry;

    odometry.x = msg->pose.pose.position.x;
    odometry.y = msg->pose.pose.position.y;

    tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odometry.yaw = yaw;

    // retrieve speeds
    odometry.v = msg->twist.twist.linear.x;
    odometry.omega = msg->twist.twist.angular.z;

    return odometry;
}

double get_yaw_from_quaternion(const geometry_msgs::Quaternion & quat)
{
    tf::Quaternion q(
                quat.x,
                quat.y,
                quat.z,
                quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

double calculateDifferenceBetweenAngles(double target, double source)
{
    double difference = target - source;
    while (difference < -M_PI) difference += 2 * M_PI;
    while (difference > M_PI) difference -= 2 * M_PI;
    return difference;
}

