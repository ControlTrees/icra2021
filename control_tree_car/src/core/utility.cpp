#include <control_tree/core/utility.h>
#include <eigen3/Eigen/Dense>


double rand_01()
{
    return (double(rand()) / RAND_MAX);
}

double rand_m11()
{
    return 2 * (double(rand()) / RAND_MAX - 0.5);
}

double draw_p(double median_p)
{
    double exponent = log(median_p) / log(0.5);
    return pow(rand_01(), exponent);
}

bool draw_bool(double average_p)
{
    return (rand_01() < average_p);
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

bool near(const Pose2D & a, const Pose2D & b, double eps)
{
    bool close = true;

    close = close && fabs(a.x-b.x) < eps;
    close = close && fabs(a.y-b.y) < eps;

    return close;
}

Pose2D project_on_trajectory(const Pose2D & p, std::vector<Pose2D> trajectory, int & index, double & mu)
{
    using namespace Eigen;

    index = -1;
    mu = -1;

    int I = 0; // number of passed valid points
    for(auto i = 0; i < trajectory.size()-1; ++i)
    {
        const auto a = trajectory[i];
        const auto b = trajectory[i+1];

        if(near(a, b))  // identical points, might be the prefix
            continue;

        Vector2D n {sin(p.yaw), -cos(p.yaw)};
        Vector2D ab {b.x - a.x, b.y - a.y};

        Matrix2f A;
        A << n.x, a.x - b.x,
             n.y, a.y - b.y;

        Vector2f B;
        B << a.x - p.x, a.y - p.y;

        if(fabs(A.determinant()) > 0.00001)
        {
            Vector2f S = A.inverse() * (B);
            double lambda = S[0];
            double _mu = S[1];

            if(0.0 <= _mu && _mu < 1.0
                    || I == 0 && _mu <= 1.0 // hack to make sure it works if the furst points were skipped
                    || i == trajectory.size() - 2 && 0 <= _mu) // proj found
            {
                index = i;
                mu = _mu;
                return {a.x + mu * ab.x, a.y + mu * ab.y, (1.0 - mu) * a.yaw + mu * b.yaw};
            }
        }

        ++I;
    }

    return Pose2D{std::nan(""), std::nan(""), std::nan("")};
}

double dist(const Position2D& a, const Position2D& b)
{
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}
