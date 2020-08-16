#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>
#include <stdlib.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>

#include <control_tree/core/utility.h>

// TODO
// improve obstacle randomization?
// constraints lateral instead of centerline?

// params
const double lane_width = 3.5;
const double reset_x_threshold = -5.0;
const double distance_ahead = 28;
const double vanishing_false_positive_distance = 10.0;

static int n_obstacles = 0;
static int n_non_obstacles = 0;

static double steadily_increasing(double signed_distance)
{
    const auto distance = signed_distance < 0 ? 0 : signed_distance;

    auto p = (distance_ahead - distance) / distance_ahead;

    return p;
}

static double steadily_decreasing(double signed_distance)
{
    const auto distance = signed_distance < 0 ? 0 : signed_distance;

    auto p = (std::max(0.0, distance - vanishing_false_positive_distance)) / distance_ahead;

    return p;
}

class Obstacle
{
public:
    Obstacle(uint id)
      : id_(id)
    {

    }

    virtual double existence_probability(double distance) const = 0;
    virtual Position2D get_position() const = 0;
    virtual bool is_false_positive() const = 0;

    uint id_;
};

class FalsePositive : public Obstacle
{
public:
    FalsePositive(uint id, const Position2D & position, double p, double certainty_distance)
    : Obstacle(id)
    , position_(position)
    , p_(p)
    , certainty_distance_(certainty_distance)
    {

    }

    double existence_probability(double distance) const
    {
        if(distance > certainty_distance_ + 2)
        {
            return p_;
        }
        else if(distance < certainty_distance_)
        {
            return 0.0;
        }
        else
        {
            double lambda = (distance - certainty_distance_) / 2;
            return 0.0 * (1 - lambda) + p_ * (lambda);
        }
    }

    Position2D get_position() const { return position_; }

    bool is_false_positive() const { return true; }

private:
    Position2D position_{0,0};
    double p_ = 0.5;
    double certainty_distance_ = 10.0;
};

class TruePositive : public Obstacle
{
public:
    TruePositive(uint id, const Position2D & position, double p, double certainty_distance, tf::TransformListener & tf_listener)
        : Obstacle(id)
        , position_(position)
        , p_(p)
        , certainty_distance_(certainty_distance)
        , tf_listener_(tf_listener)
    {

    }

    double existence_probability(double distance) const
    {
        if(distance > certainty_distance_ + 2)
        {
            return p_;
        }
        else if(distance < certainty_distance_)
        {
            return 1.0;
        }
        else
        {
            double lambda = (distance - certainty_distance_) / 2;
            return 1.0 * (1 - lambda) + p_ * (lambda);
        }

    }

    Position2D get_position() const
    {
        tf::StampedTransform transform;

        tf_listener_.lookupTransform("/map", "/lgp_obstacle_" + std::to_string(id_),
                                     ros::Time(0), transform);

        return Position2D{ transform(tf::Vector3(0,0,0)).x(), transform(tf::Vector3(0,0,0)).y() };
    }

    bool is_false_positive() const { return false; }

private:
    Position2D position_{0,0};
    double p_ = 0.5;
    double certainty_distance_ = 10.0;
    tf::TransformListener & tf_listener_;
};

class ObstacleObserver
{
public:
    ObstacleObserver(tf::TransformListener & tf_listener, int N)
        : tf_listener_(tf_listener)
        , obstacles_(N)
        , scale_bias_(0.75)
        , scale_noise_(0.2)
    {

    }

    visualization_msgs::MarkerArray observe_obstacles() const
    {
        visualization_msgs::MarkerArray markers;

        const auto car_position = get_car_position();

        //ROS_INFO_STREAM("Observe...");

        for(auto i = 0; i < obstacles_.size(); ++i)
        {
            const auto& obstacle = obstacles_[i];

            visualization_msgs::Marker marker;

            if(obstacle.get())
            {
                const auto obstacle_position = obstacle->get_position();
                const auto signed_dist_to_obstacle = obstacle_position.x - car_position.x;
                const auto existence_probability = obstacle->existence_probability(signed_dist_to_obstacle);

                // obstacle position and geometry
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";
                marker.id = obstacle->id_;//std::hash<std::string>()("obstacle");
                marker.type = visualization_msgs::Marker::CYLINDER;//;visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = obstacle_position.x + (1.0 - existence_probability) * (scale_bias_ * bias_.x + scale_noise_ * rand_m11());
                marker.pose.position.y = obstacle_position.y + (1.0 - existence_probability) * (scale_bias_ * bias_.y + scale_noise_ * rand_m11());
                marker.scale.x = 1.0; // diameter
                marker.scale.y = 1.0;
                marker.scale.z = 1.0 + i;
                marker.color.a = existence_probability;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                markers.markers.push_back(marker);

                //ROS_INFO("Obstacle existence probability: %f %d", existence_probability, obstacle->id_);
            }
        }

        //ROS_INFO_STREAM("Finished Observe...");

        return markers;
    }

    Position2D get_position(const std::string & frame_name) const
    {
        tf::StampedTransform transform;

        tf_listener_.lookupTransform("/map", frame_name,
                                     ros::Time(0), transform);

        return Position2D{ transform(tf::Vector3(0,0,0)).x(), transform(tf::Vector3(0,0,0)).y() };
    }

    Position2D get_car_position() const
    {
        return get_position("/lgp_car");
    }

    std::shared_ptr<Obstacle> obstacle(uint i) const { return obstacles_[i]; }

    void reset_bias()
    {
        bias_.x = rand_m11();
        bias_.y = rand_m11();
    }

    void erase_obstacle(uint i)
    {
        obstacles_[i] = nullptr;
    }

    void set_obstacle(uint i, const std::shared_ptr<Obstacle> & obstacle)
    {
        obstacles_[i] = obstacle;
    }

private:
    tf::TransformListener & tf_listener_;
    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    Position2D bias_;

    // params
    const double scale_bias_;
    const double scale_noise_;
};

int main(int argc, char **argv)
{
    srand(0);

    double p_obstacle = 0.1;

    ROS_INFO_STREAM("Launch obstacle control..");

    ros::init(argc, argv, "obstacle_control_control");
    ros::NodeHandle n;
    tf::TransformListener tf_listener;

    int N = 1; // number of obsatcles
    n.getParam("n_obstacles", N);
    n.getParam("p_obstacle", p_obstacle);

    std::vector<ros::Publisher> new_pose_publishers;
    for(auto i = 0; i < N; ++i)
    {
        new_pose_publishers.push_back(n.advertise<geometry_msgs::Pose2D>("/lgp_obstacle_" + std::to_string(i) + "/pose_reset", 1000));
    }
    ros::Publisher marker_publisher = n.advertise<visualization_msgs::MarkerArray>("/lgp_obstacle_belief/marker_array", 1000);

    ros::Rate loop_rate(10);

    // loop variables
    ObstacleObserver observer(tf_listener, N);

    // obstacle creation
    auto draw_new_obstacle = [&](uint obstacle_id)
    {
        // draw new OBSTACLE
        std::shared_ptr<Obstacle> obstacle;

        const auto car_position = observer.get_car_position();
        const double new_x = car_position.x + distance_ahead + rand_m11() * distance_ahead * 0.5;
        //const double new_y = rand_m11() * lane_width * 0.5;
        const double new_y = rand_m11() > 0 ?  lane_width * 0.5 - 0.5 * rand_01() : -lane_width * 0.5 + 0.5 * rand_01() ;
        const double exponent = log(p_obstacle) / log(0.5);
        const double p = pow(rand_01(), exponent);
        const double certainty_distance = 10 + ( distance_ahead - 5 ) * rand_01() * rand_01();

        const double q = rand_01();
        if(q <= p)
        {
            //ROS_INFO_STREAM("CREATE TRUE POSITIVE..");
            n_obstacles++;
            obstacle = std::shared_ptr<Obstacle>(new TruePositive(obstacle_id, {new_x, new_y}, p, certainty_distance, tf_listener) );
        }
        else
        {
            //ROS_INFO_STREAM("CREATE FALSE POSITIVE..");
            n_non_obstacles++;
            obstacle = std::shared_ptr<Obstacle>(new FalsePositive(obstacle_id, {new_x, new_y}, p, certainty_distance) );
        }

        if(!obstacle->is_false_positive())
        {
            geometry_msgs::Pose2D msg;
            msg.x = new_x;
            msg.y = new_y;

            new_pose_publishers[obstacle_id].publish(msg);
        }

        n.setParam("/n_total_obstacles", n_obstacles);
        n.setParam("/n_total_non_obstacles", n_non_obstacles);

        return obstacle;
    };

    while(ros::ok())
    {
        // observe car and reset if necessary
        try
        {
            //ROS_INFO_STREAM("................");

            const auto car_position = observer.get_car_position();

            // purge old
            for(auto i = 0; i < N; ++i)
            {
                if(observer.obstacle(i).get())
                {
                    const auto obstacle_position = observer.obstacle(i)->get_position();
                    const auto signed_dist_to_obstacle = obstacle_position.x - car_position.x;

                    if(signed_dist_to_obstacle < reset_x_threshold)
                    {
                        observer.erase_obstacle(i);
                    }
                }
            }

            // recreate new
            for(auto i = 0; i < N; ++i)
            {
                if(!observer.obstacle(i).get())
                {
                    auto obstacle = draw_new_obstacle(i);
                    observer.set_obstacle(i, obstacle);
                }
            }

            // publish observation
            auto markers = observer.observe_obstacles();
            //marker.id = obstacle_id;

            marker_publisher.publish(markers);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
