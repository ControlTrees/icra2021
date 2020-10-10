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
#include <control_tree/ros/obstacle_common.h>
#include <control_tree/ros/common.h>

// TODO
// constraints lateral instead of centerline?

// params
double lane_width = 3.5;
const double reset_x_threshold = -0;
const double distance_ahead = 35;
//const double vanishing_false_positive_distance = 10.0;

static int n_obstacles = 0;
static int n_non_obstacles = 0;

//static double steadily_increasing(double signed_distance)
//{
//    const auto distance = signed_distance < 0 ? 0 : signed_distance;

//    auto p = (distance_ahead - distance) / distance_ahead;

//    return p;
//}

//static double steadily_decreasing(double signed_distance)
//{
//    const auto distance = signed_distance < 0 ? 0 : signed_distance;

//    auto p = (std::max(0.0, distance - vanishing_false_positive_distance)) / distance_ahead;

//    return p;
//}

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
        , ref_xs_(N)
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

            if(obstacle.get())
            {
                const auto obstacle_position = obstacle->get_position();
                const auto signed_dist_to_obstacle = obstacle_position.x - car_position.x;
                const auto existence_probability = obstacle->existence_probability(signed_dist_to_obstacle);
                const double x = obstacle_position.x;
                const double y = obstacle_position.y;
//                const double sx = 2.0;
//                const double sy = 1.0;
//                const double sz = 1.5;

                const double sx = 4.0;
                const double sy = 2.0;
                const double sz = 1.5;

                // real obstacle
                visualization_msgs::Marker obstacle_marker = create_obstacle_marker(x,
                                                                             y,
                                                                             sx,
                                                                             sy,
                                                                             sz,
                                                                             existence_probability,
                                                                             obstacle->id_);

                // collision - model position and geometry
                visualization_msgs::Marker collision_marker = create_collision_marker(x,
                                                                               y,
                                                                               sx,
                                                                               sy,
                                                                               sz - 0.5 - 0.5 * i,
                                                                               existence_probability,
                                                                               obstacle->id_);

                obstacle_marker.header.stamp = collision_marker.header.stamp = ros::Time::now();

                markers.markers.push_back(collision_marker);
                markers.markers.push_back(obstacle_marker);

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

    std::vector<std::shared_ptr<Obstacle>> obstacles() const { return obstacles_; }

    void erase_obstacle(uint i)
    {
        obstacles_[i] = nullptr;
    }

    void set_obstacle(uint i, const std::shared_ptr<Obstacle> & obstacle)
    {
        obstacles_[i] = obstacle;
        ref_xs_[i] = obstacle->get_position().x;
    }

    double ref_x(uint i) const
    {
        return ref_xs_[i];
    }

private:
    tf::TransformListener & tf_listener_;
    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    std::vector<double> ref_xs_;

    // params
    const double scale_noise_;
};

static std::shared_ptr<Obstacle> draw_new_obstacle(uint obstacle_id,
                                                   double ref_x,
                                                   double p_obstacle,
                                                   const ObstacleObserver& observer,
                                                   ros::NodeHandle& n,
                                                   std::vector<ros::Publisher> & new_pose_publishers,
                                                   tf::TransformListener& tf_listener)
{
    // draw new OBSTACLE
    std::shared_ptr<Obstacle> obstacle;

    bool position_valid = false;
    Position2D new_position;

    while(!position_valid)
    {
        // X
        const double new_x = ref_x + distance_ahead + rand_m11() * distance_ahead * 0.3;

        // Y
        //const double new_y = rand_m11() * lane_width * 0.5;
        // motorbike
        //const double new_y = rand_m11() > 0 ?  lane_width * 0.5 - 0.6 * rand_01() : -lane_width * 0.5 + 0.6 * rand_01() ;

        // car
        const double road_width = lane_width + 2.0;
        const double y = road_width  * 0.5 + (0.4 - 1.1 * rand_01());
        const double new_y = rand_m11() > 0 ?  y : -y;

        new_position = Position2D{new_x, new_y};

        position_valid = true;
        for(const auto& o: observer.obstacles())
        {
            if(o.get())
            {
                const auto& other = o->get_position();

                const auto d = dist(new_position, other);

                position_valid = position_valid && d > 8.0;
            }
        }
    }

    // P
    const double p = draw_p(p_obstacle);
    const double certainty_distance = 13 + distance_ahead * rand_01();

    if(draw_bool(p_obstacle))
    {
        //ROS_INFO_STREAM("CREATE TRUE POSITIVE..");
        n_obstacles++;
        obstacle = std::shared_ptr<Obstacle>(new TruePositive(obstacle_id, new_position, p, certainty_distance, tf_listener) );
    }
    else
    {
        //ROS_INFO_STREAM("CREATE FALSE POSITIVE..");
        n_non_obstacles++;
        obstacle = std::shared_ptr<Obstacle>(new FalsePositive(obstacle_id, new_position, p, certainty_distance) );
    }

    if(!obstacle->is_false_positive())
    {
        geometry_msgs::Pose2D msg;
        msg.x = new_position.x;
        msg.y = new_position.y;

        new_pose_publishers[obstacle_id].publish(msg);
    }

    n.setParam("/n_total_obstacles", n_obstacles);
    n.setParam("/n_total_non_obstacles", n_non_obstacles);

    return obstacle;
}


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
    n.getParam("road_width", lane_width);


    std::vector<ros::Publisher> new_pose_publishers;
    for(auto i = 0; i < N; ++i)
    {
        new_pose_publishers.push_back(n.advertise<geometry_msgs::Pose2D>("/lgp_obstacle_" + std::to_string(i) + "/pose_reset", 1000));
    }
    ros::Publisher marker_publisher = n.advertise<visualization_msgs::MarkerArray>("/lgp_obstacle_belief/marker_array", 1000);

    ros::Rate loop_rate(10);

    // loop variables
    ObstacleObserver observer(tf_listener, N);

    Position2D car_position, previous_position;
    double speed = 0;

    while(ros::ok())
    {
        // observe car and reset if necessary
        try
        {
            //ROS_INFO_STREAM("................");

            car_position = observer.get_car_position();
            speed = (car_position.x - previous_position .x) * 10;
            previous_position = car_position;

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

//                    ROS_INFO_STREAM("speed:" << speed << " signed_dist_to_obstacle:" << signed_dist_to_obstacle);

//                    if(speed < 0.1 && signed_dist_to_obstacle < 3.5) // avoid car stuck to obstacle
//                    {
//                        ROS_WARN_STREAM("Erase obstacle because car seems stuck");
//                        observer.erase_obstacle(i);
//                    }
                }
            }

            // recreate new
            for(auto i = 0; i < N; ++i)
            {
                if(!observer.obstacle(i).get())
                {
                    auto obstacle = draw_new_obstacle(i, observer.ref_x(i), p_obstacle, observer, n, new_pose_publishers, tf_listener);
                    observer.set_obstacle(i, obstacle);
                }
            }

            // publish observation
            auto markers = observer.observe_obstacles();

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
