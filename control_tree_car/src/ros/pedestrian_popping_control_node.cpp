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

// params
const double reset_x_threshold = -5.0;
const double distance_ahead = 28;
double lane_width = 3.5;

static int n_crossings = 0;
static int n_non_crossings = 0;

class Pedestrian
{
public:
    Pedestrian(uint id, const Position2D & start_position, tf::TransformListener & tf_listener, ros::Publisher & pose_publisher, ros::Publisher & ctrl_publisher)
        : id_(id)
        , start_position_(start_position)
        , tf_listener_(tf_listener)
        , pose_publisher_(pose_publisher)
        , ctrl_publisher_(ctrl_publisher)
    {
        geometry_msgs::Pose2D msg;
        msg.x = start_position.x;
        msg.y = start_position.y;

        pose_publisher_.publish(msg);
    }

    virtual double crossing_probability(double now_s, double now_x) const = 0;
    virtual void step(double now_s, double now_x) = 0;
    virtual bool is_done(double now_s, double now_x) const = 0;
    virtual bool is_forward_direction() const = 0;

    Position2D get_start_position() const
    {
        return start_position_;
    }

    Position2D get_position() const
    {
        tf::StampedTransform transform;

        tf_listener_.lookupTransform("/map", "/lgp_pedestrian_" + std::to_string(id_),
                                     ros::Time(0), transform);

        return Position2D{ transform(tf::Vector3(0,0,0)).x(), transform(tf::Vector3(0,0,0)).y() };
    }

    uint id() const { return id_; }

protected:
    enum State
    {
        UNCERTAIN=0,
        MOVING,
        DONE
    } state_{UNCERTAIN};

    uint id_;
    Position2D start_position_{0,0};
    tf::TransformListener & tf_listener_;
    ros::Publisher & pose_publisher_;
    ros::Publisher & ctrl_publisher_;
};

class CrossingPedestrian : public Pedestrian
{
public:
    CrossingPedestrian(uint id,
                       const Position2D & start_position,
                       double p,
                       double crossing_x,
                       bool forward,
                       tf::TransformListener & tf_listener, ros::Publisher & pose_publisher, ros::Publisher & ctrl_publisher)
    : Pedestrian(id, start_position, tf_listener, pose_publisher, ctrl_publisher)
    , p_(p)
    , crossing_x_(crossing_x)
    , crossing_duration_(4.0)
    , forward_( forward )
    {
        n_crossings++;
        ROS_INFO_STREAM("CrossingPedestrian!! " << crossing_x << " || " << n_crossings << "|" << n_non_crossings);
    }

    bool is_forward_direction() const
    {
        return forward_;
    }

    bool is_crossing(double time, double x)
    {
        if(state_ == State::UNCERTAIN)
        {
            if(x >= crossing_x_)
            {
                state_ = State::MOVING;
                crossing_time_ = time;

                return true;
            }
        }

        if(state_ == State::MOVING)
        {
            if(time < crossing_time_ + crossing_duration_ - 0.5) // hack, restart driving a little bit before!
            {
                return true; // crossing
            }
            else
            {
                state_ = State::DONE;
                return false; // crossing finished
            }
        }

        //ROS_INFO_STREAM("state:" << state_);

        return false;
    }

    double crossing_probability(double time, double x) const
    {
        if(state_ == State::UNCERTAIN)
        {
            return p_;
        }
        else if(state_ == State::MOVING)
        {
            return 1.0; // crossing
        }
        else
        {
            return 1.0; // crossing finished
        }
    }

    void step(double now_s, double now_x)
    {
        geometry_msgs::Twist msg;

        const double speed = (lane_width + 2)/ crossing_duration_;

        if(is_crossing(now_s, now_x))
        {
            msg.linear.y = start_position_.y > 0 ? -speed : speed;
        }

        ctrl_publisher_.publish(msg);
    }

    bool is_done(double now_s, double now_d) const
    {
        return state_ == State::DONE;
    }

private:
    double p_ = 0.5;
    double crossing_time_ = 0.0;
    double crossing_x_ = 0.0;
    double crossing_duration_ = 0.0;
    bool forward_ = true;
};

class NonCrossingPedestrian : public Pedestrian
{
public:
    NonCrossingPedestrian(uint id,
                          const Position2D & start_position,
                          double p,
                          double non_crossing_x,
                          bool forward,
                          tf::TransformListener & tf_listener,
                          ros::Publisher & pose_publisher,
                          ros::Publisher & ctrl_publisher)
    : Pedestrian(id, start_position, tf_listener, pose_publisher, ctrl_publisher)
    , p_(p)
    , non_crossing_x_(non_crossing_x)
    , non_crossing_duration_(4.0)
    , forward_( forward )
    {
        n_non_crossings++;
        ROS_INFO_STREAM("NonCrossingPedestrian!! " << non_crossing_x << " || " << n_crossings << "|" << n_non_crossings);
    }

    bool is_forward_direction() const
    {
        return forward_;
    }

    bool is_non_crossing(double time, double x)
    {
        //ROS_INFO_STREAM("state:" << state_);

        if(state_ == State::UNCERTAIN)
        {
            if(x >= non_crossing_x_)
            {
                state_ = State::MOVING;
                non_crossing_time_ = time;

                return true;
            }
        }

        if(state_ == State::MOVING)
        {
            if(time < non_crossing_time_ + non_crossing_duration_ && x < get_position().x)
            {
                return true; // non crossing
            }
            else
            {
                state_ = State::DONE;
                return false; // non crossing finished
            }
        }

        return false;
    }

    double crossing_probability(double time, double x) const
    {
        if(state_ == State::UNCERTAIN)
        {
            return p_;
        }
        else if(state_ == State::MOVING)
        {
            return 0.0; // non crossing
        }
        else
        {
            return 0.0; // non crossing finished
        }
    }

    void step(double now_s, double now_x)
    {
        geometry_msgs::Twist msg;

        const double speed = ( lane_width + 2 )/ non_crossing_duration_;

        if(is_non_crossing(now_s, now_x))
        {
            msg.linear.x = (forward_ ? 1 : -1) * speed; //(lane_width * 0.5 + 1) / non_crossing_duration_;

           // ROS_INFO_STREAM("move!!" << msg.linear.x);
        }

        ctrl_publisher_.publish(msg);
    }

    bool is_done(double now_s, double now_x) const
    {
        return state_ == State::DONE;
    }

private:
    double p_ = 0.5;
    double non_crossing_time_ = 0.0;
    double non_crossing_x_ = 0.0;
    double non_crossing_duration_ = 0.0;
    bool forward_ = true;
};

class PedestrianObserver
{
public:
    PedestrianObserver(std::size_t N, tf::TransformListener & tf_listener)
        : pedestrians_(N)
        , tf_listener_(tf_listener)
    {

    }

    visualization_msgs::MarkerArray observe_pedestrians() const
    {
        visualization_msgs::MarkerArray markers;

        //ROS_INFO_STREAM("number of pedestrians:"<<pedestrians_.size());

        for(auto i = 0; i < pedestrians_.size(); ++i)
        {
            auto pedestrian = pedestrians_[i];

            if(pedestrian)
            {
                //ROS_INFO("mode: %s",  mode_ == Mode::CORRECT ? "CORRECT" : "FALSE_POSITIVE");
                const auto car_position = get_car_position();
                const auto position = pedestrian->get_position();
                const auto start_position = pedestrian->get_start_position();

                double crossing_probability = pedestrian->crossing_probability(ros::Time::now().toSec(), car_position.x);
                {
                    std::string name = "pedestrian_mesh_" + std::to_string(i);

                    // obstacle position and geometry
                    visualization_msgs::Marker marker;
                    marker.header.stamp = ros::Time::now();
                    marker.header.frame_id = "map";
                    marker.id = std::hash<std::string>()(name);
                    marker.ns = name;
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = position.x;
                    marker.pose.position.y = position.y;
                    marker.pose.position.z = 0.;

                    if(crossing_probability >= 1.0)
                    {
                        marker.mesh_resource = "package://control_tree_car/meshes/person_walking/meshes/walking.dae";
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        marker.scale.y = pedestrian->get_start_position().y > 0 ? 1.0: -1.0;
                    }
                    else if(crossing_probability <= 0.0)
                    {
                        marker.mesh_resource = "package://control_tree_car/meshes/person_walking/meshes/walking.dae";
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;

                        const double yaw = (M_PI / 2.0) * (pedestrian->is_forward_direction() ? 1.0 : -1.0);

                        marker.scale.y = 1.0;

                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        marker.pose.orientation.x = q.x();
                        marker.pose.orientation.y = q.y();
                        marker.pose.orientation.z = q.z();
                        marker.pose.orientation.w = q.w();
                    }
                    else
                    {
                        marker.mesh_resource = "package://control_tree_car/meshes/person_standing/meshes/standing.dae";
                        marker.color.r = 1.0;
                        marker.color.g = 165.0/255;
                        marker.color.b = 0.0;

                        marker.scale.y = 1.0;

                        const double crossing_angle = pedestrian->get_start_position().y > 0 ? 0 : M_PI;
                        const double final_non_crossing_angle = (M_PI / 2.0) * (pedestrian->is_forward_direction() ? 1.0 : -1.0);
                        const double yaw = (1-crossing_probability) * final_non_crossing_angle + crossing_probability * crossing_angle;

                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);

                        marker.pose.orientation.x = q.x();
                        marker.pose.orientation.y = q.y();
                        marker.pose.orientation.z = q.z();
                        marker.pose.orientation.w = q.w();
                    }

                    marker.scale.x = 1.0;
                    marker.scale.z = 1.0;
                    marker.color.a = 1.0;

                    markers.markers.push_back(marker);
                }

                {
                    std::string name = "pedestrian_crossing_prediction_" + std::to_string(i);

                    visualization_msgs::Marker crossing_prediction;
                    crossing_prediction.header.stamp = ros::Time::now();
                    crossing_prediction.header.frame_id = "map";
                    crossing_prediction.id = std::hash<std::string>()(name);
                    crossing_prediction.ns = name;
                    crossing_prediction.type = visualization_msgs::Marker::LINE_STRIP;
                    crossing_prediction.action = visualization_msgs::Marker::ADD;
                    crossing_prediction.pose.position.x = 0;
                    crossing_prediction.pose.position.y = 0;
                    crossing_prediction.scale.x = 0.1; // width

                    crossing_prediction.color.a = crossing_probability;
                    crossing_prediction.color.r = 1.0;
                    crossing_prediction.color.g = 0.0;
                    crossing_prediction.color.b = 0.0;

                    geometry_msgs::Point p;
                    p.x = position.x;
                    p.y = -lane_width / 2.0 - 1;
                    p.z = 0;

                    crossing_prediction.points.push_back(p);

                    p.x = position.x;
                    p.y = lane_width / 2.0 + 1;
                    p.z = 0;

                    crossing_prediction.points.push_back(p);

                    markers.markers.push_back(crossing_prediction);
                }

                {
                    std::string name = "pedestrian_continue_prediction_" + std::to_string(i);

                    visualization_msgs::Marker continue_prediction;
                    continue_prediction.header.stamp = ros::Time::now();
                    continue_prediction.header.frame_id = "map";
                    continue_prediction.id = std::hash<std::string>()(name);
                    continue_prediction.ns = name;
                    continue_prediction.type = visualization_msgs::Marker::LINE_STRIP;
                    continue_prediction.action = visualization_msgs::Marker::ADD;
                    continue_prediction.pose.position.x = 0;
                    continue_prediction.pose.position.y = 0;
                    continue_prediction.scale.x = 0.1; // width

                    continue_prediction.color.a = 1.0 - crossing_probability;
                    continue_prediction.color.r = 0.0;
                    continue_prediction.color.g = 1.0;
                    continue_prediction.color.b = 0.0;

                    geometry_msgs::Point p;
                    p.x = start_position.x - lane_width / 2.0 - 1;
                    p.y = position.y;
                    p.z = 0;

                    continue_prediction.points.push_back(p);

                    p.x = start_position.x + lane_width / 2.0 + 1;
                    p.y = position.y;
                    p.z = 0;

                    continue_prediction.points.push_back(p);

                    markers.markers.push_back(continue_prediction);
                }
            }
        }

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

    std::vector<std::shared_ptr<Pedestrian>> pedestrians() const { return pedestrians_; }

    std::shared_ptr<Pedestrian> pedestrian(std::size_t i) const { return pedestrians_[i]; }

    void erase(std::size_t i) { pedestrians_[i] = nullptr; }

    void replace_pedestrian(std::size_t i, const std::shared_ptr<Pedestrian> & pedestrian)
    {
        pedestrians_[i] = pedestrian;
    }

    void step(const Position2D & car_position)
    {
        for(const auto & pedestrian: pedestrians_)
        {
            if(pedestrian)
            {
                //ROS_INFO_STREAM("pedestrian start position.x:" << pedestrian->get_start_position().x);

                pedestrian->step(ros::Time::now().toSec(), car_position.x);
            }
        }
    }

private:
    std::vector<std::shared_ptr<Pedestrian>> pedestrians_;
    tf::TransformListener & tf_listener_;
};

// obstacle creation
std::shared_ptr<Pedestrian> draw_new_pedestrian(double p_crossing, uint id, const Position2D & car_position, tf::TransformListener & tf_listener,
                                                ros::Publisher & pose_publisher, ros::Publisher & ctrl_publisher)
{
    // draw new OBSTACLE
    //ROS_INFO_STREAM("Draw new pedestrian..");
    std::shared_ptr<Pedestrian> pedestrian;

    const double distance = distance_ahead + rand_01() * 40.0;
    const double new_x = car_position.x + distance;
    const double new_y = rand_01() > 0.5 ? 0.5 * lane_width + 1 : - 0.5 * lane_width - 1;
    const double certainty_x = new_x - 15; // uncertainty vanishes 15 m to the pedestrian

    const double p = draw_p(p_crossing);
    //std::cout << "p:" << p << std::endl;
    const double certainty_distance = 10 + ( distance_ahead - 5 ) * rand_01();// * rand_01();

    if(draw_bool(p_crossing))
    {
        pedestrian = std::shared_ptr<Pedestrian>(
                    new CrossingPedestrian(id,
                                           {new_x, new_y},
                                           p,
                                           certainty_x,
                                           rand_01() > 0.5,
                                           tf_listener,
                                           pose_publisher,
                                           ctrl_publisher) );
    }
    else
    {
        pedestrian = std::shared_ptr<Pedestrian>(
                    new NonCrossingPedestrian(id,
                                              {new_x, new_y},
                                              p,
                                              certainty_x,
                                              rand_01() > 0.5,
                                              tf_listener,
                                              pose_publisher,
                                              ctrl_publisher) );
    }

    return pedestrian;
};

int main(int argc, char **argv)
{
    srand(0);

    int N = 1;
    double p_crossing = 0.1;

    ROS_INFO_STREAM("Launch pdestrian control..");

    ros::init(argc, argv, "stopline_control");

    ros::NodeHandle n;
    tf::TransformListener tf_listener;

    n.getParam("n_pedestrians", N);
    n.getParam("p_crossing", p_crossing);
    n.getParam("road_width", lane_width);

    ros::Publisher markers_publisher = n.advertise<visualization_msgs::MarkerArray>("/lgp_pedestrian_belief/marker_array", 1000);
    std::vector<ros::Publisher> ctrl_publishers, pose_publishers;
    std::vector<bool> pedestrians_to_create;
    for(auto i = 0; i < N; ++i)
    {
        ctrl_publishers.push_back(n.advertise<geometry_msgs::Twist>("/lgp_pedestrian_" + std::to_string(i) + "/vel_cmd", 1000));
        pose_publishers.push_back(n.advertise<geometry_msgs::Pose2D>("/lgp_pedestrian_" + std::to_string(i) + "/pose_reset", 1000));
    }

    ros::Rate loop_rate(10);

    // loop variables
    PedestrianObserver observer(N, tf_listener);

    while(ros::ok())
    {
        // observe car and reset if necessary
        try
        {
            const auto car_position = observer.get_car_position();

            // purge old
            for(auto i = 0; i < N; ++i)
            {
                auto pedestrian = observer.pedestrian(i);

                if(pedestrian && pedestrian->is_done(ros::Time::now().toSec(), car_position.x))
                {
                    observer.erase(i);
                }
            }

            // recreate new
            for(auto i = 0; i < N; ++i)
            {
                if(!observer.pedestrian(i))
                {
                    auto pedestrian = draw_new_pedestrian(p_crossing, i, car_position, tf_listener, pose_publishers[i], ctrl_publishers[i]);
                    observer.replace_pedestrian(i, pedestrian);

                    n.setParam("/n_crossings", n_crossings);
                    n.setParam("/n_non_crossings", n_non_crossings);

                    break; // hack, don't spawn right away because gazebo / ros doesn't always like it
                }
            }

            // publish observation
            observer.step(car_position);
            auto markers = observer.observe_pedestrians();

            markers_publisher.publish(markers);

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
