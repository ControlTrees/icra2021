#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include <control_tree/core/utility.h>

using MsgsType = std::tuple<geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>;

namespace
{
geometry_msgs::PoseStamped to_pose_msg(const Pose2D& pose)
{
    tf2::Quaternion q; q.setRPY(0, 0, pose.yaw);
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    return pose_msg;
}
}

class TrajectoryController
{
public:
    TrajectoryController(int steps_per_phase)
        : steps_per_phase_(steps_per_phase)
    {

    }

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // retrieve pose
        odometry_.x = msg->pose.pose.position.x;
        odometry_.y = msg->pose.pose.position.y;
        odometry_.yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        // retrieve speeds
        odometry_.v = msg->twist.twist.linear.x;
        odometry_.omega = msg->twist.twist.angular.z;

        odo_received_ = true;
    }

    void trajectory_callback(const nav_msgs::Path::ConstPtr& msg)
    {
        trajectory_ = std::vector<Pose2D>(msg->poses.size());

        for(auto i = 0; i < msg->poses.size(); ++i)
        {
            auto pose = msg->poses[i];
            trajectory_[i] = Pose2D{pose.pose.position.x, pose.pose.position.y, get_yaw_from_quaternion(pose.pose.orientation)};
        }
    }

    MsgsType create_control() const
    {   
        const auto nominal_v = 5; // scale w according to the ratio v / nominal_v

        //// return early if not enough info
        if( trajectory_.size() == 0 || ! odo_received_ )
        {
            return std::tuple<geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>();
        }

        // get 0-0
        Pose2D current = {odometry_.x, odometry_.y, odometry_.yaw};

        // project on trajectory
        int index = -1;
        double mu = -1;

        const auto projected = project_on_trajectory(current, trajectory_, index, mu);

        if(index == -1)
        {
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;

            return std::tuple<geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
                    (twist_msg, geometry_msgs::PoseStamped(), to_pose_msg(trajectory_[2]));
        }

        /// v
        int k = index + 1;
        int l = k + 1;
        if(l >= trajectory_.size())
        {
            l = trajectory_.size() - 1;
            k = l - 1;
        }
        double v = sqrt(pow(trajectory_[l].x - trajectory_[k].x, 2) + pow(trajectory_[l].y - trajectory_[k].y, 2)) * steps_per_phase_;

        /// w
        double w_cmd = 0;

        // distance between current pose and its projection on the trajectory
        Eigen::Vector2f u(current.x - projected.x, current.y - projected.y);
        Eigen::Vector2f n(-sin(projected.yaw), cos(projected.yaw));
        const auto d = n.dot(u);

        // angle
        double d_yaw = projected.yaw - current.yaw;

        w_cmd = 3 * (-0.1 * d + d_yaw);
        w_cmd = w_cmd * (std::max(v, 2.0) / nominal_v);

        // scaling
        const double max_w = 1.0;
        if(std::fabs(w_cmd) > max_w)
        {
            //std::cout << "limit omega!" << w_cmd << std::endl;
            //v *= max_w / std::fabs(w_cmd);
            w_cmd = w_cmd > 0 ? max_w : -max_w;
        }

        ///
        auto target_pose_msg = to_pose_msg(projected);

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = w_cmd;

        return std::make_tuple(twist_msg, target_pose_msg, to_pose_msg(trajectory_[2]));
    }

    MsgsType create_control_nose_based() const
    {
        const auto d_nose = 10.0;
        const auto nominal_v = 5; // scale w according to the ratio v / nominal_v

        //// return early if not enough info
        if( trajectory_.size() == 0 || ! odo_received_ )
        {
            return std::tuple<geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>();
        }

        // get 0-0
        Pose2D current = {odometry_.x, odometry_.y, odometry_.yaw};

        // get nose
        Pose2D current_nose = {odometry_.x + cos(odometry_.yaw) * d_nose, odometry_.y + sin(odometry_.yaw) * d_nose, odometry_.yaw};

        // project nose on trajectory
        int index = -1;
        int index_nose = -1;
        double mu = -1;

        const auto projected = project_on_trajectory(current, trajectory_, index, mu);
        const auto projected_nose = project_on_trajectory(current_nose, trajectory_, index_nose, mu);

        //ROS_ERROR_STREAM("index:" << index << " size:" << trajectory_.size());
        if(index == -1)
        {
            //ROS_ERROR_STREAM("ego pose doesn't project correctly to planned trajectory");

            //ROS_ERROR_STREAM("current pose:" << current.x << " " << current.y);

//            for(auto p : trajectory_)
//            {
//                ROS_ERROR_STREAM(p.x << " " << p.y);
//            }

            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;

            return std::tuple<geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
                    (twist_msg, geometry_msgs::PoseStamped(), to_pose_msg(trajectory_[2]));
        }

        /// v
        //double v_index = sqrt(pow(trajectory_[index+1].x - trajectory_[index].x, 2) + pow(trajectory_[index+1].y - trajectory_[index].y, 2)) * steps_per_phase_;
        //double v_index_1 = sqrt(pow(trajectory_[index+2].x - trajectory_[index+1].x, 2) + pow(trajectory_[index+2].y - trajectory_[index+1].y, 2)) * steps_per_phase_;
        //double v = v_index * (1 - mu) + mu * v_index_1;
        int k = index + 1;
        int l = k + 1;
        if(l >= trajectory_.size())
        {
            l = trajectory_.size() - 1;
            k = l - 1;
        }
        double v = sqrt(pow(trajectory_[l].x - trajectory_[k].x, 2) + pow(trajectory_[l].y - trajectory_[k].y, 2)) * steps_per_phase_;
        //double v = sqrt(pow(trajectory_[index+1].x - trajectory_[index].x, 2) + pow(trajectory_[index+1].y - trajectory_[index].y, 2)) * steps_per_phase_;

        /// w
        double w_cmd = 0;
        if(index_nose != -1)
        {
            // distance between corrected nose and current nose
            Eigen::Vector2f u(current_nose.x - projected_nose.x, current_nose.y - projected_nose.y);
            Eigen::Vector2f n(-sin(projected_nose.yaw), cos(projected_nose.yaw));
            auto d = n.dot(u);

            w_cmd = -0.25 * d;
        }

        w_cmd = w_cmd * (v / nominal_v);

        const double max_w = 1.0;
        if(std::fabs(w_cmd) > max_w)
        {
            //std::cout << "limit omega!" << w_cmd << std::endl;
            //v *= max_w / std::fabs(w_cmd);
            w_cmd = w_cmd > 0 ? max_w : -max_w;
        }

        ///
        auto target_pose_msg = to_pose_msg(projected_nose);

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = w_cmd;

        return std::make_tuple(twist_msg, target_pose_msg, to_pose_msg(trajectory_[2]));
    }

private:
    // state
    bool odo_received_;
    OdometryState odometry_;
    std::vector<Pose2D> trajectory_;

    // params
    const int steps_per_phase_;
};

struct LowPassFilter
{
    geometry_msgs::Twist low_pass_filter_v(const geometry_msgs::Twist & twist, double dt)
    {
        geometry_msgs::Twist filtered = twist;

//        const double dv = twist.linear.x - last_v_;
//        const auto a = dv / dt;

//        // bound acc
//        v_filtered_ = twist.linear.x;

//        if(a > 0 && a > 8.0)
//        {
//            v_filtered_ = last_v_ + 8.0 * dt;
//        }

//        if(a < 0 && a < -8.0)
//        {
//            v_filtered_ = last_v_ - 8.0 * dt;
//        }

//        std::cout << "v filtered:" << v_filtered_ << std::endl;

        v_filtered_ = 0.7 * v_filtered_ + 0.3 * twist.linear.x;

        filtered.linear.x = v_filtered_;

        return filtered;
    }

    double v_filtered_ = 0; // low pass filtering
    double last_v_ = 0;
};



int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Launch trajectory controller..");

    int steps_per_phase = 1;
    int trajectory_index = 0;
    bool low_pass_filter = false;
    bool nose_tracking = false;

    LowPassFilter v_filter;

    // ros init
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle n;
    n.getParam("/traj_planner/steps_per_phase", steps_per_phase);
    n.getParam("/traj_controller/trajectory_index", trajectory_index);
    n.getParam("/traj_controller/low_pass_filter", low_pass_filter);
    n.getParam("/traj_controller/nose_tracking", nose_tracking);

    ros::Publisher ctrl_publisher = n.advertise<geometry_msgs::Twist>("/lgp_car/vel_cmd", 100);
    ros::Publisher start_planning_publisher = n.advertise<geometry_msgs::PoseStamped>("/lgp_car/start_planning_pose", 100);
    ros::Publisher target_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/lgp_car/target_pose", 100);

    TrajectoryController controller(steps_per_phase);

    boost::function<void(const nav_msgs::Path::ConstPtr& msg)> trajectory_callback =
            boost::bind(&TrajectoryController::trajectory_callback, &controller, _1);

    auto trajectory_subscriber = n.subscribe("/traj_planner/trajectory_" + std::to_string(trajectory_index), 100, trajectory_callback);

    boost::function<void(const nav_msgs::Odometry::ConstPtr& msg)> odometry_callback =
            boost::bind(&TrajectoryController::odometry_callback, &controller, _1);
    auto odo_subscriber = n.subscribe("/lgp_car/odometry", 1000, odometry_callback);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {  

      MsgsType msgs;
      if(nose_tracking)
          msgs = controller.create_control_nose_based();
      else
          msgs = controller.create_control();

      auto ctrl = std::get<0>(msgs);
      const auto & target = std::get<1>(msgs); // odo proj onto traj
      const auto & start = std::get<2>(msgs); // planning start

      if(low_pass_filter)
          ctrl = v_filter.low_pass_filter_v(ctrl, 1.0 / 30);

      ctrl_publisher.publish(ctrl);
      target_pose_publisher.publish(target);
      start_planning_publisher.publish(start);

      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}
