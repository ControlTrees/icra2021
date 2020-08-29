#include <gazebo/gazebo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>


namespace gazebo
{
class LGPCarPlugIn : public ModelPlugin
{
public:
    LGPCarPlugIn();

    // Called when starting
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Called by the world update start event
    void OnUpdate();

    // Callback for incoming velocity commands
    void CmdVelCB(const geometry_msgs::TwistConstPtr &msg);

    // Callback for pose reset
    void ResetPoseCB(const geometry_msgs::Pose2DConstPtr &msg);

private:
    void PublishMsgs();
    visualization_msgs::Marker CreateMarker(nav_msgs::Odometry msg) const;
    visualization_msgs::Marker CreateCar(nav_msgs::Odometry msg) const;
    visualization_msgs::Marker CreateObstacle(nav_msgs::Odometry msg) const;

    void ApplyControl();

    void InitRos();
    void QueueThread();

private:
    // Pointer to the model
    physics::ModelPtr model_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection_;

    // Model parameters
    double wheel_radius_;
    double wheel_distance_;

    // Model joints
    physics::JointPtr left_wheel_joint_;
    physics::JointPtr right_wheel_joint_;

    // Targets
    double target_v_;
    double target_omega_;

    // ROS
    // node use for ROS transport
    std::unique_ptr<ros::NodeHandle> ros_node_;

    // ROS vel subscriber
    ros::Subscriber cmd_vel_sub_;

    // ROS reset position subscriber
    ros::Subscriber reset_pos_sub_;

    // ROS odo publisher
    ros::Publisher odometry_pub_;

    // ROS marker array publisher
    ros::Publisher marker_pub_;

    // Tf boradcaster
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

    // ROS callbackqueue that helps process messages
    ros::CallbackQueue ros_queue_;

    // thread the keeps running the rosQueue
    std::thread ros_queue_thread_;
};

GZ_REGISTER_MODEL_PLUGIN(LGPCarPlugIn)
}

