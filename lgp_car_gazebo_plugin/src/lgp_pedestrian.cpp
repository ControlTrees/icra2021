#include "lgp_pedestrian.hpp"

#include "ros/subscribe_options.h"

namespace gazebo
{
LGPPedestrianPlugIn::LGPPedestrianPlugIn()
  : ModelPlugin()
  , target_vx_(0.0)
  , target_vy_(0.0)
  , target_omega_(0.0)
{
    ROS_INFO_STREAM("-->Create LGP Pedestrian Plugin");
}

void LGPPedestrianPlugIn::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ROS_INFO_STREAM("Load model:" << parent->GetName() << " sdf:" << sdf->GetName());

    // Store the pointer to the model
    model_ = parent;

    // Init Ros
    initRos();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&LGPPedestrianPlugIn::OnUpdate, this));
}

// Called by the world update start event
void LGPPedestrianPlugIn::publishMsgs()
{
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.child_frame_id = model_->GetName();
    msg.pose.pose.position.x = model_->GetWorldPose().pos.x;
    msg.pose.pose.position.y = model_->GetWorldPose().pos.y;

    msg.pose.pose.orientation.x = model_->GetWorldPose().rot.x;
    msg.pose.pose.orientation.y = model_->GetWorldPose().rot.y;
    msg.pose.pose.orientation.z = model_->GetWorldPose().rot.z;
    msg.pose.pose.orientation.w = model_->GetWorldPose().rot.w;

    msg.twist.twist.linear.x = target_vx_;
    msg.twist.twist.linear.y = target_vy_;
    msg.twist.twist.angular.z = target_omega_;
    odometry_pub_.publish(msg);

    // Tf
    geometry_msgs::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);

    // axis aligned
    tf2::Quaternion q;
    q.setRPY( 0, 0, 0 );
    geometry_msgs::TransformStamped tf_axis_aligned;
    tf_axis_aligned.header = msg.header;
    tf_axis_aligned.child_frame_id += model_->GetName() + "_axis_aligned";
    tf_axis_aligned.transform.translation.x = msg.pose.pose.position.x;
    tf_axis_aligned.transform.translation.y = msg.pose.pose.position.y;
    tf_axis_aligned.transform.translation.z = msg.pose.pose.position.z;
    tf_axis_aligned.transform.rotation.x = q.x();
    tf_axis_aligned.transform.rotation.y = q.y();
    tf_axis_aligned.transform.rotation.z = q.z();
    tf_axis_aligned.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_axis_aligned);

    // Publish marker array
    visualization_msgs::Marker marker = create_marker(msg);
    marker_pub_.publish(marker);

    // debug
    //static int n=0;
    //++n;
    //if(!(n%100))
    //ROS_INFO_STREAM("time: " << tf.header.stamp << " node:" << model_->GetName() <<  " x:" << msg.pose.pose.position.x << " y:" << msg.pose.pose.position.y);
}

visualization_msgs::Marker LGPPedestrianPlugIn::create_marker(nav_msgs::Odometry msg) const
{
    return create_pedestrian(msg);
}

visualization_msgs::Marker LGPPedestrianPlugIn::create_pedestrian(nav_msgs::Odometry msg) const
{
    visualization_msgs::Marker marker;
    marker.header = msg.header;
    marker.id = std::hash<std::string>()(msg.header.frame_id);
    marker.type = visualization_msgs::Marker::CYLINDER;//;visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = msg.pose.pose;
    marker.pose.position.z = 0.9;
    marker.scale.x = 0.5; // diameter
    marker.scale.y = 0.5;
    marker.scale.z = 1.8;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

void LGPPedestrianPlugIn::applyControl()
{
    model_->SetLinearVel(math::Vector3(target_vx_, target_vy_, 0)); //, math::Vector3(0, 0, 0));
}

void LGPPedestrianPlugIn::OnUpdate()
{
    // Apply a small linear velocity to the model.
    //model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    //model_->SetWorldTwist(ignition::math::Vector3d(2.0, 0, 0), ignition::math::Vector3d(0, 0, 0.3));

    // Publish odometry
    publishMsgs();

    //// Apply control
    applyControl();
}

void LGPPedestrianPlugIn::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
    //ROS_INFO_STREAM("Target velocities:"<< target_vx_ << " " << target_vy_);

    target_vx_ = msg->linear.x;
    target_vy_ = msg->linear.y;
    target_omega_ = msg->angular.z;

    static bool connected = false;

    if(!connected)
    {
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&LGPPedestrianPlugIn::OnUpdate, this));
    }

    connected = true;
}

void LGPPedestrianPlugIn::resetPoseCB(const geometry_msgs::Pose2DConstPtr &msg)
{
    ROS_INFO_STREAM("Reset position! " << model_->GetName() << " " << msg->x);

    model_->SetWorldPose(math::Pose(msg->x, msg->y, 0, 0, 0, msg->theta));
}

void LGPPedestrianPlugIn::initRos()
{
    // Declare ros node
    if(!ros::isInitialized())
    {
      ROS_INFO_STREAM("call ros init..");
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_ros_facade", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    ros_node_.reset(new ros::NodeHandle(model_->GetName()));

    // Subscribe to velocity command
    ros::SubscribeOptions so_vel = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                            "/" + model_->GetName() + "/vel_cmd",
                            1,
                            boost::bind(&LGPPedestrianPlugIn::cmdVelCB, this, _1),
                            ros::VoidPtr(), &ros_queue_);

    cmd_vel_sub_ = ros_node_->subscribe(so_vel);

    // Subscribe to position reset
    ros::SubscribeOptions so_pose = ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
                            "/" + model_->GetName() + "/pose_reset",
                            1,
                            boost::bind(&LGPPedestrianPlugIn::resetPoseCB, this, _1),
                            ros::VoidPtr(), &ros_queue_);

    reset_pos_sub_ = ros_node_->subscribe(so_pose);

    // Create odometry topic
    odometry_pub_ = ros_node_->advertise<nav_msgs::Odometry>("/" + model_->GetName() + "/odometry", 1000);

    // Create marker topic
    marker_pub_ = ros_node_->advertise<visualization_msgs::Marker>("/" + model_->GetName() + "/marker", 1000);

    // Init broadcaster
    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    // Spin up the queue helper thread.
    ros_queue_thread_ = std::thread(std::bind(&LGPPedestrianPlugIn::queueThread, this));

    ROS_INFO_STREAM("--<end ros init for " << model_->GetName());
}

/// \brief ROS helper function that processes messages
void LGPPedestrianPlugIn::queueThread()
{
  static const double timeout = 0.01;
  while (ros_node_->ok())
  {
    ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

} // namespace gazebo
