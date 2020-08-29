#include "lgp_car.hpp"

#include "ros/subscribe_options.h"

namespace gazebo
{

namespace
{
double round(double v)
{
    return std::round(v * 100)/ 100;
}

void filter(math::Pose &pose)
{
    auto& rot = pose.rot;
    double roll = rot.GetRoll();
    double pitch = round(rot.GetPitch());
    double yaw = rot.GetYaw();
    rot.SetFromEuler(roll, pitch, yaw);
}
}

LGPCarPlugIn::LGPCarPlugIn()
  : ModelPlugin()
  , wheel_radius_(0.19)
  , wheel_distance_(1.8)
  , target_v_(0.0)
  , target_omega_(0.0)
{
    ROS_INFO_STREAM("-->Create LGP Car Plugin");
}

void LGPCarPlugIn::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ROS_INFO_STREAM("Load model:" << parent->GetName() << " sdf:" << sdf->GetName());

    // Store the pointer to the model
    model_ = parent;

    // Retrieve Joints
    left_wheel_joint_ = model_->GetJoint("left_wheel_hinge");

    if(!left_wheel_joint_)
    {
        ROS_WARN_STREAM("Couldn't find left wheel joint in the model description! -> should be a static obstacle");
    }

    right_wheel_joint_ = model_->GetJoint("right_wheel_hinge");

    if(!right_wheel_joint_)
    {
        ROS_WARN_STREAM("Couldn't find right wheel joint in the model description! -> should be a static obstacle");
    }

    // Init Ros
    InitRos();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&LGPCarPlugIn::OnUpdate, this));
}

// Called by the world update start event
void LGPCarPlugIn::PublishMsgs()
{
    auto pose = model_->GetWorldPose();
    filter(pose); // remove unwanted tiny pitch

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.child_frame_id = model_->GetName();
    msg.pose.pose.position.x = pose.pos.x;
    msg.pose.pose.position.y = pose.pos.y;

    msg.pose.pose.orientation.x = pose.rot.x;
    msg.pose.pose.orientation.y = pose.rot.y;
    msg.pose.pose.orientation.z = pose.rot.z;
    msg.pose.pose.orientation.w = pose.rot.w;

    msg.twist.twist.linear.x = target_v_;
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
    visualization_msgs::Marker marker = CreateMarker(msg);
    marker_pub_.publish(marker);

  // debug
//    static int n=0;
//    ++n;
//    if(!(n%100))
//      ROS_INFO_STREAM("time: " << tf.header.stamp << " node:" << model_->GetName() <<  " x:" << msg.pose.pose.position.x << " y:" << msg.pose.pose.position.y << " rpm left:" << left << " rpm right:" << right);
}

visualization_msgs::Marker LGPCarPlugIn::CreateMarker(nav_msgs::Odometry msg) const
{
    if( model_->GetName().find("obstacle") != std::string::npos )
    {
        return CreateObstacle(msg);
    }

    return CreateCar(msg);
}

visualization_msgs::Marker LGPCarPlugIn::CreateCar(nav_msgs::Odometry msg) const
{
    visualization_msgs::Marker marker;
    marker.header = msg.header;
    marker.id = std::hash<std::string>()(msg.header.frame_id);
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;//;visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = msg.pose.pose;
    marker.scale.x = 1;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://lgp_car_gazebo_plugin/meshes/car.dae";

    return marker;
}

visualization_msgs::Marker LGPCarPlugIn::CreateObstacle(nav_msgs::Odometry msg) const
{
    visualization_msgs::Marker marker;
    marker.header = msg.header;
    marker.id = std::hash<std::string>()(msg.header.frame_id);
    marker.type = visualization_msgs::Marker::CYLINDER;//;visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = msg.pose.pose;
    marker.scale.x = 1.0; // diameter
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

void LGPCarPlugIn::ApplyControl()
{
    if(fabs(target_v_) < 0.1)
    {
        model_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));

        if(left_wheel_joint_ && right_wheel_joint_)
        {
            left_wheel_joint_->SetVelocity(0, 0);
            right_wheel_joint_->SetVelocity(0, 0);
        }
        return;
    }

    double nominal = target_v_ / wheel_radius_;
    double differential = target_omega_ * wheel_distance_ / ( 2 * wheel_radius_ );

    double left = nominal - differential;
    double right = nominal + differential;

    //ROS_INFO_STREAM(" rpm left:" << left << " rpm right:" << right);
    if(left_wheel_joint_ && right_wheel_joint_)
    {
        left_wheel_joint_->SetVelocity(0, left);
        right_wheel_joint_->SetVelocity(0, right);
    }
}

void LGPCarPlugIn::OnUpdate()
{
    // Apply a small linear velocity to the model.
    //model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    //model_->SetWorldTwist(ignition::math::Vector3d(2.0, 0, 0), ignition::math::Vector3d(0, 0, 0.3));

    // Publish odometry
    PublishMsgs();

    //// Apply control
    ApplyControl();
}

void LGPCarPlugIn::CmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
    target_v_ = msg->linear.x;
    target_omega_ = msg->angular.z;

    static bool connected = false;

    if(!connected)
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&LGPCarPlugIn::OnUpdate, this));

    connected = true;
//    static int n=0;
//    ++n;
//    if(!(n%100))
//      ROS_INFO_STREAM("cmdVelCB, target_v_:" << target_v_ << " target_omega_:" << target_omega_);
}

void LGPCarPlugIn::ResetPoseCB(const geometry_msgs::Pose2DConstPtr &msg)
{
    ROS_INFO_STREAM("Reset position!");

    model_->SetWorldPose(math::Pose(msg->x, msg->y, 0, 0, 0, msg->theta));
}

void LGPCarPlugIn::InitRos()
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
                            boost::bind(&LGPCarPlugIn::CmdVelCB, this, _1),
                            ros::VoidPtr(), &ros_queue_);

    cmd_vel_sub_ = ros_node_->subscribe(so_vel);

    // Subscribe to position reset
    ros::SubscribeOptions so_pose = ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
                            "/" + model_->GetName() + "/pose_reset",
                            1,
                            boost::bind(&LGPCarPlugIn::ResetPoseCB, this, _1),
                            ros::VoidPtr(), &ros_queue_);

    reset_pos_sub_ = ros_node_->subscribe(so_pose);

    // Create odometry topic
    odometry_pub_ = ros_node_->advertise<nav_msgs::Odometry>("/" + model_->GetName() + "/odometry", 1000);

    // Create marker topic
    marker_pub_ = ros_node_->advertise<visualization_msgs::Marker>("/" + model_->GetName() + "/marker", 1000);

    // Init broadcaster
    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    // Spin up the queue helper thread.
    ros_queue_thread_ = std::thread(std::bind(&LGPCarPlugIn::QueueThread, this));

    ROS_INFO_STREAM("--<end ros init for " << model_->GetName());
}

/// \brief ROS helper function that processes messages
void LGPCarPlugIn::QueueThread()
{
  static const double timeout = 0.01;
  while (ros_node_->ok())
  {
    ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

} // namespace gazebo
