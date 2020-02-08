#pragma once

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>

#include <Kin/kin.h>

#include <control_tree/core/utility.h>

class Objective;
class KOMO;

// komo
geometry_msgs::PoseStamped kin_to_pose_msg(const rai::KinematicWorld * kin);
geometry_msgs::PoseStamped kin_1d_to_pose_msg(const rai::KinematicWorld * kin);
double traj_cost(const WorldL & Gs, const std::list<Objective *> & objectives);
WorldL get_traj_start(const WorldL & configurations, int start = 1, int end = 3);
void set_komo_start(std::shared_ptr<KOMO> komo, const OdometryState & o, uint steps);
