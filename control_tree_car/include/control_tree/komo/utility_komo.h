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
int shift_komos(const std::shared_ptr<KOMO>& komo, const OdometryState & o, uint steps);
//void shift_dual(DualState& state, int dual_dim_per_step, int index);
//void shift(arr& state, uint n);
std::vector<Pose2D> convert(const std::shared_ptr<KOMO>& komo);
void slide_trajectory(uint index, uint steps, std::vector<Pose2D>& trajectory);
void translate_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory);
void update_komo(const std::vector<Pose2D>& trajectory, const std::shared_ptr<KOMO>& komo);
void update_x(arr&x, const std::vector<std::shared_ptr<KOMO>>& komos, const std::vector<intA>& vars);
