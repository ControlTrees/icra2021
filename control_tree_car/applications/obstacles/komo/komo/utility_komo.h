#pragma once

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>

#include <Kin/kin.h>

#include <common/utility.h>
#include <unordered_map>

class Objective;
class KOMO;

struct Costs
{
    double total;
    std::unordered_map<std::string, double> costs;
};

geometry_msgs::PoseStamped kin_to_pose_msg(const rai::KinematicWorld * kin);
geometry_msgs::PoseStamped kin_1d_to_pose_msg(const rai::KinematicWorld * kin);
Costs traj_cost(const WorldL & Gs, const std::list<Objective *> & objectives);
WorldL get_traj_start(const WorldL & configurations, int start = 0, int end = 2);
void unify_prefix(std::vector<std::shared_ptr<KOMO>>& komos);
void shift_komos(std::vector<std::shared_ptr<KOMO>>& komos, const OdometryState & o, uint steps);
inline int shift_komos(const std::shared_ptr<KOMO>& komo, const OdometryState & o, uint steps) {NIY;return 0;}
//void shift_dual(DualState& state, int dual_dim_per_step, int index);
//void shift(arr& state, uint n);
std::vector<Pose2D> convert(const std::shared_ptr<KOMO>& komo);
void slide_trajectory(uint index, uint steps, std::vector<Pose2D>& trajectory);
void slide_trajectory(uint index, double mu, uint steps, std::vector<Pose2D>& trajectory);
Pose2D interpolate(const Pose2D& a, const Pose2D& b, double mu);
void translate_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory);
void first_guess_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory);
void first_guess_trajectory(const OdometryState & o, double v_min, uint steps, std::vector<Pose2D>& trajectory);
void update_komo(const std::vector<Pose2D>& trajectory, const std::shared_ptr<KOMO>& komo);
void update_x(arr&x, const std::vector<std::shared_ptr<KOMO>>& komos, const std::vector<intA>& vars);
