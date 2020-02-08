#include <control_tree/komo/utility_komo.h>

#include <KOMO/komo.h>

// komo
geometry_msgs::PoseStamped kin_to_pose_msg(const rai::KinematicWorld * kin)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, kin->q(2));
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = kin->q(0);
    pose.pose.position.y = kin->q(1);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
};

geometry_msgs::PoseStamped kin_1d_to_pose_msg(const rai::KinematicWorld * kin)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = kin->q(0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
};

double traj_cost(const WorldL & Gs, const std::list<Objective *> & objectives)
{
    double cost = 0;
    for(const auto & o: objectives)
    {
        const auto & v = (*o->map)(Gs);

        for(auto i=0; i < v.y.d0; ++i)
        {
            cost += v.y(i) * v.y(i);
        }
    }
    return cost;
}

WorldL get_traj_start(const WorldL & configurations, int start, int end)
{
    if(end >= configurations.d0)
        return WorldL();

    WorldL Gs(end - start + 1);
    for(auto i = 0; i < end - start + 1; ++i)
    {
        Gs(i) = configurations(start + i);
    }
    return Gs;
}

void set_komo_start(std::shared_ptr<KOMO> komo, const OdometryState & o, uint steps)
{
    const double vx = o.v * cos(o.yaw);
    const double vy = o.v * sin(o.yaw);

    const double dx = o.x - komo->world.q(0);
    const double dy = o.y - komo->world.q(1);
    const double dyaw = o.yaw - komo->world.q(2);

    // update world
    komo->world.q(0) = o.x;
    komo->world.q(1) = o.y;
    komo->world.q(2) = o.yaw;
    komo->world.calc_Q_from_q();
    komo->world.calc_fwdPropagateFrames();

    //for(auto i=0; i < komo_->configurations.d0; ++i)
    //{
    komo->configurations(2)->q(0) = o.x;
    komo->configurations(2)->q(1) = o.y;
    komo->configurations(2)->q(2) = o.yaw;
    komo->configurations(2)->calc_Q_from_q();
    komo->configurations(2)->calc_fwdPropagateFrames();
    //}

    // shift trajectory
    for(auto i=0; i < komo->configurations.d0; ++i)
    {
        komo->configurations(i)->q(0) += dx;
        komo->configurations(i)->q(1) += dy;
        komo->configurations(i)->q(2) += dyaw;

        komo->configurations(i)->calc_Q_from_q();
        komo->configurations(i)->calc_fwdPropagateFrames();
    }

    // adjust prefix
    komo->configurations(0)->q(0) = komo->configurations(2)->q(0) - 2 * vx / steps;
    komo->configurations(0)->q(1) = komo->configurations(2)->q(1) - 2 * vy / steps;
    komo->configurations(0)->q(2) = komo->configurations(2)->q(2) - 2 * o.omega / steps;
    komo->configurations(0)->calc_Q_from_q();
    komo->configurations(0)->calc_fwdPropagateFrames();

    komo->configurations(1)->q(0) = komo->configurations(2)->q(0) - vx / steps;
    komo->configurations(1)->q(1) = komo->configurations(2)->q(1) - vy / steps;
    komo->configurations(1)->q(2) = komo->configurations(2)->q(2) - o.omega / steps;
    komo->configurations(1)->calc_Q_from_q();
    komo->configurations(1)->calc_fwdPropagateFrames();
}


