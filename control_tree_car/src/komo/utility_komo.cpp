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

Costs traj_cost(const WorldL & Gs, const std::list<Objective *> & objectives)
{
    Costs costs;
    costs.total = 0;
    for(const auto & o: objectives)
    {
        const auto & s = o->map->scale.d0 ? o->map->scale(0) : 1.0;
        const auto & v = (*o->map)(Gs);
        const auto & t = o->map->target;

        double cost = 0;
        for(auto i=0; i < v.y.d0; ++i)
        {
            auto dy = t.d0 ? v.y(i) - t.scalar() : v.y(i);
            cost += s * dy * dy;
        }

        costs.total += cost;
        costs.costs["o->name"] = cost;

        //std::cout << o->name << " " << cost << " target " << t << std::endl;
    }

    //std::cout << "TOTAL:" << costs.total << std::endl;

    return costs;
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

void shift_komos(std::vector<std::shared_ptr<KOMO>>& komos, const OdometryState & o, uint steps)
{
    // traj 0 as ref
    auto ref_traj = convert(komos[0]);

    int index; double mu;
    const auto proj = project_on_trajectory({o.x, o.y, o.yaw}, ref_traj, index, mu);

    //ROS_INFO_STREAM("index:" << index);

    OdometryState start;
    if(index >= 2) // is komo order
    {
        // nominal case
        start.x = proj.x;
        start.y = proj.y;
        start.yaw = proj.yaw;
        start.v = o.v;
        start.omega = o.omega;

        for(auto&komo: komos)
        {
            auto traj = convert(komo);

            double vx_end = steps * (traj[traj.size() - 1].x - traj[traj.size() - 2].x);
            if(vx_end > 1.0)
            {
                translate_trajectory(start, steps, traj);
            }
            else
            {   // unstuck trajectory
                ROS_WARN_STREAM("Unstuck trajectory");

                first_guess_trajectory(o, steps, traj);
            }
            update_komo(traj, komo);
        }
    }
    else if(index >= 0)
    {   // move slightly backwards? because we project on prefix, strange prefer not to change the komo

        ROS_WARN_STREAM("B. move backwards? - keep old traj");
    }
    else
    {
        ROS_WARN_STREAM("C. first iteration");

        for(auto&komo: komos)
        {
            auto traj = convert(komo);

            if(komo->world.q(0) == 0 && komo->world.q(1) == 0)
            {
                first_guess_trajectory(o, steps, traj);
            }

            update_komo(traj, komo);
        }
    }

    unify_prefix(komos);
}

void unify_prefix(std::vector<std::shared_ptr<KOMO>>& komos)
{
    for(auto i = 1; i < komos.size(); ++i)
    {
        komos[i]->configurations(0)->q(0) = komos[0]->configurations(0)->q(0);
        komos[i]->configurations(0)->q(1) = komos[0]->configurations(0)->q(1);
        komos[i]->configurations(0)->q(2) = komos[0]->configurations(0)->q(2);

        komos[i]->configurations(1)->q(0) = komos[0]->configurations(1)->q(0);
        komos[i]->configurations(1)->q(1) = komos[0]->configurations(1)->q(1);
        komos[i]->configurations(1)->q(2) = komos[0]->configurations(1)->q(2);
    }
}

int shift_komos(const std::shared_ptr<KOMO> & komo, const OdometryState & o, uint steps)
{
    // returns the number of points that are skipped

    auto traj = convert(komo);

    int index; double mu;
    const auto proj = project_on_trajectory({o.x, o.y, o.yaw}, traj, index, mu);

    //std::cout << "traj size:" << traj.size() << " index:" << index << " " << proj.x << ", " << proj.y << ", " << proj.yaw << std::endl;

    OdometryState start;
    if(index >= 2) // 2 is komo order
    {
        // nominal case
        start.x = proj.x;
        start.y = proj.y;
        start.yaw = proj.yaw;
        start.v = o.v;
        start.omega = o.omega;

        // shift and interpolate end
        //slide_trajectory(index - 2, steps, traj);

        // translate only
        translate_trajectory(start, steps, traj);
    }
    else if(index >= 0)
    {   // move slightly backwards? because we project on prefix, strange prefer not to change the komo
    }
    else if(komo->world.q(0) == 0 && komo->world.q(1) == 0)
    {
        //std::cout << "first iteration" << std::endl;
        first_guess_trajectory(o, steps, traj);
    }

    update_komo(traj, komo);

    return index;
}

//void shift_dual(DualState& state, int dual_dim_per_step, int index)
//{
//    std::cout << "dual_dim_per_step:" << dual_dim_per_step << std::endl;
//    std::cout << "index:" << index << std::endl;

//    const auto n = dual_dim_per_step * index;

//    if(!state.duals.empty())
//    for(auto i = 0; i < state.duals.size(); ++i)
//    {
//        std::cout << "state.duals:" << state.duals[i].d0 << " " << n << std::endl;

//        shift(state.duals[i], n);
//    }
//}

//void shift(arr& x, uint shift)
//{
//    //std::cout << "before:" << x << std::endl;

//    for(auto i = 0; i < x.d0 - shift; ++i)
//    {
//        x(i) = x(i + shift);
//    }

//    for(auto i = x.d0 - shift; i < x.d0; ++i)
//    {
//        x(i) = 0;
//    }

//    //std::cout << "after:" << x << std::endl;
//}

std::vector<Pose2D> convert(const std::shared_ptr<KOMO>& komo)
{
    std::vector<Pose2D> traj(komo->configurations.d0);
    for(auto i=0; i < komo->configurations.d0; ++i)
    {
        traj[i].x = komo->configurations(i)->q(0);
        traj[i].y = komo->configurations(i)->q(1);
        traj[i].yaw = komo->configurations(i)->q(2);
    }
    return traj;
}

// skip first elements and linearly interpolate for the last ones
void slide_trajectory(uint shift, uint steps, std::vector<Pose2D>& trajectory)
{
    //std::cout << "shift of:" << shift << std::endl;
    const auto& last = trajectory.back();
    const auto dx = trajectory[trajectory.size() - 1].x - trajectory[trajectory.size() - 2].x;
    const auto dy = trajectory[trajectory.size() - 1].y - trajectory[trajectory.size() - 2].y;
    const double yaw = trajectory[trajectory.size() - 1].yaw;

    for(auto i = 0; i < trajectory.size() - shift; ++i)
    {
        trajectory[i] = trajectory[i+shift];
    }

    for(auto i = 1; i <= shift; ++i)
    {
        const auto I = trajectory.size() - shift + i - 1;
        trajectory[I].x = last.x + i * dx;
        trajectory[I].y = last.y + i * dy;
        trajectory[I].yaw = yaw;
    }
}

// shift all points of the delta between proj of real pose and traj[2]. This simply moves forward the trajectory
void translate_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory)
{
    const double vx = o.v * cos(o.yaw);
    const double vy = o.v * sin(o.yaw);

    const double dx = o.x - trajectory[2].x;
    const double dy = o.y - trajectory[2].y;
    //const double dyaw = o.yaw - trajectory[2].yaw;

    // shift trajectory
    for(auto i=2; i < trajectory.size(); ++i)
    {
        trajectory[i].x += dx;
        trajectory[i].y += dy;
    }

    // adjust prefix
    trajectory[0].x = trajectory[2].x - 2 * vx / steps;
    trajectory[0].y = trajectory[2].y - 2 * vy / steps;
    //komo->configurations(0)->q(2) = komo->configurations(2)->q(2) - 2 * o.omega / steps;

    trajectory[1].x = trajectory[2].x - vx / steps;
    trajectory[1].y = trajectory[2].y - vy / steps;
    //komo->configurations(1)->q(2) = komo->configurations(2)->q(2) - o.omega / steps;
}

void first_guess_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory)
{
    const double vx = o.v * cos(o.yaw);
    const double vy = o.v * sin(o.yaw);
    const double dx = vx * 1.0 / steps;
    const double dy = vy * 1.0 / steps;

    // shift trajectory
    for(auto i=0; i < trajectory.size(); ++i)
    {
        trajectory[i].x = o.x + (i-2) * dx;
        trajectory[i].y = o.y + (i-2) * dy;
        trajectory[i].yaw = o.yaw;
    }
}

void first_guess_trajectory(const OdometryState & o, double v_min, uint steps, std::vector<Pose2D>& trajectory)
{
    {
        const double vx = o.v * cos(o.yaw);
        const double vy = o.v * sin(o.yaw);
        const double dx = vx * 1.0 / steps;
        const double dy = vy * 1.0 / steps;

        // shift prefix
        for(auto i=0; i < 2; ++i)
        {
            trajectory[i].x = o.x + (i-2) * dx;
            trajectory[i].y = o.y + (i-2) * dy;
            trajectory[i].yaw = o.yaw;
        }
    }

    {
        const double vx = v_min * cos(o.yaw);
        const double vy = v_min * sin(o.yaw);
        const double dx = vx * 1.0 / steps;
        const double dy = vy * 1.0 / steps;

        // shift prefix
        for(auto i=2; i < trajectory.size(); ++i)
        {
            trajectory[i].x = o.x + (i-2) * dx;
            trajectory[i].y = o.y + (i-2) * dy;
            trajectory[i].yaw = o.yaw;
        }
    }
}

// create straight line frm point[2]
//void shift_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory)
//{
//    const double vx = o.v * cos(o.yaw);
//    const double vy = o.v * sin(o.yaw);

//    const double dx = o.x - trajectory[2].x;
//    const double dy = o.y - trajectory[2].y;
//    //const double dyaw = o.yaw - trajectory[2].yaw;

//    // shift trajectory
//    trajectory[2].x = o.x;
//    trajectory[2].y = o.y;
//    trajectory[2].yaw = o.yaw;

//    for(auto i=0; i < trajectory.size(); ++i)
//    {
//      // adjust prefix
//      trajectory[i].x = trajectory[2].x + (i-2) * vx / steps;
//      trajectory[i].y = trajectory[2].y + (i-2) * vy / steps;
//      trajectory[i].yaw = o.yaw;
//    }
//}

void update_komo(const std::vector<Pose2D>& trajectory, const std::shared_ptr<KOMO>& komo)
{
    CHECK_EQ(trajectory.size(), komo->configurations.d0, "traj and komo should have the same size");

    //std::cout << "traj start:" << trajectory[2].x << ", " << trajectory[2].y << std::endl;

    komo->world.q(0) = trajectory[2].x;
    komo->world.q(1) = trajectory[2].y;
    komo->world.q(2) = trajectory[2].yaw;
    komo->world.calc_Q_from_q();
    komo->world.calc_fwdPropagateFrames();

    for(auto i=0; i < komo->configurations.d0; ++i)
    {
        komo->configurations(i)->q(0) = trajectory[i].x;
        komo->configurations(i)->q(1) = trajectory[i].y;
        komo->configurations(i)->q(2) = trajectory[i].yaw;
        komo->configurations(i)->calc_Q_from_q();
        komo->configurations(i)->calc_fwdPropagateFrames();
    }
}

void update_x(arr&x, const std::vector<std::shared_ptr<KOMO>>& komos, const std::vector<intA>& vars)
{
    const auto dim = komos.front()->world.q.d0;
    arr contribs = zeros(x.d0);
    arr new_x = zeros(x.d0);

    for(auto i = 0; i < komos.size(); ++i)
    {
        for(auto j = 0; j < vars[i].size(); ++j)
        {
            uint J = vars[i](j, 0);
            for(auto k = 0; k < dim; ++k)
            {
              new_x(dim * J + k) += komos[i]->configurations(j+2)->q(k);
              contribs(dim * J + k) += 1;
            }
        }
    }

    // average
    for(auto j = 0; j < x.d0; ++j)
    {
        new_x(j) /= contribs(j);
    }

    x = new_x;
}
