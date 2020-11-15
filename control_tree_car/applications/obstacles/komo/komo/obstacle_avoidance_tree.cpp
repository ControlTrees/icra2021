#include <komo/obstacle_avoidance_tree.h>

#include <common/behavior_manager.h>

#include <komo/velocity_axis.h>

#include <common/utility.h>
#include <komo/utility_komo.h>


ObstacleAvoidanceTree::ObstacleAvoidanceTree(BehaviorManager& behavior_manager, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , v_desired_(1.0)
    , existence_probability_(0.9)
    , tree_(1.0, 0)
{
    // optim structure
    update_tree(existence_probability_);

    // komo
    komo_ = std::make_shared<KOMO>();
    komo_->sparseOptimization = true;
    komo_->setModel( kin_, false );
    komo_->setTiming(tree_.n_nodes(), steps_, 1);
    komo_->verbose = 0;

    // set objectives
    circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", obstacles_, komo_->world));

    acc_ = komo_->addObjective(-123., 123., new TM_Transition(komo_->world), OT_sos, NoArr, 1.0, 2);
    acc_->vars = vars_all_order_2_;
    acc_->scales = scales_all_;

    ax_ = komo_->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL, komo_->world), OT_sos, NoArr, 1.0, 0);
    ax_->vars = vars_all_order_0_;
    ax_->scales = scales_all_;

    //vel_ = komo_->addObjective(0, -1, new VelocityAxis(komo_->world, "car_ego"), OT_sos, { v_desired_, 0, 0 }, 1.0, 1);
    vel_ = komo_->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL, komo_->world), OT_sos, {v_desired_}, 1.0, 1);
    vel_->vars = vars_all_order_1_;
    vel_->scales = scales_all_;

    car_kin_ = komo_->addObjective(-123., 123., new CarKinematic("car_ego", komo_->world), OT_eq, NoArr, 1e2, 1);
    car_kin_->vars = vars_all_order_1_;

    collision_avoidance_ = komo_->addObjective(-123., 123., circular_obstacle_, OT_ineq, NoArr, 1e2, 0);
    collision_avoidance_->vars = vars_branch_1_order_0_;

    komo_->reset(0);

    // debug
//    std::cout << "///" << std::endl;
//    std::cout << vars_branch_1_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_branch_2_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_all_order_1_ << std::endl;
}

void ObstacleAvoidanceTree::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void ObstacleAvoidanceTree::obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //ROS_INFO( "update obstacle_belief.." );

    /// obstacle
    const auto& m = msg->markers[1];
    obstacles_.front().position = {m.pose.position.x, m.pose.position.y, 0};
    existence_probability_ = m.color.a;
    obstacles_.front().radius = m.scale.x / 2;

    // clamp
    const double min = 0.1; // hack -> to change
    auto p = std::max(existence_probability_, min);
    p = std::min(p, 1.0 - min);
    //

    update_tree(p);
}

TimeCostPair ObstacleAvoidanceTree::plan()
{
    //ROS_INFO( "ObstacleAvoidanceTree::plan.." );

    if(obstacles_.front().position.d0 == 0)
    {
        ROS_INFO( "ObstacleAvoidanceTree::plan.. abort planning" );

        return {0.0, 0.0};
    }

    // update task maps
    acc_->map->scale = scales_all_;
    vel_->map->scale = scales_all_;
    vel_->map->target = {v_desired_};
    //vel_->map->target = {v_desired_, 0.0, 0.0};

    circular_obstacle_->set_obstacles(obstacles_);

    if( existence_probability_ < 0.01 )
    {
        komo_->objectives.removeValue(collision_avoidance_, false);
    }
    else if( komo_->objectives.findValue(collision_avoidance_) == -1 )
    {
        komo_->objectives.append(collision_avoidance_);
    }

    // init
    auto o = manager_.odometry();
    shift_komos(komo_, o, steps_);

    // run
    auto start = std::chrono::high_resolution_clock::now();

    komo_->reset();
    komo_->run();

    //komo_->getReport(true);
    //komo_->plotTrajectory();
    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[tree] execution time (ms): %f", execution_time_us / 1000 );

    // evaluate costs
    auto Gs = get_traj_start(komo_->configurations);
    auto cost = traj_cost(Gs, {acc_, ax_, vel_});
    //

    return {execution_time_us / 1000000, cost.total};
}

std::vector<nav_msgs::Path> ObstacleAvoidanceTree::get_trajectories()
{
    if(!komo_)
    {
        return {nav_msgs::Path(), nav_msgs::Path()};
    }

    nav_msgs::Path msg_1;
    msg_1.header.stamp = ros::Time::now();
    msg_1.header.frame_id = "map";

    for(const auto k: vars_branch_1_order_0_)
    {
        const auto& kin = komo_->configurations(k+2); // order 2
        msg_1.poses.push_back(kin_to_pose_msg(kin));
    }

    nav_msgs::Path msg_2;
    msg_2.header.stamp = ros::Time::now();
    msg_2.header.frame_id = "map";

    for(const auto k: vars_branch_2_order_0_)
    {
        const auto& kin = komo_->configurations(k+2); // order 2
        msg_2.poses.push_back(kin_to_pose_msg(kin));
    }

    return {msg_1, msg_2};
}

void ObstacleAvoidanceTree::update_tree(double p)
{
    tree_.add_edge(0, 1);
    tree_.add_edge(1, 2, p); // branch 1
    tree_.add_edge(2, 3);
    tree_.add_edge(3, 4);

    tree_.add_edge(1, 5, 1.0 - p); // branch 2
    tree_.add_edge(5, 6);
    tree_.add_edge(6, 7);

    vars_branch_1_order_0_ = tree_.get_vars({0.0, 4.0}, 4, 0, steps_);
    vars_branch_1_order_1_ = tree_.get_vars({0.0, 4.0}, 4, 1, steps_);
    vars_branch_1_order_2_ = tree_.get_vars({0.0, 4.0}, 4, 2, steps_);

    vars_branch_2_order_0_ = tree_.get_vars({0.0, 4.0}, 7, 0, steps_);
    vars_branch_2_order_1_ = tree_.get_vars({0.0, 4.0}, 7, 1, steps_);
    vars_branch_2_order_2_ = tree_.get_vars({0.0, 4.0}, 7, 2, steps_);

    vars_all_order_0_ = tree_.get_vars({0.0, 4.0}, 4, 0, steps_);
    vars_all_order_0_.append(tree_.get_vars({1.0, 4.0}, 7, 0, steps_));

    vars_all_order_1_ = tree_.get_vars({0.0, 4.0}, 4, 1, steps_);
    vars_all_order_1_.append(tree_.get_vars({1.0, 4.0}, 7, 1, steps_));

    vars_all_order_2_ = tree_.get_vars({0.0, 4.0}, 4, 2, steps_);
    vars_all_order_2_.append(tree_.get_vars({1.0, 4.0}, 7, 2, steps_));

    scales_all_ = tree_.get_scales({0.0, 4.0}, 4, steps_);
    scales_all_.append(tree_.get_scales({1.0, 4.0}, 7, steps_));

    assert(vars_all_order_1_.d0 == scales_all_.d0);
    assert(vars_all_order_2_.d0 == scales_all_.d0);
}

