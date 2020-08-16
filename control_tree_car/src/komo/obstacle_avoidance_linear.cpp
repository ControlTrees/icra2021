#include <control_tree/komo/obstacle_avoidance_linear.h>
#include <control_tree/core/behavior_manager.h>
#include <circular_obstacle.h>
#include <control_tree/komo/utility_komo.h>


ObstacleAvoidanceLinear::ObstacleAvoidanceLinear(BehaviorManager& behavior_manager, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , v_desired_(1.0)
    , existence_probability_(0.9)
{
    //
    circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", {obstacle_position_}, 1.0, 0.0));

    // komo
    komo_ = std::make_shared<KOMO>();
    komo_->sparseOptimization = true;
    komo_->setModel( kin_, false );
    komo_->setTiming(4, steps_, 1);
    komo_->verbose = 0;

    // set objectives
    acc_ = komo_->addObjective(0, -1, new TM_Transition(komo_->world), OT_sos, NoArr, 1.0, 2);
    ax_ = komo_->addObjective(0, -1, new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL), OT_sos, NoArr, 1.0, 0);
    //vel_ = komo_->addObjective(0, -1, new VelocityAxis(komo_->world, "car_ego"), OT_sos, { v_desired_, 0, 0 }, 1.0, 1);
    vel_ = komo_->addObjective(0, -1, new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL), OT_sos, { v_desired_ }, 1.0, 1);
    car_kin_ = komo_->addObjective(0, -1, new CarKinematic("car_ego"), OT_eq, NoArr, 1e2, 1);
    collision_avoidance_ = komo_->addObjective(0, -1, circular_obstacle_, OT_ineq, NoArr, 1e2, 0);

    komo_->reset(0);
}


void ObstacleAvoidanceLinear::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void ObstacleAvoidanceLinear::obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //ROS_INFO( "update obstacle_belief.." );

    /// position and geometry
    obstacle_position_ = {msg->markers.front().pose.position.x, msg->markers.front().pose.position.y, 0};

    /// existance probability
    existence_probability_ = msg->markers.front().color.a;
}

TimeCostPair ObstacleAvoidanceLinear::plan()
{
    //ROS_INFO( "ObstacleAvoidanceLinear::plan.." );

    if(obstacle_position_.d0 == 0)
    {
        ROS_INFO( "ObstacleAvoidanceTree::plan.. abort planning" );

        return {0.0, 0.0};
    }

    // update task maps
    //vel_->map->target = {v_desired_, 0.0, 0.0};
    vel_->map->target = {v_desired_};
    circular_obstacle_->set_obstacle_positions({ARR(obstacle_position_(0), obstacle_position_(1), obstacle_position_(2))});

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
    komo_->reset();

    // run
    auto start = std::chrono::high_resolution_clock::now();

    komo_->run();

    //komo_->getReport(true);
    //komo_->plotTrajectory();
    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[linear] execution time (ms): %f", execution_time_us / 1000 );

    // evaluate costs
    auto Gs = get_traj_start(komo_->configurations);
    auto cost = traj_cost(Gs, {acc_, ax_/*, vel_*/});
    //

    return {execution_time_us / 1000000, cost};

}

std::vector<nav_msgs::Path> ObstacleAvoidanceLinear::get_trajectories()
{
    if( ! komo_ )
    {
        return {nav_msgs::Path(), nav_msgs::Path()};
    }

    nav_msgs::Path msg_1;
    msg_1.header.stamp = ros::Time::now();
    msg_1.header.frame_id = "map";

    for(auto k = 0; k < komo_->configurations.d0 - 2; ++k)
    {
        const auto& kin = komo_->configurations(k+2); // order 2
        msg_1.poses.push_back(kin_to_pose_msg(kin));
    }

    return {msg_1, msg_1};
}
