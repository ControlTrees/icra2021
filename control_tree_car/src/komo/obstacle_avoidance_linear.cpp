#include <control_tree/komo/obstacle_avoidance_linear.h>
#include <control_tree/core/behavior_manager.h>
#include <circular_obstacle.h>
#include <road_bound.h>
#include <control_tree/komo/utility_komo.h>

namespace
{
std::vector<arr> get_obstacles(const std::vector<Obstacle> & obstacles)
{
    std::vector<arr> obs;

    for(auto i = 0; i < obstacles.size(); ++i)
    {
        if(obstacles[i].p >= 0.01)
            obs.push_back(obstacles[i].position);
        else
            obs.push_back({-10.0, 0, 0});
    }

    return obs;
}
}



ObstacleAvoidanceLinear::ObstacleAvoidanceLinear(BehaviorManager& behavior_manager, int n_obstacles, double road_width, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , n_obstacles_(n_obstacles)
    , road_width_(road_width)
    , kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , v_desired_(10.0)
    , obstacles_(n_obstacles_, {arr{-10, 0, 0}, 0.0})
{
    // komo
    komo_ = std::make_shared<KOMO>();
    komo_->sparseOptimization = true;
    komo_->setModel( kin_, false );
    komo_->setTiming(4, steps_, 1);
    komo_->verbose = 0;

    // set objectives
    std::vector<arr> obs = get_obstacles(obstacles_);
    circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", obs, 1.0, komo_->world));

    acc_ = komo_->addObjective(0, -1, new TM_Transition(komo_->world), OT_sos, NoArr, 1.0, 2);
    //ax_ = komo_->addObjective(0, -1, new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL, komo_->world), OT_sos, NoArr, 1.0, 0);
    ax_ = komo_->addObjective(0., -1., new RoadBound("car_ego", 3.5 / 2.0, vehicle_width, komo_->world), OT_sos, NoArr, 1.0, 0);
    vel_ = komo_->addObjective(0, -1, new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL, komo_->world), OT_sos, { v_desired_ }, 1e-1, 1);
    car_kin_ = komo_->addObjective(0, -1, new CarKinematic("car_ego", komo_->world), OT_eq, NoArr, 1e1, 1);
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

    CHECK_EQ(msg->markers.size(), obstacles_.size(), "number of obstacles are not consistent");

    for(auto i = 0; i < msg->markers.size(); ++i)
    {
      /// position and geometry
      obstacles_[i].position = {msg->markers[i].pose.position.x, msg->markers[i].pose.position.y, 0};

      /// existence probability
      obstacles_[i].p = msg->markers[i].color.a;
    }
}

TimeCostPair ObstacleAvoidanceLinear::plan()
{
    //ROS_INFO( "ObstacleAvoidanceLinear::plan.." );

    // update task maps
    //vel_->map->target = {v_desired_, 0.0, 0.0};
    vel_->map->target = {v_desired_};
    std::vector<arr> obs = get_obstacles(obstacles_);
    circular_obstacle_->set_obstacle_positions(obs);

    // init
    auto o = manager_.odometry();
    shift_komos({komo_}, o, steps_);
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
    auto cost = traj_cost(Gs, {acc_, ax_, /*vel_*/});
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

    msg_1.poses.push_back(kin_to_pose_msg(komo_->configurations(0)));
    msg_1.poses.push_back(kin_to_pose_msg(komo_->configurations(1)));

    for(auto k = 0; k < komo_->configurations.d0 - 2; ++k)
    {
        const auto& kin = komo_->configurations(k+2); // order 2
        msg_1.poses.push_back(kin_to_pose_msg(kin));
    }

    return {msg_1};
}
