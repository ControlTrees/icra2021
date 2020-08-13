#include <control_tree/komo/obstacle_avoidance_dec.h>
#include <control_tree/core/behavior_manager.h>
#include <control_tree/komo/velocity_axis.h>
#include <control_tree/core/utility.h>
#include <control_tree/komo/utility_komo.h>

#include <subtree_generators.h>

namespace
{
std::shared_ptr< KOMO > createKOMO(const rai::KinematicWorld & kin, uint n_phases, uint steps_per_phase)
{
  auto komo = std::shared_ptr< KOMO >();
  komo = std::make_shared<KOMO>();
  komo->sparseOptimization = true;
  komo->setModel(kin, false);
  komo->setTiming(n_phases, steps_per_phase, 1);
  komo->verbose = 0;

  return komo;
}
}

ObstacleAvoidanceDec::ObstacleAvoidanceDec(BehaviorManager& behavior_manager, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , N_(2)
    , kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , v_desired_(1.0)
    , existence_probability_(0.9)
    , tree_(1.0, 0)
    , options_(PARALLEL, true, NOOPT, false)
{
    options_.opt.verbose = 1;

    // optim structure
    update_tree(existence_probability_);

    // komo
    for(auto i = 0; i < N_; ++i)
    {
      // komo
      auto komo = createKOMO(kin_, 4, steps_);
      komos_.push_back(komo);

      // objectives
      Objectives objectives;

      auto p = (i == 0 ? existence_probability_ : 1.0 - existence_probability_);
      objectives.scales_ = p * ones(4 * steps_);

      objectives.acc_ = komo->addObjective(-123., 123., new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);
      objectives.acc_->vars = vars_branch_order_2_;
      objectives.acc_->scales = objectives.scales_;

      objectives.ax_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL), OT_sos, NoArr, 1.0, 0);
      objectives.ax_->vars = vars_branch_order_0_;
      objectives.ax_->scales = objectives.scales_;

      objectives.vel_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL), OT_sos, {v_desired_}, 1.0, 1);
      objectives.vel_->vars = vars_branch_order_1_;
      objectives.vel_->scales = objectives.scales_;

      objectives.car_kin_ = komo->addObjective(-123., 123., new CarKinematic("car_ego"), OT_eq, NoArr, 1e2, 1);
      objectives.car_kin_->vars = vars_branch_order_1_;

      if(i < N_-1)
      {
        objectives.circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", obstacle_position_, 1.0, 0.0));

        objectives.collision_avoidance_ = komo->addObjective(-123., 123., objectives.circular_obstacle_, OT_ineq, NoArr, 1e2, 0);
        objectives.collision_avoidance_->vars = vars_branch_order_0_;
      }

      objectivess_.push_back(objectives);

      komo->reset(0);

      // opt pb
      auto gp = std::make_shared<KOMO::Conv_MotionProblem_GraphProblem>(*komo);
      auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo->logFile);

      converters_.push_back(gp);
      constrained_problems_.push_back(pb);
    }

    init_optimization_variable();

    //komos.
//    komo_ = std::make_shared<KOMO>();
//    komo_->sparseOptimization = true;
//    komo_->setModel(kin_, false);
//    komo_->setTiming(tree_.n_nodes(), steps_, 1);
//    komo_->verbose = 0;

//    // set objectives
//    acc_ = komo_->addObjective(-123., 123., new TM_Transition(komo_->world), OT_sos, NoArr, 1.0, 2);
//    acc_->vars = vars_all_order_2_;
//    acc_->scales = scales_all_;

//    ax_ = komo_->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL), OT_sos, NoArr, 1.0, 0);
//    ax_->vars = vars_all_order_0_;
//    ax_->scales = scales_all_;

//    //vel_ = komo_->addObjective(0, -1, new VelocityAxis(komo_->world, "car_ego"), OT_sos, { v_desired_, 0, 0 }, 1.0, 1);
//    vel_ = komo_->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL), OT_sos, {v_desired_}, 1.0, 1);
//    vel_->vars = vars_all_order_1_;
//    vel_->scales = scales_all_;

//    car_kin_ = komo_->addObjective(-123., 123., new CarKinematic("car_ego"), OT_eq, NoArr, 1e2, 1);
//    car_kin_->vars = vars_all_order_1_;

//    collision_avoidance_ = komo_->addObjective(-123., 123., circular_obstacle_, OT_ineq, NoArr, 1e2, 0);
//    collision_avoidance_->vars = vars_branch_1_order_0_;

//    komo_->reset(0);

    // debug
//    std::cout << "///" << std::endl;
//    std::cout << vars_branch_1_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_branch_2_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_all_order_1_ << std::endl;
}


void ObstacleAvoidanceDec::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void ObstacleAvoidanceDec::obstacle_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
    //ROS_INFO( "update obstacle_belief.." );

    /// position and geometry
    obstacle_position_ = {msg->pose.position.x, msg->pose.position.y, 0};

    /// existance probability
    existence_probability_ = msg->color.a;

    // clamp
    const double min = 0.1; // hack -> to change
    auto p = std::max(existence_probability_, min);
    p = std::min(p, 1.0 - min);
    //

    update_tree(p);
}

TimeCostPair ObstacleAvoidanceDec::plan()
{
    //ROS_INFO( "ObstacleAvoidanceTree::plan.." );

    if(obstacle_position_.d0 == 0)
    {
        ROS_INFO( "ObstacleAvoidanceTree::plan.. abort planning" );

        return {0.0, 0.0};
    }

    // update task maps
    for(auto i = 0; i < N_; ++i)
    {
      auto& objectives = objectivess_[i];
      auto p = (i == 0 ? existence_probability_ : 1.0 - existence_probability_);
      objectives.scales_ = p * ones(4 * steps_);

      objectives.acc_->scales = objectives.scales_;
      objectives.vel_->scales = objectives.scales_;
      objectives.vel_->map->target = {v_desired_};

      if(i < N_ - 1)
      {
        objectives.circular_obstacle_->set_obstacle_position(ARR(obstacle_position_(0), obstacle_position_(1), obstacle_position_(2)));

//        if(existence_probability_ < 0.01)
//        {
//          komos_[i]->objectives.removeValue(objectives.collision_avoidance_, false);
//        }
//        else if(komos_[i]->objectives.findValue(objectives.collision_avoidance_) == -1)
//        {
//          komos_[i]->objectives.append(objectives.collision_avoidance_);
//        }
      }
    }

    // update start state
    const auto o = manager_.odometry();

    for(auto i = 0; i < N_; ++i)
    {
      set_komo_start(komos_[i], o, steps_);
      komos_[i]->reset();
    }

    // run
    auto start = std::chrono::high_resolution_clock::now();

    DecOptConstrained<ConstrainedProblem> opt(x_, constrained_problems_, xmasks_, options_);
    opt.run();

//    //komo_->getReport(true);
//    //komo_->plotTrajectory();
    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[tree] execution time (ms): %f", execution_time_us / 1000 );

//    // evaluate costs
//    auto Gs = get_traj_start(komo_->configurations);
//    auto cost = traj_cost(Gs, {acc_, ax_/*, vel_*/});
    //
    auto cost = 0;
    return {execution_time_us / 1000000, cost};
}

std::vector<nav_msgs::Path> ObstacleAvoidanceDec::get_trajectories()
{
    std::vector<nav_msgs::Path> trajs;
    trajs.reserve(N_);

    const auto t = ros::Time::now();
    for(auto i = 0; i < N_; ++i)
    {
      nav_msgs::Path msg;
      msg.header.stamp = t;
      msg.header.frame_id = "map";

      for(const auto k: vars_branch_order_0_)
      {
        const auto& kin = komos_[i]->configurations(k+2); // order 2
        msg.poses.push_back(kin_to_pose_msg(kin));
      }

      trajs.push_back(msg);
    }

    return trajs;
}

void ObstacleAvoidanceDec::update_tree(double p)
{
    tree_.add_edge(0, 1);
    tree_.add_edge(1, 2, p); // branch 1
    tree_.add_edge(2, 3);
    tree_.add_edge(3, 4);

    tree_.add_edge(1, 5, 1.0 - p); // branch 2
    tree_.add_edge(5, 6);
    tree_.add_edge(6, 7);

    vars_branch_order_0_ = tree_.get_vars({0.0, 4.0}, 4, 0, steps_);
    vars_branch_order_1_ = tree_.get_vars({0.0, 4.0}, 4, 1, steps_);
    vars_branch_order_2_ = tree_.get_vars({0.0, 4.0}, 4, 2, steps_);
}

void ObstacleAvoidanceDec::init_optimization_variable()
{
  const auto dim = komos_.front()->world.q.d0;
  const auto n_phases = tree_.n_nodes() - 1;
  const auto x_size = dim * n_phases * steps_;

  x_ = zeros(x_size);

  mp::BranchGen gen(tree_);

  while(!gen.finished())
  {
    const auto uncompressed = gen.next(); // extract subtree (here a branch)
    const auto leaves = uncompressed.get_leaves();
    CHECK_EQ(1, leaves.size(), "a branch should have one leaf");
    const auto leaf = leaves.front();
    const auto var = tree_.get_vars({0.0, 4.0}, leaf, 0, steps_);

    arr xmask = zeros(x_size);
    for(auto j: var)
    {
      for(auto k = 0; k < dim; ++k)
      {
        xmask(dim * j + k) = 1;
      }
    }

    xmasks_.push_back(xmask);
  }
}

