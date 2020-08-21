#include <control_tree/komo/obstacle_avoidance_dec.h>
#include <control_tree/core/behavior_manager.h>
#include <control_tree/core/utility.h>
#include <control_tree/komo/velocity_axis.h>

#include <velocity.h>
#include <axis_bound.h>
#include <road_bound.h>
#include <car_kinematic.h>

#include <subtree_generators.h>

// TODO: correct prefix when moving traj
// think about prob balacing (finally bettwer in dec solver?)
// probability

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

std::vector<arr> get_relevant_obstacles(const std::vector<Obstacle> & obstacles, const std::vector<bool>& activities)
{
  std::vector<arr> obs;
  for(auto j = 0; j < activities.size(); ++j)
  {
    if(activities[j])
    {
      if(obstacles[j].p >= 0.01)
        obs.push_back(obstacles[j].position);
      else
        obs.push_back({-10.0, 0, 0});
    }
  }
  return obs;
}

}

ObstacleAvoidanceDec::ObstacleAvoidanceDec(BehaviorManager& behavior_manager, int n_obstacles, double road_width, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , n_obstacles_(n_obstacles)
    , n_branches_(n_branches(n_obstacles))
    , road_width_(road_width)
    , kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , v_desired_(10.0)
    , obstacles_(n_obstacles_, {arr{-10, 0, 0}, 0.0})
    , komo_tree_(1.0, 0)
    , options_(PARALLEL, true, NOOPT, false)
{
    options_.opt.verbose = 0;
    //options_.opt.aulaMuInc = 1;

    // optim structure
    init_tree();

    // komo
    std::vector<std::vector<bool>> activities;
    fuse_probabilities(obstacles_, activities); // branch probabilities

    for(auto i = 0; i < n_branches_; ++i)
    {
      // komo
      auto komo = createKOMO(kin_, 4, steps_);
      komos_.push_back(komo);

      // objectives
      Objectives objectives;

      objectives.acc_ = komo->addObjective(-123., 123., new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);
      objectives.acc_->vars = vars_branch_order_2_;

      //objectives.ax_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL), OT_sos, NoArr, 1.0, 0);
      objectives.ax_ = komo->addObjective(-123., 123., new RoadBound("car_ego", road_width_ / 2.0, vehicle_width, komo->world), OT_sos, NoArr, 1.0, 0);
      objectives.ax_->vars = vars_branch_order_0_;

      objectives.vel_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL, komo->world), OT_sos, {v_desired_}, 1e-1, 1);
      objectives.vel_->vars = vars_branch_order_1_;

      objectives.car_kin_ = komo->addObjective(-123., 123., new CarKinematic("car_ego", komo->world), OT_eq, NoArr, 1e1, 1);
      objectives.car_kin_->vars = vars_branch_order_1_;

      std::vector<arr> obs = get_relevant_obstacles(obstacles_, activities[i]);

      if(!obs.empty())
      {
        objectives.circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", obs, 1.0, komo->world));

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
}


void ObstacleAvoidanceDec::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void ObstacleAvoidanceDec::obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    CHECK_EQ(msg->markers.size(), obstacles_.size(), "number of obstacles are not consistent");

    for(auto i = 0; i < msg->markers.size(); ++i)
    {
      /// position and geometry
      obstacles_[i].position = {msg->markers[i].pose.position.x, msg->markers[i].pose.position.y, 0};

      /// existence probability
      obstacles_[i].p = msg->markers[i].color.a;
    }
}

TimeCostPair ObstacleAvoidanceDec::plan()
{
    update_groundings();

    // update the komos based on new pose (important for efficiency)
    const auto o = manager_.odometry();

//    for(auto i = 0; i < n_branches_; ++i)
//    {
//      // update komos
//      shift_komos(komos_[i], o, steps_);

//      // Note: updating the dual seems very difficult and doesn't lead to good results!
//      // update dual
//      //const auto dual_dim_per_step = converters_[i]->dimPhi / steps_ / 4; // 4 phases
//      //shift_dual(dual_state_, dual_dim_per_step, index-2);

//      komos_[i]->reset();
//    }

    shift_komos(komos_, o, steps_);
    for(auto i = 0; i < n_branches_; ++i)
    {
        komos_[i]->reset();
    }
    //unify_prefix(komos_);

    // update the optim variable (since komos have been changed)
    update_x(x_, komos_, vars_);

    // run
    auto start = std::chrono::high_resolution_clock::now();

    //options_.checkGradients = true;

    DecOptConstrained<ConstrainedProblem> opt(x_, constrained_problems_, xmasks_, options_);

    opt.run();

    //komo_->getReport(true);
    //komo_->plotTrajectory();
    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[tree] execution time (ms): %f", execution_time_us / 1000 );

    // evaluate costs
    auto Gs = get_traj_start(komos_.front()->configurations);
    auto cost = traj_cost(Gs, {objectivess_.front().acc_, objectivess_.front().ax_, /*objectivess_.front().vel_*/});
    //

    return {execution_time_us / 1000000, cost};
}

std::vector<nav_msgs::Path> ObstacleAvoidanceDec::get_trajectories()
{
    std::vector<nav_msgs::Path> trajs;
    trajs.reserve(n_branches_);

    const auto t = ros::Time::now();
    for(auto i = 0; i < n_branches_; ++i)
    {
      nav_msgs::Path msg;
      msg.header.stamp = t;
      msg.header.frame_id = "map";

      msg.poses.push_back(kin_to_pose_msg(komos_[i]->configurations(0)));
      msg.poses.push_back(kin_to_pose_msg(komos_[i]->configurations(1)));

      for(const auto k: vars_branch_order_0_)
      {
        const auto& kin = komos_[i]->configurations(k+2); // order 2
        msg.poses.push_back(kin_to_pose_msg(kin));
      }

      trajs.push_back(msg);
    }

    return trajs;
}

void ObstacleAvoidanceDec::init_tree()
{
  convert(n_branches_, komo_tree_);

  auto leaf = komo_tree_.get_leaves().front();
  vars_branch_order_0_ = komo_tree_.get_vars({0.0, 4.0}, leaf, 0, steps_);
  vars_branch_order_1_ = komo_tree_.get_vars({0.0, 4.0}, leaf, 1, steps_);
  vars_branch_order_2_ = komo_tree_.get_vars({0.0, 4.0}, leaf, 2, steps_);
}

void ObstacleAvoidanceDec::update_groundings()
{
  std::vector<std::vector<bool>> activities;
  const auto ps = fuse_probabilities(obstacles_, activities); // branch probabilities

  for(auto i = 0; i < n_branches_; ++i)
  {
    auto& objectives = objectivess_[i];

    // set target
    objectives.vel_->map->target = {v_desired_};

    // apply scales
    double s = 1.0; //std::max(0.2, ps[i]); // min value here to keep the problem weel conditioned

    objectives.apply_scales(s * ones(4 * steps_));

    // update collision avoidance
    std::vector<arr> obs = get_relevant_obstacles(obstacles_, activities[i]);

    if(!obs.empty())
    {
      objectives.circular_obstacle_->set_obstacle_positions(obs);
    }
  }
}

void ObstacleAvoidanceDec::init_optimization_variable()
{
  const auto dim = komos_.front()->world.q.d0;
  const auto n_phases = komo_tree_.n_nodes() - 1;
  const auto x_size = dim * n_phases * steps_;

  x_ = zeros(x_size);

  mp::BranchGen gen(komo_tree_);

  while(!gen.finished())
  {
    const auto uncompressed = gen.next(); // extract subtree (here a branch)
    const auto leaves = uncompressed.get_leaves();
    CHECK_EQ(1, leaves.size(), "a branch should have one leaf");
    const auto leaf = leaves.front();
    const auto var = komo_tree_.get_vars({0.0, 4.0}, leaf, 0, steps_);

    arr xmask = zeros(x_size);
    for(auto j: var)
    {
      for(auto k = 0; k < dim; ++k)
      {
        xmask(dim * j + k) = 1;
      }
    }

    xmasks_.push_back(xmask);
    vars_.push_back(var);
  }
}

void ObstacleAvoidanceDec::Objectives::apply_scales(const arr& scales)
{
  const double surscale = 1.0; //1.5;

  //std::cout << "surscale * scales:" << surscale * scales << std::endl;

  ax_->scales = surscale * scales;
  vel_->scales = surscale * scales;
  acc_->scales = surscale * scales;
}

//-----------free functions----------------------

std::vector<double> fuse_probabilities(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{
  const uint n = pow(2.0, obstacles.size());

  //for(const auto& o: obstacles)
  //  std::cout << "o:" << o.p << std::endl;

  std::vector<double> probabilities(n, 0.0);
  activities = std::vector<std::vector<bool>>(n);

  // compute activities
  for(auto i = 0; i < obstacles.size(); ++i)
  {
    bool active = true;
    uint rythm = pow(2.0, double(i));
    for(auto j = 0; j < n; ++j)
    {
      if(j > 0 && j % rythm == 0)
      {
        active = !active;
      }
      activities[j].push_back(active);
    }
  }

  // fuse
  for(auto j = 0; j < n; ++j)
  {
    auto p = 1.0;
    for(auto i = 0; i < obstacles.size(); ++i)
    {
      if(activities[j][i])
        p *= obstacles[i].p;
      else
        p *= (1.0 - obstacles[i].p);
    }

    probabilities[j] = p;
  }

  //for(const auto& p: probabilities)
  //  std::cout << "p:" << p << std::endl;

  return probabilities;
}

void convert(uint n_branches, mp::TreeBuilder& tb)
{
  uint j = 0;

  tb.add_edge(0, 1);
  tb.add_edge(1, 2);
  tb.add_edge(2, 3);
  tb.add_edge(3, 4);

  j = 4;
  for(auto i = 1; i < n_branches; ++i)
  {
    ++j;
    tb.add_edge(1, j);
    ++j;
    tb.add_edge(j-1, j);
    ++j;
    tb.add_edge(j-1, j);
  }
}
