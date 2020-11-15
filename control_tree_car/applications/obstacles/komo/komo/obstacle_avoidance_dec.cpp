#include <komo/obstacle_avoidance_dec.h>
#include <common/behavior_manager.h>
#include <common/utility.h>
#include <komo/velocity_axis.h>

#include <velocity.h>
#include <axis_bound.h>
#include <road_bound.h>
#include <car_kinematic.h>

#include <subtree_generators.h>

namespace
{

std::vector<Obstacle> get_relevant_obstacles(const std::vector<Obstacle> & obstacles, const std::vector<bool>& activities)
{
  std::vector<Obstacle> obs;
  for(auto j = 0; j < activities.size(); ++j)
  {
    if(activities[j])
    {
      if(obstacles[j].p >= 0.01)
        obs.push_back(obstacles[j]);
      else
        obs.push_back(Obstacle{{-10.0, 0, 0}, 0, 0});
    }
  }
  return obs;
}

arr to_arr(const std::vector<double> a)
{
    arr b = zeros(a.size());

    for(auto i = 0; i < a.size(); ++i)
        b(i) = a[i];

    return b;
}

}

ObstacleAvoidanceDec::ObstacleAvoidanceDec(BehaviorManager& behavior_manager, int n_obstacles, bool tree, double road_width, double v_desired, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , n_obstacles_(n_obstacles)
    , n_branches_(n_branches(n_obstacles, tree))
    , tree_(tree)
    , road_width_(road_width)
    , steps_(steps_per_phase)
    , horizon_(5)
    , v_desired_(v_desired)//(50 / 3.6)
    , obstacles_(n_obstacles_, {arr{-10, 0, 0}, 0.0})
    , komo_tree_(1.0, 0)
    , options_(PARALLEL, true, NOOPT, false)
{
    options_.opt.verbose = 0;
    options_.opt.aulaMuInc = 1;
    options_.muInit = 2.0;
    options_.muInc = 2.0;

    // optim structure
    init_tree();

    // komo
    std::vector<std::vector<bool>> activities;
    fuse_probabilities(obstacles_, tree_, activities); // branch probabilities

    for(auto i = 0; i < n_branches_; ++i)
    {
      // komo
      auto komo = komo_factory_.create_komo(horizon_, steps_);
      komos_.push_back(komo);

      // objectives
      auto obstacles = get_relevant_obstacles(obstacles_, activities[i]);

      Objectives objectives = komo_factory_.ground_komo(komo, obstacles, road_width_, v_desired_);
      objectives.acc_->vars = vars_branch_order_2_;
      objectives.ax_->vars = vars_branch_order_0_;
      objectives.vel_->vars = vars_branch_order_1_;
      objectives.car_kin_->vars = vars_branch_order_1_;

      if(!obstacles.empty())
      {
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
    CHECK_EQ(msg->markers.size(), obstacles_.size() * 2, "number of obstacles are not consistent");

    for(auto i = 0; i < msg->markers.size() / 2; ++i)
    {
      const auto&m = msg->markers[2 * i + 1];

      /// position and geometry
      obstacles_[i].position = {m.pose.position.x - 0.5, m.pose.position.y, 0};
      obstacles_[i].p = m.color.a;
      //obstacles_[i].radius = m.scale.x;
      obstacles_[i].radius = m.scale.x / 2 + 0.5; //-> video only

      //std::cout << "p[" << i << "] =" << obstacles_[i].p << std::endl;
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

    // update the optim variable (since komos have been changed)
    update_x(x_, komos_, vars_);
    auto bs = to_arr(belief_state_);
    //options_.checkGradients = true;

    // run
    auto start = std::chrono::high_resolution_clock::now();

    DecOptConstrained<ConstrainedProblem, BeliefState> opt(x_, constrained_problems_, xmasks_, BeliefState(bs), options_);
    opt.run();

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[tree] execution time (ms): %f", execution_time_us / 1000 );

    // evaluate costs
    auto Gs = get_traj_start(komos_.front()->configurations);
    auto cost = traj_cost(Gs, {objectivess_.front().acc_, objectivess_.front().ax_, objectivess_.front().vel_});
    //

    return {execution_time_us / 1000000, cost.total};
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
  convert(n_branches_, horizon_, komo_tree_);

  auto leaf = komo_tree_.get_leaves().front();
  vars_branch_order_0_ = komo_tree_.get_vars({0.0, horizon_}, leaf, 0, steps_);
  vars_branch_order_1_ = komo_tree_.get_vars({0.0, horizon_}, leaf, 1, steps_);
  vars_branch_order_2_ = komo_tree_.get_vars({0.0, horizon_}, leaf, 2, steps_);
}

void ObstacleAvoidanceDec::update_groundings()
{
  std::vector<std::vector<bool>> activities;
  belief_state_ = fuse_probabilities(obstacles_, tree_, activities); // branch probabilities

  for(auto i = 0; i < n_branches_; ++i)
  {
    auto& objectives = objectivess_[i];

    // set target
    objectives.vel_->map->target = {v_desired_};

    // update collision avoidance
    auto obstacles = get_relevant_obstacles(obstacles_, activities[i]);

    if(!obstacles.empty())
    {
      objectives.circular_obstacle_->set_obstacles(obstacles);
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
    const auto var = komo_tree_.get_vars({0.0, horizon_}, leaf, 0, steps_);

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

//-----------free functions----------------------

template<bool tree>
std::vector<double> fuse(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{

}

template<>
std::vector<double> fuse<true>(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{
    const uint n = pow(2.0, obstacles.size());

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

    return probabilities;
}

template<>
std::vector<double> fuse<false>(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{
    const uint n = 1;

    std::vector<double> probabilities(n, 0.0);
    activities = std::vector<std::vector<bool>>(n);

    // compute activities
    for(auto i = 0; i < obstacles.size(); ++i)
    {
      activities[0].push_back(true);
    }

    // fuse
    probabilities[0] = 1.0;

    return probabilities;
}

std::vector<double> fuse_probabilities(const std::vector<Obstacle>& obstacles, bool tree, std::vector<std::vector<bool>> & activities)
{
    if(tree)
        return fuse<true>(obstacles, activities);
    else
        return fuse<false>(obstacles, activities);
}

void convert(uint n_branches, uint horizon, mp::TreeBuilder& tb)
{
  uint j = 1;

  for(;j <= horizon; ++j)
  {
    tb.add_edge(j-1, j);
  }

  --j;

  for(auto i = 1; i < n_branches; ++i)
  {
    ++j;
    tb.add_edge(1, j);

    for(auto k = 3; k <= horizon; ++k)
    {
        ++j;
        tb.add_edge(j-1, j);
    }
  }
}
