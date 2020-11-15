#pragma once

#include <common/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>
#include <functional>

#include <Eigen/Dense>

#include <KOMO/komo.h>
#include <komo/utility_komo.h>

#include <circular_obstacle.h>

constexpr double vehicle_width = 1.8;

struct Objectives
{
  Objective * acc_{0};
  Objective * ax_{0};
  Objective * vel_{0};
  Objective * car_kin_{0};
  std::shared_ptr<Car3CirclesCircularObstacle> circular_obstacle_;
  Objective * collision_avoidance_{0};
};

class KomoFactory
{
public:
    KomoFactory();
    std::shared_ptr< KOMO > create_komo(uint n_phases, uint steps_per_phase) const;
    Objectives ground_komo(const std::shared_ptr< KOMO > & komo,
                           const std::vector<Obstacle> & obstacles,
                           double road_width,
                           double v_desired) const;

private:
    rai::KinematicWorld kin_;
};
