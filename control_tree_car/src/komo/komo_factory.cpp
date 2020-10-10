#include <control_tree/komo/komo_factory.h>

#include <velocity.h>
#include <axis_bound.h>
#include <road_bound.h>
#include <car_kinematic.h>

#include <ros/package.h>

KomoFactory::KomoFactory()
    : kin_((ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
{}

std::shared_ptr< KOMO > KomoFactory::create_komo(uint n_phases, uint steps_per_phase) const
{
  auto komo = std::shared_ptr< KOMO >();
  komo = std::make_shared<KOMO>();
  komo->sparseOptimization = true;
  komo->setModel(kin_, false);
  komo->setTiming(n_phases, steps_per_phase, 1);
  komo->verbose = 0;

  return komo;
}

Objectives KomoFactory::ground_komo(const std::shared_ptr< KOMO > & komo, const std::vector<Obstacle> & obstacles, double road_width, double v_desired) const
{
    Objectives objectives;

    objectives.acc_ = komo->addObjective(-123., 123., new TM_Transition(komo->world), OT_sos, NoArr, 2.0, 2);
    //objectives.ax_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::Y, AxisBound::EQUAL), OT_sos, NoArr, 1.0, 0);
    objectives.ax_ = komo->addObjective(-123., 123., new RoadBound("car_ego", road_width / 2.0, vehicle_width, komo->world), OT_sos, NoArr, 1.0, 0);
    objectives.vel_ = komo->addObjective(-123., 123., new AxisBound("car_ego", AxisBound::X, AxisBound::EQUAL, komo->world), OT_sos, {v_desired}, 1.5 * 1e-1, 1);
    objectives.car_kin_ = komo->addObjective(-123., 123., new CarKinematic("car_ego", komo->world), OT_eq, NoArr, 1e1, 1);

    if(!obstacles.empty())
    {
      objectives.circular_obstacle_ = std::shared_ptr<Car3CirclesCircularObstacle> (new Car3CirclesCircularObstacle("car_ego", obstacles, komo->world));
      objectives.collision_avoidance_ = komo->addObjective(-123., 123., objectives.circular_obstacle_, OT_ineq, NoArr, 1e2, 0);
    }

    return objectives;
}
