#include <eigen3/Eigen/Dense>
#include <memory>

#include "ros/ros.h"
#include <ros/package.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"

#include <control_tree/core/utility.h>
#include <control_tree/komo/komo_factory.h>
#include <control_tree/ros/obstacle_common.h>

#include <KOMO/komo.h>


namespace
{
class TrajEvaluator
{
public:
    TrajEvaluator(int steps_per_phase, double road_width, double v_desired)
        : steps_per_phase_(steps_per_phase)
    {
        komo_ = komo_factory_.create_komo(2, steps_per_phase);
        objectives_ = komo_factory_.ground_komo(komo_, {}, road_width, v_desired);

        komo_->reset(0);
    }

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // retrieve pose
        odometry_.x = msg->pose.pose.position.x;
        odometry_.y = msg->pose.pose.position.y;
        odometry_.yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        // retrieve speeds
        odometry_.v = msg->twist.twist.linear.x;
        odometry_.omega = msg->twist.twist.angular.z;

        odo_received_ = true;
    }

    void trajectory_callback(const nav_msgs::Path::ConstPtr& msg)
    {
        trajectory_ = std::vector<Pose2D>(msg->poses.size());

        for(auto i = 0; i < msg->poses.size(); ++i)
        {
            auto pose = msg->poses[i];
            trajectory_[i] = Pose2D{pose.pose.position.x, pose.pose.position.y, get_yaw_from_quaternion(pose.pose.orientation)};
        }
    }

    double car_x() const { return odometry_.x; }

    double evaluate()
    {   
        //// return early if not enough info
        if( trajectory_.size() == 0 || ! odo_received_ )
        {
            return 0.0;
        }

        // get 0-0
        Pose2D current = {odometry_.x, odometry_.y, odometry_.yaw};

        // project on trajectory
        int index = -1;
        double mu = -1;

        const auto projected = project_on_trajectory(current, trajectory_, index, mu);

        //std::cout << "index:" << index << std::endl;

        if(index == -1 || index == 0)
        {
            ROS_WARN_STREAM("wrong projection");
            return 0.0;
        }

        update_komo(trajectory_[index - 1], komo_->configurations(0));
        update_komo(trajectory_[index], komo_->configurations(1));
        update_komo(trajectory_[index + 1], komo_->configurations(2));

        auto Gs = get_traj_start(komo_->configurations, 0, 2);
        auto cost = traj_cost(Gs, {objectives_.acc_, objectives_.ax_, objectives_.vel_});


        static int n = 0;
        ++n;

        if(n >=100)
            cost_evaluator_.acc(cost.total);

        return cost_evaluator_.average();
    }

    void update_komo(const Pose2D & pose, rai::KinematicWorld* kin) const
    {
        kin->q(0) = pose.x;
        kin->q(1) = pose.y;
        kin->q(2) = pose.yaw;

        kin->calc_Q_from_q();
        kin->calc_fwdPropagateFrames();
    }

private:
    // state
    bool odo_received_;
    OdometryState odometry_;
    std::vector<Pose2D> trajectory_;
    KomoFactory komo_factory_;
    std::shared_ptr< KOMO > komo_;
    Objectives objectives_;
    Evaluator cost_evaluator_;

    // params
    const int steps_per_phase_;
};
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Launch evaluation node..");

    int steps_per_phase = 1;
    int trajectory_index = 0;
    double road_width = 3.5;
    double v_desired = 10;

    // ros init
    ros::init(argc, argv, "evaluation");
    ros::NodeHandle n;
    n.getParam("/traj_planner/steps_per_phase", steps_per_phase);
    n.getParam("/traj_controller/trajectory_index", trajectory_index);
    n.getParam("/road_width", road_width);
    n.getParam("/v_desired", v_desired);

    // logging
    std::ofstream ofs(filename(n));

    // evaluation
    TrajEvaluator evaluator(steps_per_phase, road_width, v_desired);

    boost::function<void(const nav_msgs::Path::ConstPtr& msg)> trajectory_callback =
            boost::bind(&TrajEvaluator::trajectory_callback, &evaluator, _1);
    auto trajectory_subscriber = n.subscribe("/traj_planner/trajectory_" + std::to_string(trajectory_index), 100, trajectory_callback);

    boost::function<void(const nav_msgs::Odometry::ConstPtr& msg)> odometry_callback =
            boost::bind(&TrajEvaluator::odometry_callback, &evaluator, _1);
    auto odo_subscriber = n.subscribe("/lgp_car/odometry", 1000, odometry_callback);

    ros::Rate loop_rate(10);

    int i = 0;
    while (ros::ok())
    {  
      auto avg = evaluator.evaluate();
      auto car_x = evaluator.car_x();

      ros::spinOnce();

      loop_rate.sleep();

      // logging
      ++i;
      if(i && i%100 == 0)
      {
         ROS_INFO_STREAM("[cost evaluation] cost:" << avg << " " << i << " evaluations :)");

         if(i%100 == 0)
            log_to_file(ofs, n, car_x, i, avg);
      }
    }

    return 0;
}
