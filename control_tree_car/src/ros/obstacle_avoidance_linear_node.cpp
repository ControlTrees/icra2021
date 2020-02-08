#include <ros/package.h>
#include <tf/transform_listener.h>

#include <control_tree/core/behavior_manager.h>
#include <control_tree/komo/obstacle_avoidance_linear.h>
#include <control_tree/ros/obstacle_common.h>

#include <visualization_msgs/Marker.h>


int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Launch lgp car control, obstacle avoidance..");

    double p_obstacle = 0.1;
    int steps_per_phase = 1;

    // ros init
    ros::init(argc, argv, "lgp_car_traj_planner");
    ros::NodeHandle n;
    tf::TransformListener tf_listener;

    n.getParam("/traj_planner/steps_per_phase", steps_per_phase);
    n.getParam("p_obstacle", p_obstacle);

    ros::Publisher trajectory_publisher = n.advertise<nav_msgs::Path>("/traj_planner/trajectory_0", 1000);
    ros::Publisher centerline_publisher = n.advertise<visualization_msgs::Marker>("/environment/center_line", 1000);

    BehaviorManager manager;

    // instanciate behaviors
    auto obstacle_avoidance_linear = std::shared_ptr<ObstacleAvoidanceLinear>(new ObstacleAvoidanceLinear(manager, steps_per_phase));
    manager.register_behavior("ObstacleAvoidanceLinear", obstacle_avoidance_linear);
    manager.set_current_behavior("ObstacleAvoidanceLinear");

    boost::function<void(const nav_msgs::Odometry::ConstPtr& msg)> odometry_callback =
            boost::bind(&BehaviorManager::odometry_callback, &manager, _1);
    auto odo_subscriber = n.subscribe("/lgp_car/odometry", 1000, odometry_callback);

    // connect behaviors
    boost::function<void(const std_msgs::Float32::ConstPtr& msg)> speed_callback =
            boost::bind(&ObstacleAvoidanceLinear::desired_speed_callback, obstacle_avoidance_linear.get(), _1);

    boost::function<void(const visualization_msgs::Marker::ConstPtr& msg)> obstacle_callback =
            boost::bind(&ObstacleAvoidanceLinear::obstacle_callback, obstacle_avoidance_linear.get(), _1);

    auto speed = n.subscribe("/gui_control/lgp_car/desired_speed", 1000, speed_callback);
    auto obstacle = n.subscribe("/lgp_obstacle_belief/marker", 1000, obstacle_callback);

    ros::Rate loop_rate(10);

    std::ofstream ofs(filename("linear", p_obstacle));

    double car_x = 0;
    int i = 0;
    while (ros::ok())
    {
        manager.plan();

        std::vector<nav_msgs::Path> trajectories = manager.get_trajectories();
        trajectory_publisher.publish(trajectories[0]);

        if((i - 10) % 50 == 0)
        {
            try
            {
                tf::StampedTransform transform;
                tf_listener.lookupTransform("/map", "/lgp_car", ros::Time(0), transform);

                car_x = transform(tf::Vector3(0,0,0)).x();

                centerline_publisher.publish(create_center_line(transform(tf::Vector3(0,0,0)).x()));
            }
            catch (tf::TransformException ex)
            {
                --i;
            }
        }
        ros::spinOnce();

        ++i;

        loop_rate.sleep();

        if(i%100==0)
        {
            log_to_file(ofs, n, car_x, i, manager.cost(), manager.planning_time());

            ROS_INFO_STREAM("cost:" << manager.cost() << " time:" << manager.planning_time());
        }
    }

    return 0;
}
