#include <fstream>
#include <ros/package.h>
#include <tf/transform_listener.h>

#include <control_tree/core/behavior_manager.h>
#include <control_tree/qp/stopline_qp_tree.h>
#include <control_tree/ros/pedestrian_common.h>

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Launch lgp car control, pedestrian tree..");

    int steps_per_phase = 1;
    int n_branches = 2;
    int n_pedestrians = 1;
    double p_crossing = 0;

    // ros init
    ros::init(argc, argv, "lgp_car_pedestrian_qp_tree_planner");
    ros::NodeHandle n;

    n.getParam("n_branches", n_branches);
    n.getParam("n_pedestrians", n_pedestrians);
    n.getParam("p_crossing", p_crossing);

    tf::TransformListener tf_listener;
    n.getParam("/traj_planner/steps_per_phase", steps_per_phase);
    ros::Publisher trajectory_publisher_1 = n.advertise<nav_msgs::Path>("/traj_planner/trajectory_11", 1000);
    ros::Publisher trajectory_publisher_2 = n.advertise<nav_msgs::Path>("/traj_planner/trajectory_12", 1000);
    ros::Publisher trajectory_publisher_3 = n.advertise<nav_msgs::Path>("/traj_planner/trajectory_13", 1000);
    //ros::Publisher ctrl_publisher = n.advertise<geometry_msgs::Twist>("/lgp_car/vel_cmd", 1000);
    ros::Publisher border_publisher = n.advertise<visualization_msgs::Marker>("/environment/center_line", 1000);

    BehaviorManager manager;

    // instanciate behaviors
    auto stopline = std::shared_ptr<StopLineQPTree>(new StopLineQPTree(manager, n_branches, steps_per_phase));
    manager.register_behavior("StopLineTree", stopline);
    manager.set_current_behavior("StopLineTree");

    boost::function<void(const nav_msgs::Odometry::ConstPtr& msg)> odometry_callback =
            boost::bind(&BehaviorManager::odometry_callback, &manager, _1);
    auto odo_subscriber = n.subscribe("/lgp_car/odometry", 1000, odometry_callback);

    // connect behaviors
    boost::function<void(const std_msgs::Float32::ConstPtr& msg)> speed_callback =
            boost::bind(&StopLineQPTree::desired_speed_callback, stopline.get(), _1);

    boost::function<void(const visualization_msgs::MarkerArray::ConstPtr& msg)> stopline_callback =
            boost::bind(&StopLineQPTree::stopline_callback, stopline.get(), _1);

    auto speed = n.subscribe("/gui_control/lgp_car/desired_speed", 1000, speed_callback);
    auto stopline_sub = n.subscribe("/lgp_pedestrian_belief/marker_array", 1000, stopline_callback);

    ros::Rate loop_rate(10);

    std::ofstream ofs(filename("tree", p_crossing, n_pedestrians, n_branches));

    double car_x = 0;
    int i = 0;
    while (ros::ok())
    {
        manager.plan();

        std::vector<nav_msgs::Path> trajectories = manager.get_trajectories();
        trajectory_publisher_1.publish(trajectories[0]);
        if(trajectories.size() > 1)
            trajectory_publisher_2.publish(trajectories[1]);
        if(trajectories.size() > 2)
            trajectory_publisher_3.publish(trajectories[2]);

        //ctrl_publisher.publish(manager.get_ctrl());

        if((i - 10) % 50 == 0)
        {
            try
            {
                tf::StampedTransform transform;
                tf_listener.lookupTransform("/map", "/lgp_car", ros::Time(0), transform);

                car_x = transform(tf::Vector3(0,0,0)).x();

                border_publisher.publish(create_road_border(car_x));
            }
            catch (tf::TransformException ex)
            {
                --i;
            }
        }
        ros::spinOnce();

        ++i;

        loop_rate.sleep();

        //
        if(i%100==0)
        {
            log_to_file(ofs, n, car_x, i, manager.cost(), manager.velocity(), manager.planning_time());
        }
    }

    return 0;
}
