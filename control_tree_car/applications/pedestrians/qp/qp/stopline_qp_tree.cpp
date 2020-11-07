#include <common/behavior_manager.h>
#include <common/utility.h>
#include <qp/stopline_qp_tree.h>
#include <qp/QP_tree_problem_DecQP_utils.h>

StopLineQPTree::StopLineQPTree(BehaviorManager& behavior_manager, int n_branches, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , n_branches_(n_branches)
    , steps_(steps_per_phase)
    , model_(1.0 / steps_per_phase, 1.0, 5.0)
    , solver_(model_, u_min_, u_max_)
    , v_desired_(50/3.6)
    , stoplines_(n_branches_ > 1 ? n_branches_ - 1 : 1)
    , optimization_run_(false)
    , optimization_error_(false)
    , acc_plotter_(n_branches, "acceleration", "[-8.0:2.0]")
    , vel_plotter_(n_branches, "velocity", "[0.0:15.0]")
{    
    create_tree();
}

void StopLineQPTree::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void StopLineQPTree::stopline_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    const auto N = msg->markers.size() / 3; // 3 markers per obstacle (actual pedestrian, crossing probability, forward probability)
    const auto & o = manager_.odometry();

    //ROS_INFO_STREAM( "number of pedestrians:" << N );

    stoplines_.resize(N);

    for(auto i = 0; i < N; ++i)
    {
        /// position and geometry
        const auto stop = msg->markers[3*i].pose.position.x - 5.3 + 1.2 - 2.5; // saftey distance
        /// existence probability
        const auto probability = msg->markers[3*i+1].color.a;

        if(o.x < stop + 1 && probability > 0.01)
        {
            stoplines_[i].x = stop;
            stoplines_[i].p = probability;
        }
        else
        {
            stoplines_[i].x = std::numeric_limits<double>::infinity();
            stoplines_[i].p = 0;
        }
    }

    // sort stoplines
    std::sort(stoplines_.begin(), stoplines_.end(), [](const Stopline & a, const Stopline & b){return a.x < b.x;});

//    for(auto i = 0; i < stoplines_.size(); ++i)
//    {
//        ROS_INFO_STREAM( i << "--th stopline " << stoplines_[i].x);
//    }

    create_tree();
}

TimeCostPair StopLineQPTree::plan()
{
    //ROS_INFO( "plan.." );

    // INITIAL STATES
    auto o = manager_.odometry();
    x0_ = Vector2d();
    x0_ << 0, o.v;

    // DESIRED
    Vector2d xd;
    xd << 0, v_desired_;

    // CONSTRAINT
    //ROS_INFO_STREAM("--------------");

    Constraints k(tree_->n_steps, tree_->varss);
    for(auto i = 0; i < (n_branches_ > 1 ? n_branches_ - 1 : 1); ++i)
    {
        //ROS_INFO_STREAM( i << " th stopline " << stoplines_[i].x);

        double xmax = 0;
        if( stoplines_[i].p < 0.01
         || o.x > stoplines_[i].x + 1 // far behind
          )
        {
            xmax = 1000, 0; // rather remove constraint!
        }
        else
        {
            xmax = stoplines_[i].x - o.x;
        }

        k.add_constraint(i, Vector2d(xmax, 0), Vector2d(1, 0)); // xmax on branch 0, all indices
    }
    // v is zero on branch 0, last index

    /// SOLVER
    //ROS_INFO( "solve.." );

    auto start = std::chrono::high_resolution_clock::now();

    auto U = solver_.solve(x0_, xd, k, tree_->n_steps, tree_->varss, tree_->scaless);

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    ROS_INFO( "[tree qp] execution time (ms): %f", execution_time_us / 1000 );

    debug(U, o);

    optimization_error_ = !validate_and_save_solution(U, o);

    if(optimization_error_)
    {
        ROS_INFO_STREAM( "Generate control for emergency brake, o.x:" << o.x << " o.v:" << o.v << " v_desired_:" << v_desired_);

        const auto & U = emergency_brake(o.v, *tree_, steps_, u_min_);

        bool ok_emergency_brake = validate_and_save_solution(U, o);

        if(!ok_emergency_brake)
        {
            ROS_WARN( "Planning error :((" );
            ROS_WARN_STREAM(U);
        }
    }

    // plots
//    if(U_sol_.rows())
//    {
//        acc_plotter_.update(tree_->varss, tree_->scaless, [this](int i){return U_sol_(i);});
//        vel_plotter_.update(tree_->varss, tree_->scaless, [this](int i){return X_sol_(2*i+1);});
//    }

    std::cerr << "speed:" << o.v << " costs:" << model_.cost(x0_, U_sol_[0], xd) << std::endl;

    return {execution_time_us / 1000000, model_.cost(x0_, U_sol_[0], xd)};
}

bool StopLineQPTree::validate_and_save_solution(const VectorXd & U, const OdometryState & o)
{
    if(U.rows())
    {
        //ROS_INFO( "Solved :)" );
        auto x0 = x0_;
        x0(0) += o.x;

        auto X = model_.predict_trajectory(x0, U, tree_->varss);

        if(valid(U, X))
        {
            U_sol_ = U;
            X_sol_ = X;

            optimization_run_ = true;

            return true;
        }
        else
        {
            ROS_WARN( "Optimization succeeded but invalid trajectory" );
        }
    }
    else
    {
        ROS_WARN( "Infeasible :(" );
    }

    return false;
}

std::vector<nav_msgs::Path> StopLineQPTree::get_trajectories()
{
    std::vector<nav_msgs::Path> paths(tree_->varss.size());

    if(!optimization_run_ || optimization_error_)
    {
        return paths;
    }

    auto msg_from_s = [](double x)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
    };

    for(auto l = 0; l < tree_->varss.size(); ++l)
    {
        {
            nav_msgs::Path msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            msg.poses.reserve(tree_->varss[l].size() + 1);

            msg.poses.push_back(msg_from_s(x0_[0]));

            if(l > 0 && tree_->scaless[l].back() < 0.01) // particular cas to avoid sending degenerated trajs
            {
                paths[l] = paths[l-1];
            }
            else
            {
                // nominal case
                for(auto k : tree_->varss[l])
                {
                    msg.poses.push_back(msg_from_s(X_sol_[2*k]));
                }

                paths[l] = msg;
            }
        }
    }

    return paths;
}

void StopLineQPTree::create_tree()
{
    //ROS_INFO_STREAM("create tree..");

    if(n_branches_ == 1)
    {
        tree_ =  TreePb::refined(std::make_shared<Tree1Branch>(), steps_);
    }
    else
    {
        const auto ps = fuse_probabilities(stoplines_, n_branches_ - 1);

        CHECK_EQ(ps.size(), n_branches_ - 1, "discrepancy in number of branches in control tree");

        tree_ = TreePb::refined(std::make_shared<TreeNBranches>(ps), steps_);
    }

    CHECK_EQ(tree_->scaless.size(), n_branches_, "discrepancy in number of branches in control tree");

    //ROS_INFO_STREAM("stoplines probabilit: "  << stoplines_[0].p << " " << stoplines_[1].p);
    //            //ROS_INFO_STREAM("build tree 3 branches "  << p[0] << " " << p[1] << " " << 1 - p[0] - p[1] );
}

bool StopLineQPTree::valid(const VectorXd & U, const VectorXd & X) const
{
    bool valid = true;
    const double eps = 0.15;

    for(auto i = 0; i < U.rows(); ++i)
    {
        if(std::isnan(U[i]))
        {
            ROS_WARN_STREAM("nan in U vector!");
            valid = false;
        }

        if(!( u_min_ - eps <= U[i] && U[i] <= u_max_ + eps))
        {
            ROS_WARN_STREAM("control out of bounds!:" << U[i]);
            valid = false;
        }
    }

    for(auto i = 0; i < X.rows(); ++i)
    {
        if( std::isnan(X[i]))
        {
            ROS_WARN_STREAM("nan in X vector!");
            valid = false;
        }
    }

    return valid;
}

void StopLineQPTree::debug(const VectorXd & U, const OdometryState & o) const
{
    double jerk = 0;
    for(auto i = 1; i < 2; ++i)
    {
        jerk = U[i] - U[i-1];

        if(fabs(jerk) > 0.5)
        {
            ROS_WARN_STREAM("High Jerk at trajectory start!!!");

            // odo
            ROS_INFO_STREAM( "x: " << o.x << " v: " << o.v);

            // pb
            auto bs = get_belief_state(tree_->scaless);

            ROS_INFO_STREAM("Belief state:" << bs);

            for(auto j = 0; j < stoplines_.size(); ++j)
            {
                ROS_INFO_STREAM( j << "--th stopline, x: " << stoplines_[j].x);
            }
        }
    }
}

VectorXd emergency_brake(const double v, const TreePb & tree, int steps_per_phase, double u)
{
    VectorXd U = VectorXd::Zero(tree.n_steps);

    for(auto vars : tree.varss)
    {
        double v_loop = v;

        for(auto i = 0; i < vars.size(); ++i)
        {
            double remaining_braking_time = fabs(v_loop / u);

            if(remaining_braking_time > 1.0 / steps_per_phase)
            {
                U[vars[i]] = u;
                v_loop += u / steps_per_phase;
            }
            else
            {
                // last step
                U[vars[i]] = -v_loop / steps_per_phase;
                v_loop = 0;
                break;
            }
        }
    }

    return U;
}

std::vector<double> fuse_probabilities(const std::vector<Stopline> & stoplines, int n)
{
    std::vector<double> probabilities(n);

    double q = 1.0;
    for(auto i = 0; i < n; ++i)
    {
        probabilities[i] = q * stoplines[i].p;
        q *= (1 - stoplines[i].p);
    }

    for(auto j = n; j < stoplines.size(); ++j)
    {
        probabilities[n-1] += q * stoplines[j].p;
        q *= (1 - stoplines[j].p);
    }

    return probabilities;
}

VectorXd emergency_brake(double v, int n_phases, int steps_per_phase, double u)
{
    VectorXd U = VectorXd::Zero(n_phases * steps_per_phase);

    for(auto i = 0; i < U.rows(); ++i)
    {
        double remaining_braking_time = fabs(v / u);

        if(remaining_braking_time > 1.0 / steps_per_phase)
        {
            U[i] = u;
            v += u / steps_per_phase;
        }
        else
        {
            // last step
            U[i] = -v / steps_per_phase;
            v = 0;
            break;
        }
    }

    return U;
}
