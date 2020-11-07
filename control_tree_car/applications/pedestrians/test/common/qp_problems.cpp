#include "qp_problems.h"

#include <ostream>
#include <iostream>
#include <chrono>

#include <qp/QP_tree_problem_OSQP.h>
#include <qp/QP_tree_problem_DecQP.h>

QP_problem create_2_branches_2_steps_constrained(double p)
{
    // MODEL
    MPC_model model(0.5, 1.0, 3.0);

    // TREE
    Tree2Branches2Steps tree(p);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, 5.0;

    Vector2d xd;
    xd << 0, 10.0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(20, 0), Vector2d(1, 0));

    return {model, tree, k, x0, xd};
}

QP_problem create_2_branches_4_steps_constrained(double p)
{
    auto n_steps_per_phase = 4;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 3.0);

    // TREE
    auto tree = TreePb::refined(Tree2Branches(p), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, 5.0;

    Vector2d xd;
    xd << 0, 10.0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(20, 0), Vector2d(1, 0));

    return {model, tree, k, x0, xd};
}

QP_problem create_3_branches_4_steps_constrained(double p)
{
    auto n_steps_per_phase = 4;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 3.0);

    // TREE
    auto tree = TreePb::refined(Tree3Branches(0.3, 0.3), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, 5.0;

    Vector2d xd;
    xd << 0, 10.0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(20, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(15, 0), Vector2d(1, 0));

    return {model, tree, k, x0, xd};
}

QP_problem create_4_branches_4_steps_constrained(double p)
{
    auto n_steps_per_phase = 4;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 3.0);

    // TREE
    auto tree = TreePb::refined(Tree4Branches(0.2, 0.2, 0.2), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, 5.0;

    Vector2d xd;
    xd << 0, 10.0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(20, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(17.5, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(15, 0), Vector2d(1, 0));

    return {model, tree, k, x0, xd};
}

QP_problem create_2_stages_branching(double p)
{
    auto n_steps_per_phase = 4;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 3.0);

    // TREE
    auto tree = Tree2Stages(0.2);
    auto refined_tree = TreePb::refined(tree, n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0.0, 5.0;

    Vector2d xd;
    xd << 0, 5.0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);

    k.add_constraint(0, Vector2d(9, 0) , Vector2d(1, 0));
    k.add_constraint(1, Vector2d(11, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(13, 0), Vector2d(1, 0));
    k.add_constraint(3, Vector2d(15, 0), Vector2d(1, 0));

    Constraints refined_k = Constraints::refined(k, n_steps_per_phase);

    return {model, refined_tree, refined_k, x0, xd};
}

QP_problem create_paper_1_branch_4_steps_constrained(double p)
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 13.41;

    const auto offset = 2.5 + 7.5;
    const auto d_safety = 2.5;
    const auto d0 = offset - d_safety + 20;
    const auto d1 = offset - d_safety + 30;
    const auto d2 = offset - d_safety + 40;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 10.0);

    // TREE
    auto tree = TreePb::refined(Tree1Branch(), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, v0;

    Vector2d xd;
    xd << 0, v0;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(d0, 0), Vector2d(1, 0));

    IntA last;
    int last_index = int(tree.varss[0].size())-1;
    last.push_back(last_index);

    auto last_speed = [](double d, double v0, double T)
    {
        double a = 0.5 * v0 * v0 / d;
        return v0 - a * T;
    };

    k.add_constraint(0, Vector2d(0, last_speed(d0, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);

    return {model, tree, k, x0, xd};
}

QP_problem create_paper_4_branches_4_steps_constrained(double p)
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 13.41;

    const auto offset = 2.5 + 7.5;
    const auto d_safety = 2.5;
    const auto d0 = offset - d_safety + 20;
    const auto d1 = offset - d_safety + 30;
    const auto d2 = offset - d_safety + 40;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 10.0);

    // TREE
    const auto p0 = p;
    const auto p1 = (1-p) * p;
    const auto p2 = (1-p)*(1-p) * p;
    const auto tree = TreePb::refined(TreeNBranches({p0, p1, p2}), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, v0;

    Vector2d xd;
    xd << 0, vdesired;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(d0, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(d1, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(d2, 0), Vector2d(1, 0));

    IntA last;
    int last_index = int(tree.varss[0].size())-1;
    last.push_back(last_index);

    auto last_speed = [](double d, double v0, double T)
    {
        double a = 0.5 * v0 * v0 / d;
        return v0 - a * T;
    };

    k.add_constraint(0, Vector2d(0, last_speed(d0, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(1, Vector2d(0, last_speed(d1, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(2, Vector2d(0, last_speed(d2, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);

    return {model, tree, k, x0, xd};
}

QP_problem create_10_branches_4_steps_constrained()
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 13.41;

    const auto offset = 7.5;
    const auto d_safety = 2.5;
    const auto d0 = offset - d_safety + 20;
    const auto d1 = offset - d_safety + 30;
    const auto d2 = offset - d_safety + 40;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 10.0);

    // TREE
    const auto tree = TreePb::refined(Tree10Branches(), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, v0;

    Vector2d xd;
    xd << 0, vdesired;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(d0, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(d1, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(d2, 0), Vector2d(1, 0));

    IntA last;
    int last_index = int(tree.varss[0].size())-1;
    last.push_back(last_index);

    auto last_speed = [](double d, double v0, double T)
    {
        double a = 0.5 * v0 * v0 / d;
        return v0 - a * T;
    };

    k.add_constraint(0, Vector2d(0, last_speed(d0, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(1, Vector2d(0, last_speed(d1, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(2, Vector2d(0, last_speed(d2, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);

    return {model, tree, k, x0, xd};
}

QP_problem create_20_branches_4_steps_constrained()
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 13.41;

    const auto offset = 7.5;
    const auto d_safety = 2.5;
    const auto d0 = offset - d_safety + 20;
    const auto d1 = offset - d_safety + 30;
    const auto d2 = offset - d_safety + 40;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 10.0);

    // TREE
    const auto tree = TreePb::refined(TreeNBranches(20), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, v0;

    Vector2d xd;
    xd << 0, vdesired;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(d0, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(d1, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(d2, 0), Vector2d(1, 0));
    k.add_constraint(3, Vector2d(d1, 0), Vector2d(1, 0));
    k.add_constraint(10, Vector2d(d2, 0), Vector2d(1, 0));
    k.add_constraint(16, Vector2d(d0, 0), Vector2d(1, 0));

    IntA last;
    int last_index = int(tree.varss[0].size())-1;
    last.push_back(last_index);

    auto last_speed = [](double d, double v0, double T)
    {
        double a = 0.5 * v0 * v0 / d;
        return v0 - a * T;
    };

    k.add_constraint(0, Vector2d(0, last_speed(d0, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(1, Vector2d(0, last_speed(d1, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);
    k.add_constraint(2, Vector2d(0, last_speed(d2, v0, last_index / n_steps_per_phase)), Vector2d(0, 1), last);

    return {model, tree, k, x0, xd};
}

QP_problem create_N_branches_4_steps_constrained(int N)
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 13.41;

    const auto offset = 7.5;
    const auto d_safety = 2.5;
    const auto d0 = offset - d_safety + 15;
    const auto dN = offset - d_safety + 45;
    const auto d = (45 - 20) / N;

    // MODEL
    MPC_model model(1.0 / n_steps_per_phase, 1.0, 5.0);

    // TREE
    const auto tree = TreePb::refined(TreeNBranches(N), n_steps_per_phase);

    // INITIAL STATE
    Vector2d x0;
    x0 << 0, v0;

    Vector2d xd;
    xd << 0, vdesired;

    // CONSTRAINTS
    Constraints k(tree.n_steps, tree.varss);
    for(auto i = 0; i < N; ++i)
    {
        auto di = d0 + i * d;
        k.add_constraint(i, Vector2d(di, 0), Vector2d(1, 0));
    }

    return {model, tree, k, x0, xd};
}

// debug Configurations found in simulation
QP_problem replicate_simulation_1()
{
    const auto n_steps_per_phase = 4;
    const auto v0 = 13.41;  //13.41; // 48 km/h // 30 mph
    const auto vdesired = 48/3.6;

    MPC_model model(1.0 / n_steps_per_phase, 1.0, 5.0);

    const auto tree = TreePb::refined(TreeNBranches({0.150897, 0.384692, 0.454784, 0}), n_steps_per_phase);
//    const auto tree = TreePb::refined(TreeNBranches({0.15, 0.15, 0.15, 0.15, 0.2}), n_steps_per_phase);

    const double ox = 1071.41;
    Vector2d x0;
    x0 << 0, 6.48015;

    Vector2d xd;
    xd << 0, vdesired;

    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(1080.81 - ox, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(1085.35 - ox, 0), Vector2d(1, 0));
    k.add_constraint(2, Vector2d(1106.66 - ox, 0), Vector2d(1, 0));
    k.add_constraint(3, Vector2d(1000, 0), Vector2d(1, 0));
    //k.add_constraint(4, Vector2d(200, 0), Vector2d(1, 0));


    return {model, tree, k, x0, xd};

    //[ INFO] [1598170096.753487138]: x: 1071.41 v: 6.48015
    //[ INFO] [1598170096.753537535]: Belief state: 0.150897 0.384692 0.454784 0 0.00962723
    //[ INFO] [1598170096.753566643]: 0--th stopline, x: 1080.81
    //[ INFO] [1598170096.753586247]: 1--th stopline, x: 1085.35
    //[ INFO] [1598170096.753603506]: 2--th stopline, x: 1106.66
    //[ INFO] [1598170096.753619602]: 3--th stopline, x: inf

}

VectorXd QPTest::plan_OSQP(const QP_problem &pb, bool _plot, const std::string & filename)
{
    QP_tree_problem_OSQP solver(pb.model, u_min, u_max);

    auto start = std::chrono::high_resolution_clock::now();

    const auto & U = solver.solve(pb.x0, pb.xd, pb.k, pb.tree.n_steps, pb.tree.varss, pb.tree.scaless);

    auto end = std::chrono::high_resolution_clock::now();
    execution_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //
    std::cout << "n branches: " << pb.tree.varss.size() << " execution time:" << execution_time_ms << std::endl;
    //
    const auto & X = pb.model.predict_trajectory(pb.x0, U, pb.tree.varss);

    // plot
    if(_plot) plot_XU(X, U, pb);
    if(filename.size()) save_XU(X, U, pb, filename);

    return U;
}

VectorXd QPTest::plan_JointQP(const QP_problem &pb, bool _plot, const std::string & filename)
{
    QP_tree_problem_JointQP solver(pb.model, u_min, u_max);

    auto start = std::chrono::high_resolution_clock::now();

    const auto & U = solver.solve(pb.x0, pb.xd, pb.k, pb.tree.n_steps, pb.tree.varss, pb.tree.scaless);

    auto end = std::chrono::high_resolution_clock::now();
    execution_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //
    std::cout << "n branches: " << pb.tree.varss.size() << " execution time (ms):" << execution_time_ms << std::endl;
    //
    const auto & X = pb.model.predict_trajectory(pb.x0, U, pb.tree.varss);

    // plot
    if(_plot) plot_XU(X, U, pb);
    if(filename.size()) save_XU(X, U, pb, filename);

    return U;
}

VectorXd QPTest::plan_DecQP(const QP_problem &pb, bool _plot, const std::string & filename)
{
    QP_tree_problem_DecQP solver(pb.model, u_min, u_max);

    auto start = std::chrono::high_resolution_clock::now();

    const auto & U = solver.solve(pb.x0, pb.xd, pb.k, pb.tree.n_steps, pb.tree.varss, pb.tree.scaless);

    auto end = std::chrono::high_resolution_clock::now();
    execution_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //
    std::cout << "n branches: " << pb.tree.varss.size() << " execution time (ms):" << execution_time_ms << std::endl;
    //
    const auto & X = pb.model.predict_trajectory(pb.x0, U, pb.tree.varss);

    // plot
    if(_plot) plot_XU(X, U, pb);
    if(filename.size()) save_XU(X, U, pb, filename);

    return U;
}

void QPTest::plot_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb) const
{
    plot([&U](int i){return U(i);}, pb.tree.varss, pb.tree.scaless, acc_axis);
    plot([&X](int i){return X(2*i+1);}, pb.tree.varss, pb.tree.scaless, vel_axis);
    plot([&X](int i){return X(2*i);}, pb.tree.varss, pb.tree.scaless, x_axis);
}

void QPTest::save_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb, const std::string & filename) const
{
    std::ofstream file(filename);

    const auto dt = pb.model.A(0, 1);

    /// Save controls

    // extract common part of the tree
    if(pb.tree.varss.size()>1)
    {
        file << "#" << " " << "t" << " " << "common u" << std::endl;
        IntA vars;
        for(auto i = 0; i < pb.tree.varss[0].size(); ++i)
        {
            if(pb.tree.scaless[0][i] == 1.00)
            {
                vars.push_back(pb.tree.varss[0][i]);
            }
            else
            {
                break;
            }
        }

        save([&U](int i){return U(i);}, dt, vars, file);
    }

    // save all branches
    file << "#" << " " << "t" << " " << "u" << std::endl;
    save([&U](int i){return U(i);}, dt, pb.tree.varss, pb.tree.scaless, file);


    /// Save state
    // we need to append x0 and modify the vars accordingly(hacky)!
    VectorXd Xfull(X.rows() + pb.x0.rows());
    Xfull << pb.x0[0], pb.x0[1], X;
    auto varss = pb.tree.varss;
    varss[0].push_back(varss[0].size());
    for(auto i = 1; i < varss.size(); ++i)
    {
        IntA vars;
        bool switched = false;

        for(auto j = 0; j < pb.tree.varss[i].size(); ++j)
        {
            if(!switched)
            {
                vars.push_back(pb.tree.varss[i][j]);

                if(pb.tree.varss[i][j]+1 != pb.tree.varss[i][j+1])
                {
                    vars.push_back(pb.tree.varss[i][j]+1);
                    switched = true;
                }
            }
            else
            {
                vars.push_back(pb.tree.varss[i][j]+1);
            }
        }

        varss[i] = vars;
    }

    file << "#" << " " << "t" << " " << "x" << std::endl;
    save([&Xfull](int i){return Xfull(2*i);}, dt, varss, pb.tree.scaless, file);

    file << "#" << " " << "t" << " " << "v" << std::endl;
    save([&Xfull](int i){return Xfull(2*i+1);}, dt, varss, pb.tree.scaless, file);

    file.close();
}
