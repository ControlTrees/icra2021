#include <control_tree/qp/QP_tree_problem_CGAL.h>

QP_tree_problem_CGAL::QP_tree_problem_CGAL(const MPC_model & mpc, double u_min, double u_max)
    : QP_tree_problem_base(mpc, u_min, u_max)
{
    qp = Program(CGAL::SMALLER, true, u_min, true, u_max);

    control_bounds_in_A_ = false; // makes A matrix slightly smaller
}

VectorXd QP_tree_problem_CGAL::call_solver()
{
    // H-D
    for(auto i = 0; i < H.rows(); ++i)
    {
        for(auto j = 0; j <= i; ++j)
        {
          qp.set_d(i, j, H(i, j));  // !!specify 2D!!
        }
    }

    // C
    for(auto i = 0; i < C.rows(); ++i)
    {
        qp.set_c(i, C(i));
    }

    // KA
    for(auto i = 0; i < KA.rows(); ++i)
    {
        for(auto j = 0; j < KA.cols(); ++j)
        {
            qp.set_a(j, i, KA(i, j)); // inverted order j, i!!!!!
        }
    }

    // Up
    for(auto i = 0; i < Up.rows(); ++i)
    {
        qp.set_b(i, Up(i));
    }

    ///
    //std::cout << "start solving" << std::endl;

    Solution s = CGAL::solve_quadratic_program(qp, ET());
    s.solves_quadratic_program(qp);

    // output solution
    //std::cout << s;
    //std::cout << "Objective value:\n" << CGAL::to_double(s.objective_value()) << std::endl;

    if(s.is_optimal()) //!s.is_infeasible())
    {
        Eigen::VectorXd U = Eigen::VectorXd::Zero(H.rows()) ;

        int i = 0;
        for(auto v = s.variable_values_begin(); v != s.variable_values_end(); ++v)
        {
            double u = CGAL::to_double(*v);
            U(i) = u;
            ++i;
        }

        return U;
    }

    return Eigen::VectorXd();
}
