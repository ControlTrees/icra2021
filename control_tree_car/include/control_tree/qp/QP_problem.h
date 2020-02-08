#pragma once

#include <control_tree/qp/MPC_model.h>

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef double ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

// program and solution types
typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

struct QP_problem
{
    QP_problem(const MPC_model & mpc, int n_steps)
        : mpc(mpc)
        , n_steps(n_steps)
    {
        auto Q_bar = mpc.get_Q_bar(n_steps);
        auto R_bar = mpc.get_R_bar(n_steps);

        S = mpc.get_S(n_steps);
        T = mpc.get_T(n_steps);

        H = 2 * (R_bar + S.transpose() * Q_bar * S);
        F = 2 * (T.transpose() * Q_bar * S);
        G = 2 * Q_bar * S;
    }

    VectorXd solve(const Vector2d & x0, const Vector2d & xd, const Vector2d & xmax, double u_min, double u_max)
    {
        Eigen::VectorXd Xd = mpc.get_Xd(xd, n_steps);
        Eigen::VectorXd Xmax = mpc.get_Xd(xmax, n_steps);

        Program qp (CGAL::SMALLER, true, u_min, true, u_max);

        //std::cout << "x0:\n" << x0 << std::endl;
        //std::cout << "Xd:\n" << Xd << std::endl;
        //std::cout << "Xmax:\n" << Xmax << std::endl;
        //std::cout << "H-1:\n" << H.inverse() << std::endl;

        // H-D
        for(auto i = 0; i < H.rows(); ++i)
        {
            for(auto j = 0; j <= i; ++j)
            {
              qp.set_d(i, j, H(i, j));  // !!specify 2D!!
            }
        }

        // C
        auto C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();
        for(auto i = 0; i < C.rows(); ++i)
        {
            qp.set_c(i, C(i));
        }

        /// Constraints
        Eigen::MatrixXd Sextract = Eigen::MatrixXd::Zero(n_steps, 2 * n_steps);
        for(auto i = 0; i < n_steps; ++i)
        {
            Sextract(i, 2*i) = 1;
        }

        auto KA = Sextract * S;
        for(auto i = 0; i < KA.rows(); ++i)
        {
            for(auto j = 0; j < KA.cols(); ++j)
            {
                qp.set_a(j, i, KA(i, j)); // inverted order j, i!!!!!
            }
        }


        // Kb
        VectorXd Kb = Sextract* (Xmax - T * x0);
        for(auto i = 0; i < Kb.rows(); ++i)
        {
            qp.set_b(i, Kb(i));
        }

        ///
        //std::cout << "start solving" << std::endl;
        Solution s = CGAL::solve_quadratic_program(qp, ET());
        assert (s.solves_quadratic_program(qp));

        // output solution
        //std::cout << s;
        //std::cout << "Objective value:\n" << CGAL::to_double(s.objective_value()) << std::endl;

        Eigen::VectorXd U(n_steps) ;
        int i = 0;
        for(auto v = s.variable_values_begin(); v != s.variable_values_end(); ++v)
        {
            double u = CGAL::to_double(*v);
            U(i) = u;
            ++i;
        }

        return U;
    }

    const MPC_model & mpc;
    int n_steps;

    MatrixXd S;
    MatrixXd T;

    MatrixXd H;
    MatrixXd F;
    MatrixXd G;
};
