#pragma once

#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>

class QP_tree_solver_base
{
public:
    QP_tree_solver_base(const MPC_model & mpc,
                    double u_min, double u_max);

    virtual VectorXd solve(const Vector2d & x0, const Vector2d & xd, const Constraints & k,
                   int n_steps,
                   const std::vector<IntA> & varss,
                   const std::vector<Arr> & scaless) = 0;

protected:
    const MPC_model & mpc;

    const double u_min_;
    const double u_max_;
};


class QP_tree_joint_solver_base : public QP_tree_solver_base
{
public:
    QP_tree_joint_solver_base(const MPC_model & mpc,
                    double u_min, double u_max);

    VectorXd solve(const Vector2d & x0, const Vector2d & xd, const Constraints & k,
                   int n_steps,
                   const std::vector<IntA> & varss,
                   const std::vector<Arr> & scaless);

    MatrixXd get_H() const {return H;}
    MatrixXd get_KA() const {return KA;}

private:
    virtual VectorXd call_solver() = 0;
//    VectorXd solve_unconstrained(const Vector2d x0, const Vector2d xd)
//    {
//        return -H.inverse() * F.transpose() * x0;
//    }

protected:
    bool control_bounds_in_A_;

    MatrixXd Q_bar; // state cost
    MatrixXd R_bar; // control cost

    MatrixXd S;
    MatrixXd T;

    MatrixXd F;
    MatrixXd G;

    // final matrices used by the solver
    MatrixXd H;
    VectorXd C;
    MatrixXd KA;
    VectorXd Up;
    VectorXd Lo;
};
