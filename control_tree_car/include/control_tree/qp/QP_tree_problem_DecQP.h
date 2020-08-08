#pragma once

#include <control_tree/qp/MPC_model.h>
#include <control_tree/qp/QP_constraints.h>
#include <control_tree/qp/QP_tree_problem_base.h>

class QP_tree_problem_DecQP : public QP_tree_problem_base
{
public:
    QP_tree_problem_DecQP(const MPC_model & mpc,
                    double u_min, double u_max);

private:
    VectorXd call_solver() override;
};
