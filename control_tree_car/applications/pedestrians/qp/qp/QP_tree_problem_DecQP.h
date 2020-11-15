#pragma once

#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>
#include <qp/QP_tree_solver_base.h>
#include <Core/array.h>

#include <unordered_map>

#include <Optimization/qp_lagrangian.h>


/*
 * Use custom QP solver for solving the joint QP
 */

class QP_tree_problem_JointQP : public QP_tree_joint_solver_base
{
public:
    QP_tree_problem_JointQP(const MPC_model & mpc,
                    double u_min, double u_max);

private:
    VectorXd call_solver() override;

    DecOptConfig options;
};

/*
 * Solve QP in decentralized fashion
 */

class QP_tree_problem_DecQP : public QP_tree_solver_base
{
public:
    QP_tree_problem_DecQP(const MPC_model & mpc,
                    double u_min, double u_max);

    VectorXd solve(const Vector2d & x0, const Vector2d & xd, const Constraints & k,
                   int n_steps,
                   const std::vector<IntA> & varss,
                   const std::vector<Arr> & scaless);

private:
    /**
     * @brief build_qp
     * @param i branch id
     * @param n_steps number of steps on branch
     * @param varss local vars of the branch
     * @param scaless local scales of the branch
     * @param constraints mapping branch id -> constraints
     * @param x0 initial state
     * @param xd desired state
     * @return
     */
    std::shared_ptr<QP_Problem> build_qp(int i,
                                         int n_steps,
                                         const std::vector<IntA>& varss,
                                         const std::vector<Arr>& scaless,
                                         const std::unordered_map<int, Constraints> & constraints,
                                         const Vector2d & x0,
                                         const Vector2d & xd
                                         ) const;

    DecOptConfig options;
};

std::vector<arr> get_compressed_masks(int n_steps, int dim, const std::vector<IntA>& varss, IntA& var, IntA& global_to_branch);
std::vector<Arr> get_compressed_scales(const std::vector<Arr>& scaless);
std::unordered_map<int, Constraints> get_compressed_constraints(const Constraints & k, const IntA& var, const IntA& global_to_branch); // branch to constraints
