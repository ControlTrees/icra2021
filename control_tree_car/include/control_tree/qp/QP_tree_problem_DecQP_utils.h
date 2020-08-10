#pragma once

#include <control_tree/qp/MPC_model.h>
#include <control_tree/qp/QP_constraints.h>
#include <control_tree/qp/QP_tree_solver_base.h>
#include <Core/array.h>

#include <unordered_map>

#include <Optimization/qp_lagrangian.h>

arr convert(const MatrixXd& M);
VectorXd convert(const arr& v);
arr createK(const MatrixXd& KA);
arr createU(const VectorXd& Up, const VectorXd& Lo);
IntA to_local(const IntA& global_indices, const IntA& global_to_branch);

std::vector<arr> get_compressed_masks(int n_steps, int dim, const std::vector<IntA>& varss, IntA& var, IntA& global_to_branch);
std::vector<Arr> get_compressed_scales(const std::vector<Arr>& scaless);
std::unordered_map<int, Constraints> get_compressed_constraints(const Constraints & k, const IntA& var, const IntA& global_to_branch); // branch to constraints