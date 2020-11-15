#pragma once

#include <Core/array.h>
#include <qp/QP_constraints.h>
#include <unordered_map>

arr convert(const MatrixXd& M);
VectorXd convert(const arr& v);
arr createK(const MatrixXd& KA);
arr createU(const VectorXd& Up, const VectorXd& Lo);
IntA to_local(const IntA& global_indices, const IntA& global_to_branch);

std::vector<Arr> get_compressed_scales(const std::vector<Arr>& scaless);
Arr get_one_scale(const std::vector<Arr>& scaless);
std::vector<arr> get_compressed_masks(int n_steps, int dim, const std::vector<IntA>& varss, IntA& var, IntA& global_to_branch);

arr get_belief_state(const std::vector<Arr>& scaless);
std::unordered_map<int, Constraints> get_compressed_constraints(const Constraints & k, const IntA& var, const IntA& global_to_branch); // branch to constraints
