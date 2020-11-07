#include <qp/QP_tree_problem_DecQP_utils.h>

arr convert(const MatrixXd& M)
{
  arr m(M.rows(), M.cols());

  for(uint i = 0; i < M.rows(); ++i)
  {
    for(uint j = 0; j < M.cols(); ++j)
    {
      m(i, j) = M(i,j);
    }
  }

  return m;
}

VectorXd convert(const arr& v)
{
  VectorXd V(v.d0);

  for(uint i = 0; i < V.rows(); ++i)
  {
    V(i) = v(i);
  }

  return V;
}

arr createK(const MatrixXd& KA)
{
  arr K(2 * KA.rows(), KA.cols());

  for(uint i = 0; i < KA.rows(); ++i)
  {
    for(uint j = 0; j < KA.cols(); ++j)
    {
      K(i, j) = KA(i,j);
      K(i + KA.rows(), j) = -KA(i,j);
    }
  }

  return K;
}

arr createU(const VectorXd& Up, const VectorXd& Lo)
{
  arr u(2 * Up.rows());

  for(uint i = 0; i < Up.rows(); ++i)
  {
    u(i) = Up[i];
    u(i + Up.rows()) = -Lo[i];
  }

  return u;
}

IntA to_local(const IntA& global_indices, const IntA& global_to_branch)
{
  IntA local_indices(global_indices.size());

  for(uint j = 0; j < global_indices.size(); ++j)
  {
    local_indices[j] = global_to_branch[global_indices[j]];
  }

  return local_indices;
}

std::vector<arr> get_compressed_masks(int n_steps, int dim, const std::vector<IntA>& varss, IntA& var, IntA& global_to_branch)
{
  CHECK(!varss.empty() && var.empty(), "Preconditions not met");

  var.resize(varss.front().size());

  for(uint i = 0; i < var.size(); ++i)
  {
    var[i] = i;
  }

  std::vector<arr> masks(varss.size(), zeros(n_steps * dim));
  global_to_branch = IntA(n_steps);

  for(uint i = 0; i < varss.size(); ++i)
  {
    const auto& vars = varss[i];
    auto& mask = masks[i];

    for(uint j = 0; j < vars.size(); ++j)
    {
      for(uint k = 0; k < dim; ++k)
      {
        mask(dim * vars[j] + k) = 1.0;
        global_to_branch[vars[j]] = j;
      }
    }
  }

  return masks;
}

std::vector<Arr> get_compressed_scales(const std::vector<Arr>& joint_scaless)
{
  std::vector<Arr> scaless(joint_scaless.size());

  for(uint i = 0; i < joint_scaless.size(); ++i)
  {
    scaless[i] = Arr(joint_scaless[i].size(), joint_scaless[i].back());
  }

  return scaless;
}

Arr get_one_scale(const std::vector<Arr>& joint_scaless)
{
    return Arr(joint_scaless.front().size(), 1.0);
}

arr get_belief_state(const std::vector<Arr>& joint_scaless)
{
    arr bs = zeros(joint_scaless.size());

    for(uint i = 0; i < joint_scaless.size(); ++i)
    {
      bs(i) = joint_scaless[i].back();
    }

    return bs;
}

std::unordered_map<int, Constraints> get_compressed_constraints(const Constraints & k, const IntA& var, const IntA& global_to_branch)
{
  std::unordered_map<int, Constraints> constraints;
  constraints.reserve(k.xmaxs.size());

  for(uint i = 0; i < k.xmaxs.size(); ++i)
  {
    const auto& c = k.xmaxs[i];
    const auto& global_indices = std::get<3>(c);

    int branch = std::get<0>(c);

    // global to local
    const auto local_indices = to_local(global_indices, global_to_branch);

    auto cxmax = std::make_tuple(
          std::get<0>(c),
          std::get<1>(c),
          std::get<2>(c),
          local_indices);

    auto kit = constraints.find(branch);
    if(kit!=constraints.end())
    {
      kit->second.xmaxs.push_back(cxmax);
    }
    else
    {
      Constraints constraint(var.size(), std::vector<IntA>({var}));
      constraint.xmaxs = {cxmax};
      constraints.insert(std::make_pair(branch, constraint));
    }
  }

  return constraints;
}
