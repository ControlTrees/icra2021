#include <control_tree/qp/QP_tree_problem_DecQP.h>

namespace{

arr convert(const MatrixXd& M)
{
  arr m(M.rows(), M.cols());

  for(auto i = 0; i < M.rows(); ++i)
  {
    for(auto j = 0; j < M.cols(); ++j)
    {
      m(i, j) = M(i,j);
    }
  }

  return m;
}

VectorXd convert(const arr& v)
{
  VectorXd V(v.d0);

  for(auto i = 0; i < V.rows(); ++i)
  {
    V(i) = v(i);
  }

  return V;
}

arr createK(const MatrixXd& KA)
{
  arr K(2 * KA.rows(), KA.cols());

  for(auto i = 0; i < KA.rows(); ++i)
  {
    for(auto j = 0; j < KA.cols(); ++j)
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

  for(auto i = 0; i < Up.rows(); ++i)
  {
    u(i) = Up[i];
    u(i + Up.rows()) = -Lo[i];
  }

  return u;
}

IntA to_local(const IntA& global_indices, const IntA& global_to_branch)
{
  IntA local_indices(global_indices.size());

  for(auto j = 0; j < global_indices.size(); ++j)
  {
    local_indices[j] = global_to_branch[global_indices[j]];
  }

  return local_indices;
}
}

//----Joint------------------------//

QP_tree_problem_JointQP::QP_tree_problem_JointQP(const MPC_model & mpc, double u_min, double u_max)
    : QP_tree_joint_solver_base(mpc, u_min, u_max)
    , options(PARALLEL, true, NOOPT, false)
{
  options.opt.verbose = 0;
}

VectorXd QP_tree_problem_JointQP::call_solver()
{
    const auto P = convert(H);
    const auto q = convert(C);
    const auto K = createK(KA);
    const auto u = createU(Up, Lo);

    auto qp = std::make_shared<QP_Problem>(P, q, K, u);

    std::vector<std::shared_ptr<QP_Problem>> pbs;
    pbs.push_back(qp);

    arr x = zeros(P.d0);

    DecOptConfig options(PARALLEL, false, NOOPT, false);
    DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

    opt.run();

    return convert(x);
}

//----Dec---------------------------//

QP_tree_problem_DecQP::QP_tree_problem_DecQP(const MPC_model & mpc, double u_min, double u_max)
  : QP_tree_solver_base(mpc, u_min, u_max)
  , options(PARALLEL, true, NOOPT, false)
{
  options.opt.verbose = 0;
}

VectorXd QP_tree_problem_DecQP::solve(const Vector2d & x0, const Vector2d & xd, const Constraints& joint_k,
                                      int n_steps,
                                      const std::vector<IntA> & joint_varss,
                                      const std::vector<Arr> & joint_scaless)
{
  // generate compressed var and masks
  IntA var, global_to_branch;
  const auto masks = get_compressed_masks(n_steps, mpc.get_dim(), joint_varss, var, global_to_branch);
  const auto scaless = get_compressed_scales(joint_scaless);

  // compress constraints
  const auto ks = get_compressed_constraints(joint_k, var, global_to_branch);

  // build subproblems
  const auto branch_n_steps = var.size();
  std::vector<IntA> branch_varss({var});
  std::vector<std::shared_ptr<QP_Problem>> pbs; pbs.reserve(masks.size());

  std::vector<std::future<std::shared_ptr<QP_Problem>>> futures;
  futures.reserve(masks.size());
  for(auto i = 0; i < masks.size(); ++i)
  {
    futures.push_back(
          std::async(std::launch::async,
                     [&, i]()
                      {
                        return build_qp(i, branch_n_steps, branch_varss, {scaless[i]}, ks, x0, xd);
                      }
                    )
    );
  }

  for(auto& future: futures)
  {
    pbs.emplace_back(future.get());
  }

  // solve
  arr x = zeros(n_steps * mpc.get_dim());

  DecOptConstrained<QP_Problem> opt(x, pbs, masks, options);

  opt.run();

  return convert(x);
}

std::shared_ptr<QP_Problem> QP_tree_problem_DecQP::build_qp(int i,
                                                            int n_steps,
                                                            const std::vector<IntA>& varss,
                                                            const std::vector<Arr>& scaless,
                                                            const std::unordered_map<int, Constraints> & constraints,
                                                            const Vector2d & x0,
                                                            const Vector2d & xd) const
{
  // build MPC matrices
  // costs (could be computed just once and scaled?)
  const auto S = mpc.get_S(n_steps, varss);
  const auto T = mpc.get_T(n_steps, varss);

  const auto Q_bar = mpc.get_Q_bar(n_steps, varss, scaless);
  const auto R_bar = mpc.get_R_bar(n_steps, varss, scaless);

  const auto H = 2 * (R_bar + S.transpose() * Q_bar * S);
  const auto F = 2 * (T.transpose() * Q_bar * S);
  const auto G = 2 * Q_bar * S;

  const Eigen::VectorXd & Xd = mpc.get_Xd(xd, n_steps);
  const auto C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();

  // constraints
  Eigen::MatrixXd KA;
  Eigen::VectorXd Up, Lo;

  const auto kit = constraints.find(i);
  if(kit!=constraints.end())
  {
    const auto& k = kit->second;
    const Eigen::VectorXd Xmax = k.getXmax();
    const Eigen::MatrixXd Sextract = k.getSextract();

    const auto nk = Sextract.rows() + H.rows();
    KA.resize(nk, H.rows());
    Up.resize(nk);
    Lo.resize(nk);

    // traj constraints
    KA.block(0, 0, Sextract.rows(), H.rows()) = Sextract * S;
    Up.head(Sextract.rows()) = Sextract * (Xmax - T * x0);
    Lo.head(Up.rows()) = VectorXd::Constant(Up.rows(), std::numeric_limits<double>::lowest());

    // control bounds constraints
    KA.block(Sextract.rows(), 0, H.rows(), H.rows()) = MatrixXd::Identity(H.rows(), H.rows());
    Up.tail(H.rows()) = VectorXd::Constant(H.rows(), u_max_);
    Lo.tail(H.rows()) = VectorXd::Constant(H.rows(), u_min_);
  }
  else // no traj constraints, add control bounds only
  {
    // control bounds constraints
    KA = MatrixXd::Identity(H.rows(), H.rows());
    Up = VectorXd::Constant(H.rows(), u_max_);
    Lo = VectorXd::Constant(H.rows(), u_min_);
  }

  // build QP matrices
  const auto P = convert(H);
  const auto q = convert(C);
  const auto K = createK(KA);
  const auto u = createU(Up, Lo);

  return std::make_shared<QP_Problem>(P, q, K, u);
}

//---free compression functions--------------------//

std::vector<arr> get_compressed_masks(int n_steps, int dim, const std::vector<IntA>& varss, IntA& var, IntA& global_to_branch)
{
  CHECK(!varss.empty() && var.empty(), "Preconditions not met");

  var.resize(varss.front().size());

  for(auto i = 0; i < var.size(); ++i)
  {
    var[i] = i;
  }

  std::vector<arr> masks(varss.size(), zeros(n_steps * dim));
  global_to_branch = IntA(n_steps);

  for(auto i = 0; i < varss.size(); ++i)
  {
    const auto& vars = varss[i];
    auto& mask = masks[i];

    for(auto j = 0; j < vars.size(); ++j)
    {
      for(auto k = 0; k < dim; ++k)
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

  for(auto i = 0; i < joint_scaless.size(); ++i)
  {
    scaless[i] = Arr(joint_scaless[i].size(), joint_scaless[i].back());
  }

  return scaless;
}

std::unordered_map<int, Constraints> get_compressed_constraints(const Constraints & k, const IntA& var, const IntA& global_to_branch)
{
  std::unordered_map<int, Constraints> constraints;
  constraints.reserve(k.xmaxs.size());

  for(auto i = 0; i < k.xmaxs.size(); ++i)
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
