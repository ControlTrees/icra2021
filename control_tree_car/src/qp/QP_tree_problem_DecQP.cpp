#include <control_tree/qp/QP_tree_problem_DecQP.h>
#include <Optimization/qp_lagrangian.h>

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
}

QP_tree_problem_DecQP::QP_tree_problem_DecQP(const MPC_model & mpc, double u_min, double u_max)
    : QP_tree_problem_base(mpc, u_min, u_max)
{
}

VectorXd QP_tree_problem_DecQP::call_solver()
{
    const auto P = convert(H);
    const auto q = convert(C);
    auto K = createK(KA);
    auto u = createU(Up, Lo);

    auto qp = std::make_shared<QP_Problem>(P, q, K, u);

    std::vector<std::shared_ptr<QP_Problem>> pbs;
    pbs.push_back(qp);

    arr x = zeros(P.d0);

    DecOptConfig options(PARALLEL, false, NOOPT, false);
    DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

    opt.run();

    return convert(x);
}
