#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <OptimDec/decentralized_optimizer.h>

struct QP_Problem
{
  const arr P;
  const arr q;
  const arr K;
  const arr u;

  QP_Problem(const arr& P, const arr& q, const arr& K, const arr& u);

  double operator()(arr& dL, arr& HL, const arr& x) const;

private:
  arr qT;
};

struct QP_Lagrangian : ScalarFunction
{
  QP_Lagrangian(const QP_Problem &P, OptOptions opt=NOOPT, arr& lambdaInit=NoArr);

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< CORE METHOD: the unconstrained scalar function F

  const QP_Problem& qp;

  //-- parameters of the unconstrained (Lagrangian) scalar function
  double mu;         ///< penalty parameter for inequalities g
  double nu{0};      ///< NIY! penalty parameter for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g and equalities h

  double get_costs();            ///< info on the terms from last call
  double get_sumOfGviolations(); ///< info on the terms from last call
  double get_sumOfHviolations(); ///< info on the terms from last call
  uint get_dimOfType(const ObjectiveType& tt); ///< info on the terms from last call
  void aulaUpdate(bool anyTimeVariant, double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);

private:
  arr x;            ///< last evaluated x
  double c{0.0};    ///< last evaluated cost
  arr g;            ///< Kx - u
  arr g_violations; ///< Kx - u if > 0 else 0
};

template<>
struct LagrangianTypes<QP_Problem>
{
  typedef QP_Lagrangian Lagrangian;
  typedef DecLagrangianProblem<Lagrangian> DecLagrangian;
};
