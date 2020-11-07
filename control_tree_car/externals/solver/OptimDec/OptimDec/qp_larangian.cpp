#include <OptimDec/qp_lagrangian.h>

namespace
{
arr extractGreaterThan0(const arr& a, const arr& mask)
{
  auto b = a;

  for(uint i = 0; i < a.d0; ++i)
  {
    if(mask(i) < 0)
    {
      if(b.nd == 1)
      {
        b(i) = 0.0;
      }
      else
      {
        for(uint j = 0; j < b.d1; ++j)
        {
          b(i,j) = 0;
        }
      }
    }
  }
  return b;
}
}

QP_Problem::QP_Problem(const arr& P, const arr& q, const arr& K, const arr& u)
  : P(P)
  , q(q)
  , K(K)
  , u(u)
{
  transpose(qT, q);
}

double QP_Problem::operator()(arr& dL, arr& HL, const arr& x) const
{
  arr xT;
  transpose(xT, x);

  auto v = 0.5 * xT * P * x + qT * x;

  if(!!dL)
  {
    dL = P * x + q;
  }

  if(!!HL)
  {
    HL = P;
  }

  return v(0);
}

QP_Lagrangian::QP_Lagrangian(const QP_Problem& P, OptOptions opt, arr& lambdaInit)
  : qp(P)
  , mu(opt.muInit)
{
  lambda = zeros(qp.K.d0);

  ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
    return this->lagrangian(dL, HL, x);
  });
}

double QP_Lagrangian::lagrangian(arr& dL, arr& HL, const arr& _x) ///< CORE METHOD: the unconstrained scalar function F
{
  if(_x!=x)
  {
    x = _x;
  }

  double L = c = qp(dL, HL, x);

  // handle inequality constraints
  if(qp.K.d0)
  {
    g = qp.K * x - qp.u; // Jg = K
    g_violations = extractGreaterThan0(g, g);
    const auto& Jg = qp.K;
    const auto Jg_violations = extractGreaterThan0(Jg, g);

    // add value
    L += scalarProduct(lambda, g);                        // lagrange term
    L += mu * scalarProduct(g_violations, g_violations);  // square penalty

    // jacobian
    if(!!dL)
    {
      dL += comp_At_x(lambda, Jg);                             // lagrange term
      dL += 2.0 * mu * comp_At_x(g_violations, Jg_violations); // square penalty
    }

    // hessian
    if(!!HL)
    {
      HL += 2.0 * mu * comp_At_A(Jg_violations);              // square penalty
    }
  }

  return L;
}

double QP_Lagrangian::get_costs()
{
  return c;
}

double QP_Lagrangian::get_sumOfGviolations()
{
  return sum(g_violations);
}

double QP_Lagrangian::get_sumOfHviolations()
{
  return 0.0;
}

uint QP_Lagrangian::get_dimOfType(const ObjectiveType& tt)
{
  if(tt == OT_ineq)
    return lambda.d0;
  if(tt == OT_eq)
    return 0;
  if(tt == OT_sos)
    return 1;
  return 0;
}

void QP_Lagrangian::aulaUpdate(bool anyTimeVariant, double _, double muInc, double *L_x, arr &dL_x, arr &HL_x)
{
  for(uint i = 0; i < lambda.d0; ++i)
  {
    lambda(i) = std::max(lambda(i) + 2.0 * mu * g(i), 0.0);
    // see "A tutorial on Newton methods for constrained trajectory optimization and relations to SLAM,
    // Gaussian Process smoothing, optimal control, and probabilistic inference" p.7

    // if g(i) negative, the value of lambda will decrease (less need to push away ro the constraint!).
    // lambda(i) can't become negative though as it would mean pushing towards the constraints
  }

  if(muInc>1. && mu<1e6) mu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || !!dL_x || !!HL_x)
  {
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}
