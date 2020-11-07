#include <OptimDec/qp_lagrangian.h>

std::shared_ptr<QP_Problem> createUnconstrained2d(double center)
{
  // unconstrained min at (center, center), constraints: x[0] < 0.2, x[1] > 0.8

  // min(x^2+qx) => 2x + q = 0 => x = -q/2 => q = -2 center

  arr P(2,2);
  P(0,0) = 2;
  P(1,1) = 2;

  arr q(2);
  q(0) = -2 * center;
  q(1) = -2 * center;

  return std::make_shared<QP_Problem>(P, q, arr(), arr());;
}

std::shared_ptr<QP_Problem> createConstrained2d(double center)
{
  // unconstrained min at (center, center), constraints: x[0] < 0.2, x[1] > 0.8

  // min(x^2+qx) => 2x + q = 0 => x = -q/2 => q = -2 center

  arr P(2,2);
  P(0,0) = 2;
  P(1,1) = 2;

  arr q(2);
  q(0) = -2 * center;
  q(1) = -2 * center;

  arr K(2, 2);
  K(0, 0) = 1;
  K(1, 1) = -1;

  arr u(2);
  u(0) = 0.2;
  u(1) = -0.8;

  return std::make_shared<QP_Problem>(P, q, K, u);
}
