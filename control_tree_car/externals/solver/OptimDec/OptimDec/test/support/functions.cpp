#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <OptimDec/decentralized_lagrangian.h>

struct Parabol : public ConstrainedProblem
{
  // min x^2
  // s.t. x < -0.5
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = arr(2, 1);
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_ineq;
    }

    phi(0) = x(0);
    J(0, 0) = 1.0;

    phi(1) = x(0) + 0.5;
    J(1, 0) = 1.0;
  }
};


struct Distance2D : public ConstrainedProblem
{
  // min dist to point (10, 2)
  // s.t. x = 0.0
  // s.t  y < 1
  Distance2D(const arr& center, arr mask = ones(2))
    : center_(center)
    , mask_(mask)
  {
    CHECK_EQ(center.d0, 2, "wrong vector dimension")
    CHECK_EQ(mask.d0, 2, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(4);
      J = zeros(4, 2);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(4);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_eq;
      ot(3) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = mask_(0) * (x(0) - center_(0));
    Js->elem(0, 0) = mask_(0);

    phi(1) = mask_(1) * (x(1) - center_(1));
    Js->elem(1, 1) = mask_(1);

    phi(2) = x(0);
    Js->elem(2, 0) = 1.0;

    phi(3) = x(1) - 1.0;
    Js->elem(3, 1) = 1.0;
  }

  arr center_;
  arr mask_;
};

struct ParabolWithFTerm : public ConstrainedProblem
{
  // min x^2 -x
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = arr(2, 1);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_f;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = x(0);
    Js->elem(0, 0) = 1.0;

    phi(1) = -x(0);
    Js->elem(1, 0) = -1.0;
  }
};

struct Distance3D : public ConstrainedProblem
{
  // min dist to center
  // s.t. x = 0.0
  Distance3D(const arr& center, arr mask = ones(3))
    : center_(center)
    , mask_(mask)
  {
    CHECK_EQ(center.d0, 3, "wrong vector dimension")
    CHECK_EQ(mask.d0, 3, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(4);
      J = zeros(4, 3);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(4);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_sos;
      ot(3) = OT_eq;
      //ot(4) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = mask_(0) * (x(0) - center_(0));
    Js->elem(0, 0) = mask_(0);

    phi(1) = mask_(1) * (x(1) - center_(1));
    Js->elem(1, 1) = mask_(1);

    phi(2) = mask_(2) * (x(2) - center_(2));
    Js->elem(2, 2) = mask_(2);

    phi(3) = x(0);
    Js->elem(3, 0) = 1.0;

//    phi(4) = x(1) - 1.0;
//    J(4, 1) = 1.0;
  }

  arr center_;
  arr mask_;
};

struct Distance4D : public ConstrainedProblem
{
  // min dist to center
  // s.t. x = 0.0
  // s.t  y < 1
  Distance4D(const arr& center, arr mask = ones(4))
    : center_(center)
    , mask_(mask)
  {
    CHECK_EQ(center.d0, 4, "wrong vector dimension")
    CHECK_EQ(mask.d0, 4, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(5);
      J = zeros(5, 4);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(5);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_sos;
      ot(3) = OT_sos;
      ot(4) = OT_eq;
      //ot(4) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = mask_(0) * (x(0) - center_(0));
    Js->elem(0, 0) = mask_(0);

    phi(1) = mask_(1) * (x(1) - center_(1));
    Js->elem(1, 1) = mask_(1);

    phi(2) = mask_(2) * (x(2) - center_(2));
    Js->elem(2, 2) = mask_(2);

    phi(3) = mask_(3) * (x(3) - center_(3));
    Js->elem(3, 3) = mask_(3);

    phi(4) = x(0);
    Js->elem(4, 0) = 1.0;

//    phi(4) = x(1) - 1.0;
//    J(4, 1) = 1.0;
  }

  arr center_;
  arr mask_;
};

struct Valley2D : public ConstrainedProblem
{
  // min y^2
  // s.t  y < -1
  Valley2D()
  {
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = zeros(2, 2);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = x(1);
    Js->elem(0, 1) = 1.0;

    phi(1) = x(1) + 1.0;
    Js->elem(1, 1) = 1.0;
  }
};

struct Valley2DSideWays : public ConstrainedProblem
{
  // min (y - slpha * x)^2
  // x = x_start
  // s.t  y < -1 - alpha * x
  Valley2DSideWays()
  {

  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(3);
      J = zeros(3, 2);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(3);

      ot(0) = OT_sos;
      ot(1) = OT_eq;
      ot(2) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = x(1) - alpha * x(0);
    Js->elem(0, 0) = -alpha;
    Js->elem(0, 1) = 1.0;

    phi(1) = x(0) - xstart;
    Js->elem(1, 0) = 1.0;

    phi(2) = x(1) + 1.0 + alpha * x(0);
    Js->elem(2, 0) = alpha;
    Js->elem(2, 1) = 1.0;
  }

  double xstart = 0;
  double alpha = 0.1;
};

struct Valley2DSideWaysDecomposed0 : public ConstrainedProblem
{
  // min (y - slpha * x)^2
  // x = x_start
  Valley2DSideWaysDecomposed0()
  {

  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = zeros(2, 2);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_eq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = x(1) - alpha * x(0);
    Js->elem(0, 0) = -alpha;
    Js->elem(0, 1) = 1.0;

    phi(1) = x(0) - xstart;
    Js->elem(1, 0) = 1.0;
  }

  double alpha = 0.1;
  double xstart = 0;
};

struct Valley2DSideWaysDecomposed1 : public ConstrainedProblem
{
  // s.t  y < -1 - alpha * x
  Valley2DSideWaysDecomposed1()
  {

  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(1);
      J = zeros(1, 2);
      J.special = &J.sparse();
      ot = ObjectiveTypeA(1);

      ot(0) = OT_ineq;
    }

    const auto Js = dynamic_cast<rai::SparseMatrix*>(J.special);

    phi(0) = x(1) + 1.0 + alpha * x(0);
    Js->elem(0, 0) = alpha;
    Js->elem(0, 1) = 1.0;
  }

  double alpha = 0.1;
};
