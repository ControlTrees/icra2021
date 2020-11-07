#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <OptimDec/decentralized_optimizer.h>
#include <OptimDec/qp_lagrangian.h>
#include <gtest/gtest.h>

#include "support/qp_problems.cpp"

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

namespace
{
  DecOptConfig buildUncompressedOptions()
  {
    DecOptConfig options(PARALLEL, false, NOOPT, true);
    options.opt.stopTolerance = 0.001;
    return options;
  }

  DecOptConfig buildCompressedOptions()
  {
    DecOptConfig options(PARALLEL, true, NOOPT, true);
    options.opt.stopTolerance = 0.001;
    return options;
  }
}

TEST(QP, Unconstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;

  auto qp = std::make_shared<QP_Problem>(P, q, NoArr, NoArr);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildUncompressedOptions();
  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, {}, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.5, x(0), eps_s);
}

TEST(QP, OneDimOneConstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;
  arr K(1, 1); K(0, 0) = 1;
  arr u(1); u(0) = 0.2;

  auto qp = std::make_shared<QP_Problem>(P, q, K, u);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildUncompressedOptions();

  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, {}, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
}

TEST(QP, OneDimTwoConstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;
  arr K(2, 1); K(0, 0) = 1; K(1, 0) = -1;
  arr u(2); u(0) = 0.2; u(1) = 0.0;

  auto qp = std::make_shared<QP_Problem>(P, q, K, u);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildUncompressedOptions();

  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, {}, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
}

// 2 dims - 1 prob
TEST(QP, TwoDimTwoConstrained) {
  arr x = arr(2);
  x(0) = 0;
  x(1) = 1;

  // unconstrained min at (0.5, 0.5), constraints: x[0] < 0.2, x[1] > 0.8

  auto qp = createConstrained2d(0.5);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildUncompressedOptions();

  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, {}, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
  EXPECT_NEAR(0.8, x(1), eps_s);
}

// 2 dims - 2 probs
TEST(QP, TwoProblemsBattlingOverX) { // cost battling
  arr x = arr(3);
  x(0) = 0;
  x(1) = 0;
  x(2) = 0;

  // unconstrained min at (0.5, 0.5), constraints: x[0] < 0.2, x[1] > 0.8
  auto qp1 = createUnconstrained2d(0.4);
  auto qp2 = createUnconstrained2d(0.6);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp1);
  pbs.push_back(qp2);

  std::vector<arr> masks;
  masks.push_back(arr{1.0, 1.0, 0.0});
  masks.push_back(arr{1.0, 0.0, 1.0});

  auto options = buildCompressedOptions();

  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, masks, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.5, x(0), eps_s);
  EXPECT_NEAR(0.4, x(1), eps_s);
  EXPECT_NEAR(0.6, x(2), eps_s);
}

TEST(QP, TwoProblemsBattlingOverXWithConstraints) {
  arr x = arr(3);
  x(0) = 0;
  x(1) = 0;
  x(2) = 0;

  // unconstrained min at (0.5, 0.5), constraints: x[0] < 0.2, x[1] > 0.8
  auto qp1 = createConstrained2d(0.5);
  auto qp2 = createUnconstrained2d(0.5);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp1);
  pbs.push_back(qp2);

  std::vector<arr> masks;
  masks.push_back(arr{1.0, 1.0, 0.0});
  masks.push_back(arr{1.0, 0.0, 1.0});

  auto options = buildCompressedOptions();

  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, masks, AverageUpdater(), options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), 2 * eps_s);
  EXPECT_NEAR(0.8, x(1), eps_s);
  EXPECT_NEAR(0.5, x(2), eps_s);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
