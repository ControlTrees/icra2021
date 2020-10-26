#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <gtest/gtest.h>

#include "support/functions.cpp"

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

TEST(OptConstrained, SimpleParabolTestG) {
  arr x{1.0};
  arr dual; //dual

  Parabol pb;

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(-0.5, x(0), eps_s);
}

TEST(OptConstrained, Distance2DTestHG) {
  arr x{1.0, 2.0};
  arr dual; //dual

  Distance2D pb({10.0, 2.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
}

TEST(OptConstrained, MultipleRuns) {
  arr x{1.0, 2.0};
  arr dual;

  Distance2D pb({10.0, 2.0});

  OptConstrained opt(x, dual, pb);
  std::cout << "phase 1" << std::endl;
  opt.run();
  std::cout << "phase 2" << std::endl;
  x -= 0.3;
  OptConstrained opt2(x, dual, pb);
  opt2.run();
  std::cout << "phase 3" << std::endl;
  OptConstrained opt3(x, dual, pb);
  opt3.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
}


TEST(OptConstrained, Distance3DTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3D pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(OptConstrained, Distance3DDecompXYTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3D pb(arr{1.0, 1.0, 1.0}, arr{1.0, 1.0, 0.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(OptConstrained, Distance3DDecompXZTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3D pb(arr{1.0, 1.0, 1.0}, arr{1.0, 0.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(0.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

// WARM START
TEST(OptConstrained, Valley2DWarmStart) {
  arr x{0.0, 0.0};
  arr dual;

  Valley2D pb;

  uint its1 = 0;
  OptOptions options=NOOPT;
  options.aulaMuInc = 1.0;
  {
  OptConstrained opt(x, dual, pb, -1, options);
  opt.run();
  its1 = opt.its;
  }

  uint its2 = 0;
  {
  x(0)+=1; // shift in the direction of the valley
  OptConstrained opt(x, dual, pb, -1, options);
  opt.run();
  its2 = opt.its;
  }

  EXPECT_NEAR(1.0, x(0), eps_s);
  EXPECT_NEAR(-1.0, x(1), eps_s);
  EXPECT_TRUE(its1 > its2);
}

TEST(OptConstrained, Valley2DSideWaysWarmStart) {
  arr x{0.0, 0.0};
  arr dual;

  Valley2DSideWays pb;

  uint its1 = 0;
  OptOptions options=NOOPT;
  options.aulaMuInc = 1.0;
  {
  pb.xstart = x(0);
  OptConstrained opt(x, dual, pb, -1, options);
  opt.run();
  its1 = opt.its;
  }

  uint its2 = 0;
  {
  x(0)+=1; // shift in the direction of the valley
  pb.xstart = x(0);
  OptConstrained opt(x, dual, pb, -1, options);
  opt.run();
  its2 = opt.its;
  }

//  EXPECT_NEAR(1.0, x(0), eps_s);
//  EXPECT_NEAR(-1.0, x(1), eps_s);
  EXPECT_TRUE(its1 > its2);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
