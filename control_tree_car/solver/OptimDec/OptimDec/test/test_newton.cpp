#include <Optim/newton.h>
#include <gtest/gtest.h>

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

double simple_parabol(arr& g, arr& H, const arr& x)
{
  double value = 0;

  CHECK_EQ(x.d0, 1, "expect scalar array");

  value = x(0) * x(0);

  if(!g.p)
    g = arr(1);

  g(0) = 2 * x(0);

  if(!H.p)
    H = arr(1, 1);

  H(0, 0) = 2.0;

  return value;
}

TEST(NewtonOptimizer, SimpleParabolTestGH) {
  arr x0{1.0};
  arr g;
  arr H;

  ScalarFunction f{simple_parabol};

  double value = f(g, H, x0);

  EXPECT_NEAR(1.0, value, eps);
  EXPECT_NEAR(2.0, g(0), eps);
  EXPECT_NEAR(2.0, H(0, 0), eps);
}

TEST(NewtonOptimizer, SimpleParabolNewton) {
  arr x{1.0};
  arr g;
  arr H;

  ScalarFunction f{simple_parabol};
  OptNewton opt(x, f);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
}

TEST(NewtonOptimizer, SimpleParabolNewtonSeveralRuns) {
  arr x{1.0};
  arr g;
  arr H;

  ScalarFunction f{simple_parabol};
  OptNewton opt(x, f, NOOPT, &std::cout);
  std::cout << "phase 1" << std::endl;
  opt.run();
  std::cout << "phase 2" << std::endl;
  opt.run();
  std::cout << "phase 3" << std::endl;
  opt.run();
  std::cout << "phase 4" << std::endl;
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
}


//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
