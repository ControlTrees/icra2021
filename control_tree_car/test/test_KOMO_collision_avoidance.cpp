#include <chrono>

#include "common/komo_problems.h"

#include <gtest/gtest.h>

TEST(ProbabilityFusion, OneObstacle2Branches)
{
  std::vector<Obstacle> obstacles;
  obstacles.push_back(Obstacle{arr(), 0.1});

  std::vector<std::vector<bool>> activities;
  auto ps = fuse_probabilities(obstacles, activities);

  EXPECT_EQ(std::vector<double>({0.1, 0.9}), ps);
}

TEST(ProbabilityFusion, TwoObstacle4Branches)
{
  std::vector<Obstacle> obstacles;
  obstacles.push_back(Obstacle{arr(), 0.1});
  obstacles.push_back(Obstacle{arr(), 0.5});

  std::vector<std::vector<bool>> activities;
  auto ps = fuse_probabilities(obstacles, activities);

  EXPECT_EQ(std::vector<double>({0.05, 0.5*(1.0 - 0.1), 0.05, 1.0 - 0.05 * 2 - 0.5*(1.0 - 0.1)}), ps);
}

TEST_F(KomoLinearTest, DISABLED_scenario_1) // obstacle front (30m) left
{
  auto pb = create_scenario_1();

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoJointTest, DISABLED_scenario_1) // obstacle front (30m) left
{
  auto pb = create_scenario_1();

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoDecTest, scenario_1)
{
  auto pb = create_scenario_1();

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoDecTest, DISABLED_scenario_2)
{
  auto pb = create_scenario_2();

  plan(pb, true);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

