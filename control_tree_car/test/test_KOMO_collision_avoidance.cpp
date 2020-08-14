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

TEST_F(KomoLinearTest, DISABLED_scenario_1)
{
  auto pb = create_scenario_1(0.5);

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoJointTest, DISABLED_scenario_1)
{
  auto pb = create_scenario_1(0.5);

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoDecTest1Obstacle, scenario_1)
{
  auto pb = create_scenario_1(0.2);

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoDecTest1Obstacle, scenario_1_bis)
{
  auto pb = create_scenario_1(0.8);

  plan(pb, false);
  //plan(pb, false);
}

TEST_F(KomoDecTest2Obstacle, scenario_2)
{
  auto pb = create_scenario_2();

  plan(pb, false);
  //plan(pb, false);
}

TEST_F(KomoDecTest3Obstacle, scenario_3)
{
  auto pb = create_scenario_3();

  plan(pb, false);
  //plan(pb, false);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

