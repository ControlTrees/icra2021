#include <chrono>

#include "common/komo_problems.h"

#include <gtest/gtest.h>

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


////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

