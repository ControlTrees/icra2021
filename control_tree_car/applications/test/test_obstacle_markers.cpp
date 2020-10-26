#include <control_tree/ros/obstacle_common.h>

#include <gtest/gtest.h>

TEST(ObstacleBoundingBox, RoadLeftSide)
{
    const double y = 1.5;
    auto marker = create_collision_marker(0, y, 2, 1, 1, 0.5, 0);

    EXPECT_GE(marker.pose.position.y, y);
}


////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

