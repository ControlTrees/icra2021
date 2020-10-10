#include <control_tree/core/utility.h>
#include <control_tree/ros/common.h>

#include <gtest/gtest.h>

using namespace std;

static constexpr uint N = 10000;

static uint n_above_threshold(double median_p)
{
    double sum = 0;
    uint n = 0;

    for(auto i = 0; i < N; ++i)
    {
        double p = draw_p(median_p);

        if(p > median_p)
        {
            ++n;
        }
    }

    return n;
}

static uint n_true(double average_p)
{
    uint n = 0;

    for(auto i = 0; i < N; ++i)
    {
        if(draw_bool(average_p))
        {
            ++n;
        }
    }

    return n;
}

TEST(DrawProbability, probability_drawing_0p5)
{
    auto n = n_above_threshold(0.5);

    EXPECT_GE(n, 0.4 * N);
    EXPECT_LE(n, 0.6 * N);
}

TEST(DrawProbability, probability_drawing_0p15)
{
    auto n = n_above_threshold(0.15);

    EXPECT_GE(n, 0.4 * N);
    EXPECT_LE(n, 0.6 * N);
}

TEST(DrawProbability, probability_drawing_0p05)
{
    auto n = n_above_threshold(0.05);

    EXPECT_GE(n, 0.4 * N);
    EXPECT_LE(n, 0.6 * N);
}

TEST(DrawProbability, probability_drawing_0p85)
{
    auto n = n_above_threshold(0.85);

    EXPECT_GE(n, 0.4 * N);
    EXPECT_LE(n, 0.6 * N);
}

TEST(DrawBinary, probability_drawing_0p15)
{
    auto n = n_true(0.15);

    EXPECT_GE(n, 0.05 * N);
    EXPECT_LE(n, 0.25 * N);
}

TEST(DrawBinary, probability_drawing_0p85)
{
    auto n = n_true(0.85);

    EXPECT_GE(n, 0.75 * N);
    EXPECT_LE(n, 0.95 * N);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

