#include <qp/stopline_qp_tree.h>

#include <gtest/gtest.h>

using namespace std;

TEST(Probability_fusion, one_to_one)
{
    std::vector<Stopline> stoplines{{0, 0.5}};

    const auto probabilities = fuse_probabilities(stoplines, 1);

    EXPECT_EQ(probabilities[0], 0.5);
}

TEST(Probability_fusion, two_to_two)
{
    std::vector<Stopline> stoplines{{0, 0.5}, {1, 0.5}};

    const auto probabilities = fuse_probabilities(stoplines, 2);

    EXPECT_EQ(probabilities[0], 0.5);
    EXPECT_EQ(probabilities[1], 0.25);
}


TEST(Probability_fusion, two_to_one)
{
    std::vector<Stopline> stoplines{{0, 0.5}, {1, 0.5}};

    const auto probabilities = fuse_probabilities(stoplines, 1);

    EXPECT_EQ(probabilities[0], 0.75);
}

TEST(Probability_fusion, two_to_one_edge_case_1)
{
    std::vector<Stopline> stoplines{{0, 0.5}, {1, 1.0}};

    const auto probabilities = fuse_probabilities(stoplines, 1);

    EXPECT_EQ(probabilities[0], 1.0);
}

TEST(Probability_fusion, two_to_one_edge_case_2)
{
    std::vector<Stopline> stoplines{{0, 0.5}, {1, 0.0}};

    const auto probabilities = fuse_probabilities(stoplines, 1);

    EXPECT_EQ(probabilities[0], 0.5);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

