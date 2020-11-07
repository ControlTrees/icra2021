#include <qp/QP_tree_problem_OSQP.h>

#include <gtest/gtest.h>

using namespace std;

TEST(QP_tree_problem, test_matrix_sparsification)
{
    MatrixXd M(3, 3);
    M << 1, 0, 2,
         0, 0, 3,
         4, 5, 6;

    auto MS = create_csc_matrix(M);

    EXPECT_EQ(MS->nzmax, 6);

    c_float expected_x[6] = {1, 4, 5, 2, 3, 6};
    for(auto I = 0; I < 6; ++I)
        EXPECT_EQ(MS->x[I],expected_x[I]);

    c_float expected_i[6] = {0, 2, 2, 0, 1, 2};
    for(auto I = 0; I < 6; ++I)
        EXPECT_EQ(MS->i[I],expected_i[I]);

    c_float expected_p[4] = {0, 2, 3, 6};
    for(auto I = 0; I < 4; ++I)
        EXPECT_EQ(MS->p[I],expected_p[I]);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

