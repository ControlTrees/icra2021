#include <OptimDec/utils.h>
#include <gtest/gtest.h>

TEST(AddMuOnHessian, NonSparse) {
  arr H = zeros(3, 3);

  intA var(2, 1, {1, 2});
  arr mask(3, {0.0, 1.0, 1.0});

  add(H, 1.0, var, arr());

  EXPECT_EQ(H(0, 0), 0.0);
  EXPECT_EQ(H(1, 1), 1.0);
  EXPECT_EQ(H(2, 2), 1.0);
}

TEST(AddMuOnHessian, SparseIfDiagElementsAlreadyExist) {
  arr H = zeros(3, 3);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  H.special = &H.sparse();

  intA var(2, 1, {1, 2});
  arr mask(3, {0.0, 1.0, 1.0});

  add(H, 1.0, var, mask);

  const auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

  EXPECT_EQ(Hs->elem(0, 0), 1.0);
  EXPECT_EQ(Hs->elem(1, 1), 2.0);
  EXPECT_EQ(Hs->elem(2, 2), 2.0);
}

TEST(AddMuOnHessian, SparseIfDiagNonExistingElementOnHessian) {
  arr H = zeros(3, 3);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  //H(2, 2) = 1.0;
  H.special = &H.sparse();

  intA var(2, 1, {1, 2});
  arr mask(3, {0.0, 1.0, 1.0});

  add(H, 1.0, var, mask);

  const auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

  EXPECT_EQ(Hs->elem(0, 0), 1.0);
  EXPECT_EQ(Hs->elem(1, 1), 2.0);
  EXPECT_EQ(Hs->elem(2, 2), 1.0);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
