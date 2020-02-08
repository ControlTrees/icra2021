#pragma once

#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using intA = std::vector<int>;
using arr = std::vector<double>;

struct MPC_model
{
    MPC_model(double dt, double Q_v_weight, double R_u_weight);

    VectorXd predict_trajectory(const Vector2d & x0, const VectorXd & U) const;
    VectorXd predict_trajectory(const Vector2d & x0, const VectorXd & U, const std::vector<intA> & varss) const;

    double cost(const Vector2d & x0, const VectorXd & U, const Vector2d & xd) const; // cost of full control sequence
    double cost(const Vector2d & x, double u, const Vector2d & xd) const; // cost of single step

    VectorXd check_constraints(const Vector2d & x0, const VectorXd & U, const Vector2d & xmax) const;

    VectorXd get_Xd(const Vector2d & xd, int n_steps) const;

    MatrixXd get_Q_bar(int n_steps) const;
    MatrixXd get_Q_bar(int n_steps, const std::vector<intA> & varss, const std::vector<arr> & scaless) const;

    MatrixXd get_R_bar(int n_steps) const;
    MatrixXd get_R_bar(int n_steps, const std::vector<intA> & varss, const std::vector<arr> & scaless) const;

    MatrixXd get_S(int n_steps) const;
    MatrixXd get_S(int n_steps, const std::vector<intA> & varss) const;

    MatrixXd get_T(int n_steps) const;
    MatrixXd get_T(int n_steps, const std::vector<intA> & varss) const;

    Matrix2d A; // system model
    Vector2d B; // system model

    Matrix2d Q; // state costs
    Matrix< double, 1, 1> R; // control costs
};
