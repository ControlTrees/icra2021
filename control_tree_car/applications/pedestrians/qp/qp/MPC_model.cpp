#include <qp/MPC_model.h>

MPC_model::MPC_model(double dt, double Q_v_weight, double R_u_weight)
{
    // dynamic model
    A << 1 , dt,
            0 , 1;

    B << 0 , dt;

    // costs
    Q << 0, 0,
         0, Q_v_weight;

    R << R_u_weight;
}

VectorXd MPC_model::predict_trajectory(const Vector2d & x0, const VectorXd & U) const
{
    auto n_steps = U.rows();

    MatrixXd S = get_S(n_steps);
    MatrixXd T = get_T(n_steps);

    return T * x0 + S * U;
}

VectorXd MPC_model::predict_trajectory(const Vector2d & x0, const VectorXd & U, const std::vector<IntA> & varss) const
{
    auto n_steps = U.rows();

    MatrixXd S = get_S(n_steps, varss);
    MatrixXd T = get_T(n_steps, varss);

    return T * x0 + S * U;
}

double MPC_model::cost(const Vector2d & x0, const VectorXd & U, const Vector2d & xd) const
{
    auto n_steps = U.rows();

    VectorXd Xd = get_Xd(xd, n_steps);

    MatrixXd Q_bar = get_Q_bar(n_steps);
    MatrixXd R_bar = get_R_bar(n_steps);

    const auto & X = predict_trajectory(x0, U);

    const auto cost = (X - Xd).transpose() * Q_bar * (X - Xd) + U.transpose() * R_bar * U;

    return cost(0,0);
}

double MPC_model::cost(const Vector2d & x, double u, const Vector2d & xd) const
{
    const auto _u = VectorXd::Ones(1) * u;
    const auto & c = (x - xd).transpose() * Q * (x - xd) + _u.transpose() * R * _u;

    return c[0];
}

VectorXd MPC_model::check_constraints(const Vector2d & x0, const VectorXd & U, const Vector2d & xmax) const
{
    auto n_steps = U.rows();

    auto X = predict_trajectory(x0, U);

    VectorXd active = VectorXd::Zero(X.rows());

    VectorXd Xmax = VectorXd::Zero(n_steps * 2);

    VectorXd SU = get_S(n_steps) * U;
    VectorXd Tx0 = get_T(n_steps) * x0;

    for(uint i = 0; i < n_steps; ++i)
    {
        Xmax(2*i) = xmax(0);
        Xmax(2*i+1) = xmax(1);
    }

    //std::cout << "X\n" << X << std::endl;
    //std::cout << "Xmax\n" << Xmax << std::endl;
    //std::cout << "SU\n" << SU << std::endl;
    //std::cout << "Xmax - Tx0\n" << Xmax - Tx0 << std::endl;


    for(auto i = 0; i < n_steps; ++i)
    {
        active[2*i] = X[2*i] <= Xmax[2*i] ? 0 : 1.0;
        active[2*i+1] = X[2*i+1] <= Xmax[2*i+1] ? 0 : 1.0;

        assert((X[2*i] <= Xmax[2*i]) == (SU[2*i] <= (Xmax - Tx0)[2*i]));
        assert((X[2*i+1] <= Xmax[2*i+1]) == (SU[2*i+1] <= (Xmax - Tx0)[2*i+1]));
    }

    return active;
}

VectorXd MPC_model::get_Xd(const Vector2d & xd, int n_steps) const
{
    VectorXd Xd = VectorXd(n_steps * 2);

    for(auto i = 0; i < n_steps; ++i)
    {
        Xd(2*i) = xd(0);
        Xd(2*i+1) = xd(1);
    }
    return Xd;
}

MatrixXd MPC_model::get_Q_bar(int n_steps) const
{
    MatrixXd Q_bar = MatrixXd::Zero(2*n_steps, 2*n_steps); // coef velocity
    for(auto i = 0; i < n_steps; ++i)
    {
        Q_bar.block(2*i, 2*i, 2, 2) = Q;
    }
    return Q_bar;
}

MatrixXd MPC_model::get_Q_bar(int n_steps, const std::vector<IntA> & varss, const std::vector<Arr> & scaless) const
{
    MatrixXd Q_bar = MatrixXd::Zero(2*n_steps, 2*n_steps); // coef velocit
    assert(varss.size() == scaless.size());

    for(auto i = 0; i < varss.size(); ++i)
    {
        auto vars = varss[i];
        auto scales = scaless[i];

        for(auto j = 0; j < vars.size(); ++j)
        {
            auto p = scales[j];
            auto IJ = vars[j];
            Q_bar.block(2*IJ, 2*IJ, 2, 2) = p * Q;
        }
    }
    return Q_bar;
}

MatrixXd MPC_model::get_R_bar(int n_steps) const
{
    MatrixXd R_bar = MatrixXd::Zero(n_steps, n_steps);
    for(auto i = 0; i < n_steps; ++i)
    {
        R_bar.block(i, i, 1, 1) = R;
    }
    return R_bar;
}

MatrixXd MPC_model::get_R_bar(int n_steps, const std::vector<IntA> & varss, const std::vector<Arr> & scaless) const
{
    MatrixXd R_bar = MatrixXd::Zero(n_steps, n_steps);

    assert(varss.size() == scaless.size());

    for(auto i = 0; i < varss.size(); ++i)
    {
        auto vars = varss[i];
        auto scales = scaless[i];

        for(auto j = 0; j < vars.size(); ++j)
        {
            auto p = scales[j];
            auto IJ = vars[j];
            R_bar.block(IJ, IJ, 1, 1) = p * R;
        }
    }
    return R_bar;
}

MatrixXd MPC_model::get_S(int n_steps) const
{
    MatrixXd S = MatrixXd::Zero(n_steps*2, n_steps);
    for(auto i = 0; i < n_steps; ++i)
    {
        for(auto j = 0; j <= i; ++j)
        {
            S.block(2*i, j, 2, 1) = A.pow(i-j) * B;
        }
    }
    return S;
}

MatrixXd MPC_model::get_S(int n_steps, const std::vector<IntA> & varss) const
{
    MatrixXd S = MatrixXd::Zero(n_steps*2, n_steps);

    for(auto i = 0; i < n_steps; i+=1)
    {
        S.block(2*i, i, 2, 1) = B;
    }

    for(auto vars: varss)
    {
        for(auto i = 0; i < vars.size(); i+=1)
        {
            auto I = 2 * ( vars[i] ); // where global on traj

            for(auto j = 0; j < i; j+=1)
            {
                auto J = vars[j];
                S.block(I, J, 2, 1) = A.pow(i-j) * B;
            }
        }
    }
    return S;
}

MatrixXd MPC_model::get_T(int n_steps) const
{
    MatrixXd T = MatrixXd::Zero(n_steps*2, 2);
    for(auto i = 0; i < n_steps; i+=1)
    {
        T.block(2*i, 0, 2, 2) = A.pow(i+1);
    }
    return T;
}

MatrixXd MPC_model::get_T(int n_steps, const std::vector<IntA> & varss) const
{
    MatrixXd T = MatrixXd::Zero(n_steps*2, 2);

    for(auto vars: varss)
    {
        for(auto i = 0; i < vars.size(); i+=1)
        {
            T.block(2 * (vars[i]), 0, 2, 2) = A.pow(i+1);
        }
    }

    return T;
}
