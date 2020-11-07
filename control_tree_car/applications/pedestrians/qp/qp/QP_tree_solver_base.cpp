#include <qp/QP_tree_solver_base.h>

QP_tree_solver_base::QP_tree_solver_base(const MPC_model & mpc, double u_min, double u_max)
    : mpc(mpc)
    , u_min_(u_min)
    , u_max_(u_max)
{

}

QP_tree_joint_solver_base::QP_tree_joint_solver_base(const MPC_model & mpc, double u_min, double u_max)
    : QP_tree_solver_base(mpc, u_min, u_max)
    , control_bounds_in_A_(true)
{

}

VectorXd QP_tree_joint_solver_base::solve(const Vector2d & x0, const Vector2d & xd, const Constraints & k,
                                int n_steps,
                                const std::vector<IntA> & varss,
                                const std::vector<Arr> & scaless)
{
    // build matrices
    S = mpc.get_S(n_steps, varss);
    T = mpc.get_T(n_steps, varss);

    Q_bar = mpc.get_Q_bar(n_steps, varss, scaless);
    R_bar = mpc.get_R_bar(n_steps, varss, scaless);

    H = 2 * (R_bar + S.transpose() * Q_bar * S);
    F = 2 * (T.transpose() * Q_bar * S);
    G = 2 * Q_bar * S;

    const Eigen::VectorXd & Xd = mpc.get_Xd(xd, n_steps);
    C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();

    /// Constraints
    const Eigen::VectorXd Xmax = k.getXmax();
    const Eigen::MatrixXd Sextract = k.getSextract();

    if(!KA.rows())
    {
        if(control_bounds_in_A_)
        {
            KA.resize(Sextract.rows() + H.rows(), H.rows());
            Up.resize(Sextract.rows() + H.rows());
            Lo.resize(Sextract.rows() + H.rows());
        }
        else
        {
            KA.resize(Sextract.rows(), H.rows());
            Up.resize(Sextract.rows());
            Lo.resize(Sextract.rows());
        }
    }

    // traj constraints
    KA.block(0, 0, Sextract.rows(), H.rows()) = Sextract * S;
    Up.head(Sextract.rows()) = Sextract * (Xmax - T * x0);
    Lo.head(Up.rows()) = VectorXd::Constant(Up.rows(), std::numeric_limits<double>::lowest());

    // control bounds constraints
    if(control_bounds_in_A_)
    {
        KA.block(Sextract.rows(), 0, H.rows(), H.rows()) = MatrixXd::Identity(H.rows(), H.rows());
        Up.tail(H.rows()) = VectorXd::Constant(H.rows(), u_max_);
        Lo.tail(H.rows()) = VectorXd::Constant(H.rows(), u_min_);
    }

    // call solver
    return call_solver();
}

