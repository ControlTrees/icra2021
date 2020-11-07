#include <qp/QP_constraints.h>

bool Constraints::validate() const
{
    return varss.size() >= xmaxs.size();
}

MatrixXd Constraints::getSextract() const
{
    // collect active constraints
    std::list< int > active_rows;
    std::vector<int> rows_activity_flag(2 * n_steps);

    for(uint c = 0; c < xmaxs.size(); ++c)
    {
        const auto& mask = std::get<2>(xmaxs[c]);
        const auto& indices = std::get<3>(xmaxs[c]);

        for(auto i: indices)
        {
            if(!rows_activity_flag[2*i] && mask(0))
            {
                rows_activity_flag[2*i] = 1;
                active_rows.push_back(2*i);
            }

            if(!rows_activity_flag[2*i+1] && mask(1))
            {
                rows_activity_flag[2*i+1] = 1;
                active_rows.push_back(2*i+1);
            }
        }
    }

    MatrixXd Sextract = Eigen::MatrixXd::Zero(active_rows.size(), 2 * n_steps);

    int I = 0;
    for(auto i : active_rows)
    {
        Sextract(I, i) = 1;
        ++I;
    }

    return Sextract;
}

VectorXd Constraints::getXmax() const
{
    MatrixXd Xmax = Eigen::VectorXd::Constant(2 * n_steps, std::numeric_limits<double>::max());

    for(uint c = 0; c < xmaxs.size(); ++c)
    {
        const auto& xmax = std::get<1>(xmaxs[c]);
        const auto& mask = std::get<2>(xmaxs[c]);
        const auto& indices = std::get<3>(xmaxs[c]);

        for(auto i: indices)
        {
            if(mask(0))
            {
                Xmax(2*i) = std::min(Xmax(2*i), xmax(0));
            }

            if(mask(1))
            {
                Xmax(2*i+1) = std::min(Xmax(2*i+1), xmax(1));
            }
        }
    }

    // following is necessary??
    for(uint i = 0; i < Xmax.rows(); ++i)
    {
        if(Xmax(i) == std::numeric_limits<double>::max())
        {
            Xmax(i) = 1.0; // put dummy value instead of max to avoid numerical instability
        }
    }

    return Xmax;
}
