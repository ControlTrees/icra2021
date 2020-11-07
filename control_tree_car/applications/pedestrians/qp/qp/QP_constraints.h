#pragma once

#include <qp/MPC_model.h>

struct Constraints // specific 2d state
{
    Constraints(int n_steps, const std::vector<IntA> & varss)
        : n_steps(n_steps)
        , varss(varss)
    {

    }

    void add_constraint(int branch, const Eigen::Vector2d & xmax, const Eigen::Vector2d & mask)
    {
        xmaxs.push_back(std::make_tuple(branch, xmax, mask, varss[branch]));
    }

    void add_constraint(int branch, const Eigen::Vector2d & xmax, const Eigen::Vector2d & mask, const IntA & indices_branch)
    {
        IntA indices_global(indices_branch.size());

        for(uint i = 0; i < indices_branch.size(); ++i) // local -> global
        {
            int j = indices_branch[i];
            if(j == -1) j = varss[branch].size() - 1;

            indices_global[i] = varss[branch][j];
        }
        xmaxs.push_back(std::make_tuple(branch, xmax, mask, indices_global));
    }

    static Constraints refined(const Constraints & constraints, int n_steps_per_phase)
    {
        // refine the varss
        auto refined_varss = std::vector<IntA>(constraints.varss.size());

        for(uint i = 0; i < constraints.varss.size(); ++i)
        {
            const auto & vars = constraints.varss[i];

            IntA refined_vars;
            refined_vars.reserve(vars.size() * n_steps_per_phase);

            for(auto j = 0; j < vars.size(); ++j)
            {
                for(auto s = 0; s < n_steps_per_phase; ++s)
                {
                    refined_vars.push_back(n_steps_per_phase*vars[j] + s);
                }
            }

            refined_varss[i] = refined_vars;
        }

        // refine the xmaxs
       auto refined_xmaxs = std::vector<std::tuple<int, Eigen::Vector2d, Eigen::Vector2d, IntA>>(constraints.xmaxs.size());

       for(uint i = 0; i < constraints.xmaxs.size(); ++i)
       {
            const auto & indices = std::get<3>(constraints.xmaxs[i]);

            IntA refined_indices;
            refined_indices.reserve(indices.size() * n_steps_per_phase);

            for(uint j = 0; j < indices.size(); ++j)
            {
                for(uint s = 0; s < n_steps_per_phase; ++s)
                {
                    refined_indices.push_back(n_steps_per_phase*indices[j] + s);
                }
            }

            refined_xmaxs[i] = std::make_tuple(
                                std::get<0>(constraints.xmaxs[i]),
                                std::get<1>(constraints.xmaxs[i]),
                                std::get<2>(constraints.xmaxs[i]),
                                refined_indices);
       }

       Constraints refined_constraints(constraints.n_steps * n_steps_per_phase, refined_varss);
       refined_constraints.xmaxs = refined_xmaxs;

       return refined_constraints;
    }

    MatrixXd getSextract() const;
    VectorXd getXmax() const;

    bool validate() const;

    int n_steps;

    std::vector<IntA> varss;
    std::vector<std::tuple<int, Eigen::Vector2d, Eigen::Vector2d, IntA>> xmaxs; // branch, max value and mask, global indices, branch indices, the values of masks should be 0 or 1
};
