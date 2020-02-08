#pragma once

#include <control_tree/qp/MPC_model.h>

struct Constraints
{
    Constraints(int n_steps, const std::vector<intA> & varss)
        : n_steps(n_steps)
        , varss(varss)
    {

    }

    void add_constraint(int branch, const Eigen::Vector2d & xmax, const Eigen::Vector2d & mask)
    {
        xmaxs.push_back(std::make_tuple(branch, xmax, mask, varss[branch]));
    }

    void add_constraint(int branch, const Eigen::Vector2d & xmax, const Eigen::Vector2d & mask, const intA & indices_branch)
    {
        intA indices_global(indices_branch.size());

        for(auto i = 0; i < indices_branch.size(); ++i) // local -> global
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
        auto refined_varss = std::vector<intA>(constraints.varss.size());

        for(auto i = 0; i < constraints.varss.size(); ++i)
        {
            const auto & vars = constraints.varss[i];

            intA refined_vars;
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
       auto refined_xmaxs = std::vector<std::tuple<int, Eigen::Vector2d, Eigen::Vector2d, intA>>(constraints.xmaxs.size());

       for(auto i = 0; i < constraints.xmaxs.size(); ++i)
       {
            const auto & indices = std::get<3>(constraints.xmaxs[i]);

            intA refined_indices;
            refined_indices.reserve(indices.size() * n_steps_per_phase);

            for(auto j = 0; j < indices.size(); ++j)
            {
                for(auto s = 0; s < n_steps_per_phase; ++s)
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

    MatrixXd reduced() const;

    MatrixXd getSextract() const;
    VectorXd getXmax() const;

    bool validate() const;

    int n_steps;

    std::vector<intA> varss;
    std::vector<std::tuple<int, Eigen::Vector2d, Eigen::Vector2d, intA>> xmaxs; // branch, max value and mask, indices, the values of masks should be 0 or 1
};
