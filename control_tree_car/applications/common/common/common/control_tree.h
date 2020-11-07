#pragma once

#include <cassert>

#include <unordered_set>
#include <vector>
#include <memory>

using IntA = std::vector<int>;
using Arr = std::vector<double>;

struct TreePb
{
    std::vector<IntA> varss;
    std::vector<Arr> scaless;
    int n_steps;

    static TreePb refined(const TreePb & tree, int n_steps_per_phase)
    {
        TreePb refined_tree;
        refined_tree.varss = std::vector<IntA>(tree.varss.size());
        refined_tree.scaless = std::vector<Arr>(tree.scaless.size());

        for(uint i = 0; i < tree.varss.size(); ++i)
        {
            const auto & vars = tree.varss[i];
            const auto & scales = tree.scaless[i];

            IntA refined_vars;
            Arr refined_scales;
            refined_vars.reserve(vars.size() * n_steps_per_phase);
            refined_scales.reserve(scales.size() * n_steps_per_phase);

            for(uint j = 0; j < vars.size(); ++j)
            {
                for(int s = 0; s < n_steps_per_phase; ++s)
                {
                    refined_vars.push_back(n_steps_per_phase*vars[j] + s);
                    refined_scales.push_back(scales[j]);
                }
            }

            refined_tree.varss[i] = refined_vars;
            refined_tree.scaless[i] = refined_scales;
        }

        refined_tree.n_steps = n_steps_per_phase * tree.n_steps;

        return refined_tree;
    }

    static std::shared_ptr<TreePb> refined(const std::shared_ptr<TreePb> & tree, int n_steps_per_phase)
    {
        std::shared_ptr<TreePb> refined_tree = std::make_shared<TreePb>();
        refined_tree->varss = std::vector<IntA>(tree->varss.size());
        refined_tree->scaless = std::vector<Arr>(tree->scaless.size());

        for(uint i = 0; i < tree->varss.size(); ++i)
        {
            const auto & vars = tree->varss[i];
            const auto & scales = tree->scaless[i];

            IntA refined_vars;
            Arr refined_scales;
            refined_vars.reserve(vars.size() * n_steps_per_phase);
            refined_scales.reserve(scales.size() * n_steps_per_phase);

            for(uint j = 0; j < vars.size(); ++j)
            {
                for(int s = 0; s < n_steps_per_phase; ++s)
                {
                    refined_vars.push_back(n_steps_per_phase*vars[j] + s);
                    refined_scales.push_back(scales[j]);
                }
            }

            refined_tree->varss[i] = refined_vars;
            refined_tree->scaless[i] = refined_scales;
        }

        refined_tree->n_steps = n_steps_per_phase * tree->n_steps;

        return refined_tree;
    }

    void set_n_steps()
    {
        // sanity check (with set size and max element)
        int max = -1;
        std::unordered_set<int> ids;
        for(const auto & vars: varss)
        {
            for(const auto var: vars)
            {
                if(var > max) max = var;
                ids.insert(var);
            }
        }

        assert (max + 1 == ids.size());

        n_steps = ids.size();
    }
};

struct Tree2Branches2Steps : public TreePb
{
    Tree2Branches2Steps(double p)
    {
        varss.push_back({0, 1, 2, 3, 4, 5, 6, 7, 8, 9});
        varss.push_back({0, 1, 10, 11, 12, 13, 14, 15, 16, 17});

        scaless.push_back({1.0, 1.0, p, p, p, p, p, p, p, p});
        scaless.push_back({1.0, 1.0, 1.0-p, 1.0-p, 1.0-p, 1.0-p, 1.0-p, 1.0-p, 1.0-p, 1.0-p});

        set_n_steps();
    }
};

struct Tree1Branch : public TreePb
{
    Tree1Branch()
    {
        varss.push_back({0, 1, 2, 3, 4});

        scaless.push_back({1.0, 1.0, 1.0, 1.0, 1.0});

        set_n_steps();
    }
};

struct Tree2Branches : public TreePb
{
    Tree2Branches(double p)
    {
        varss.push_back({0, 1, 2, 3, 4});
        varss.push_back({0, 5, 6, 7, 8});

        scaless.push_back({1.0, p, p, p, p});
        scaless.push_back({1.0, 1.0-p, 1.0-p, 1.0-p, 1.0-p});

        set_n_steps();
    }
};

struct Tree3Branches : public TreePb
{
    Tree3Branches(double p, double q)
    {
        assert(p + q <= 1.0);

        varss.push_back({0, 1, 2, 3});
        varss.push_back({0, 4, 5, 6});
        varss.push_back({0, 7, 8, 9});

        scaless.push_back({1.0, p, p, p});
        scaless.push_back({1.0, q, q, q});
        scaless.push_back({1.0, 1.0 - p - q, 1.0 - p - q, 1.0 - p - q});

        set_n_steps();
    }
};

struct Tree4Branches : public TreePb
{
    Tree4Branches(double p, double q, double r)
    {
        assert(p + q + r <= 1.0);

        varss.push_back({0, 1, 2, 3, 4});
        varss.push_back({0, 5, 6, 7, 8});
        varss.push_back({0, 9, 10, 11, 12});
        varss.push_back({0, 13, 14, 15, 16});

        scaless.push_back({1.0, p, p, p, p});
        scaless.push_back({1.0, q, q, q, q});
        scaless.push_back({1.0, r, r, r, r});
        double P = 1.0 - p - q - r;
        scaless.push_back({1.0, P, P, P, P});

        set_n_steps();
    }
};

struct Tree10Branches : public TreePb
{
    Tree10Branches()
    {
        varss.push_back({0, 1, 2, 3}); //1
        varss.push_back({0, 4, 5, 6}); //2
        varss.push_back({0, 7, 8, 9}); //3
        varss.push_back({0, 10, 11, 12}); //4
        varss.push_back({0, 13, 14, 15}); //5
        varss.push_back({0, 16, 17, 18}); //6
        varss.push_back({0, 19, 20, 21}); //7
        varss.push_back({0, 22, 23, 24}); //8
        varss.push_back({0, 25, 26, 27}); //9
        varss.push_back({0, 28, 29, 30}); //10

        double p = 1.0 / 10;
        for(auto i = 0; i < 10; ++i)
        {
            scaless.push_back({1.0, p, p, p});
        }

        set_n_steps();
    }
};

struct TreeNBranches : public TreePb
{
    TreeNBranches(int n)
    {
        int j = 0;
        for(int i = 0; i < n; ++i)
        {
            varss.push_back({0, ++j, ++j, ++j, ++j});
        }

        double p = 1.0 / n;
        for(int i = 0; i < n; ++i)
        {
            scaless.push_back({1.0, p, p, p, p});
        }

        set_n_steps();
    }

    TreeNBranches(const std::vector<double> & ps)
    {
        auto n = ps.size() + 1;
        int j = 0;
        for(uint i = 0; i < n; ++i)
        {
            varss.push_back({0, ++j, ++j, ++j, ++j});
        }

        double q = 1.0;
        for(uint i = 0; i < ps.size(); ++i)
        {
            scaless.push_back({1.0, ps[i], ps[i], ps[i], ps[i]});
            q -= ps[i];
        }

        assert(q >= 0.0);

        scaless.push_back({1.0, q, q, q, q});

        set_n_steps();
    }
};

struct Tree2Stages : public TreePb
{
    Tree2Stages(double p)
    {
        varss.push_back({0, 1, 2, 3});
        varss.push_back({0, 1, 4, 5});

        varss.push_back({0, 6, 7, 8});
        varss.push_back({0, 6, 9, 10});

        scaless.push_back({1.0, 0.5, 0.25, 0.25});
        scaless.push_back({1.0, 0.5, 0.25, 0.25});
        scaless.push_back({1.0, 0.5, 0.25, 0.25});
        scaless.push_back({1.0, 0.5, 0.25, 0.25});
        //scaless.push_back({1.0, 0.5 * (1-p), 0.25 * (1-p), 0.25 * (1-p)});
        //scaless.push_back({1.0, 0.5 * (1-p), 0.25 * (1-p), 0.25 * (1-p)});

        set_n_steps();
    }
};
