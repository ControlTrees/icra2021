#include <qp/QP_tree_problem_OSQP.h>

csc * create_csc_matrix(const MatrixXd & M)
{
    std::list<std::pair<int, int>> nz;
    std::list<int> col_starts;

    int k = 0;
    for(uint J = 0; J < M.cols(); ++J)
    {
        col_starts.push_back(k);
        for(auto I = 0; I < M.rows(); ++I)
        {
            if(M(I, J)!=0)
            {
                nz.push_back(std::make_pair(I, J));
                ++k;
            }
        }
    }

    c_int m = M.rows();
    c_int n = M.cols();
    c_int nzmax = nz.size();

    c_float * x = new c_float[nzmax];
    c_int * i   = new c_int[nzmax];
    c_int * p   = new c_int[col_starts.size()+1];

    k = 0;
    for(auto IJ : nz)
    {
        auto & I = IJ.first;
        auto & J = IJ.second;
        x[k] = M(I, J);
        i[k] = I;
        ++k;
    }

    int l = 0;
    for(auto cs : col_starts)
    {
        p[l] = cs;
        ++l;
    }

    p[col_starts.size()] = nzmax;

    return csc_matrix(m, n, nzmax, x, i, p);
}

static c_float * create_c_float_array(const VectorXd & C)
{
    c_float * q = new c_float[C.rows()];

    for(uint I = 0; I < C.rows(); ++I)
    {
        q[I] = C(I);
    }

    return q;
}


static c_float * create_default_c_float_array(int rows, double value)
{
    c_float * q = new c_float[rows];

    for(uint I = 0; I < rows; ++I)
    {
        q[I] = value;
    }

    return q;
}

QP_tree_problem_OSQP::QP_tree_problem_OSQP(const MPC_model & mpc, double u_min, double u_max)
    : QP_tree_joint_solver_base(mpc, u_min, u_max)
{
    // Workspace structures
    settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Define solver settings
    if (settings) osqp_set_default_settings(settings);
    settings->verbose = false;
    //
    //settings->polish = true;
    //settings->eps_abs *= 0.01;
    //settings->eps_rel *= 0.01;
}

QP_tree_problem_OSQP::~QP_tree_problem_OSQP()
{
    if (data) {
        if (data->P) c_free(data->P);
        if (data->q) c_free(data->q);
        if (data->A) c_free(data->A);
        if (data->l) c_free(data->l);
        if (data->u) c_free(data->u);
        c_free(data);
    }
    if (settings)  c_free(settings);
}

VectorXd QP_tree_problem_OSQP::call_solver()
{
    // P-H
    auto P = create_csc_matrix(H.triangularView<Upper>());

    // Q-C
    c_float * q = create_c_float_array(C);

    // A
    auto A = create_csc_matrix(KA);

    // l
    auto l = create_c_float_array(Lo);
    auto u = create_c_float_array(Up);

    // Exitflag
    c_int exitflag = 0;

    // Populate data
    if (data)
    {
        data->n = H.rows();
        data->m = KA.rows();
        data->P = P;
        data->q = q;
        data->A = A;
        data->l = l;
        data->u = u;
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // Get solution
    auto U_sol = Eigen::VectorXd();
    if(strcmp(work->info->status, "solved")==0)
    {
        U_sol = Eigen::VectorXd(H.rows());
        for(auto I = 0; I < H.rows();++I)
            U_sol[I] = work->solution->x[I];
    }

    // Clean workspace
    osqp_cleanup(work);

//    c_free(data->q);
//    c_free(data->l);
//    c_free(data->u);

    return U_sol;
}
