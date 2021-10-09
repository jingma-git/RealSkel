#pragma once
#include <iostream>

#include <igl/speye.h>
#include <igl/repdiag.h>
#include <igl/slice_into.h>
#include <igl/slice.h>
#include <igl/cat.h>
#include <igl/covariance_scatter_matrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/arap_rhs.h>
#include <igl/polar_svd3x3.h>
#include <igl/columnize.h>
#include <igl/get_seconds.h>
#include <igl/cotmatrix.h>

enum SolverStatus
{
    // Good
    SOLVER_STATUS_CONVERGED = 0,
    // OK
    SOLVER_STATUS_MAX_ITER = 1,
    // Bad
    SOLVER_STATUS_ERROR = 2,
    NUM_SOLVER_STATUSES = 3,
};

enum ARAPEnergyType
{
    ARAP_ENERGY_TYPE_SPOKES = 0,
    ARAP_ENERGY_TYPE_SPOKES_AND_RIMS = 1,
    ARAP_ENERGY_TYPE_ELEMENTS = 2,
    ARAP_ENERGY_TYPE_DEFAULT = 3,
    NUM_ARAP_ENERGY_TYPES = 4
};

struct ARAPData
{
    int n;
    ARAPEnergyType energy;
    Eigen::SparseMatrix<double> K, M; // rhs pre-multiplier #V*3 * #V diffirential coordinates, mass matrix
    Eigen::SparseMatrix<double> CSM;  // covariance matrix S=VV'
                                      // to find best rotation by S=U*sigma*W' Ri=UW'
    igl::min_quad_with_fixed_data<double> solver_data;
    Eigen::VectorXi b;
    int dim;

    int max_iter;
    double solution_diff_threshold;
    double inactive_threshold;

    ARAPData() : n(0),
                 energy(ARAP_ENERGY_TYPE_SPOKES_AND_RIMS),
                 max_iter(10),
                 solution_diff_threshold(1.0e-14),
                 inactive_threshold(1.0e-14),
                 K(),
                 CSM(),
                 solver_data(),
                 b(),
                 dim(-1) // force this to be set by _precomputation
                 {};
};

inline void fit_rotations(const Eigen::MatrixXd &S,
                          const bool single_precision,
                          Eigen::MatrixXd &R)
{
    using namespace std;
    using namespace Eigen;

    const int dim = S.cols();
    const int nr = S.rows() / dim;
    R.resize(dim, dim * nr);
    Matrix3d si;
    for (int r = 0; r < nr; r++)
    {
        for (int i = 0; i < dim; i++)
        {
            for (int j = 0; j < dim; j++)
            {
                si(i, j) = S(i * nr + r, j);
            }
        }
        // cout << "rotation" << r << endl;
        // cout << si << endl;

        Matrix3d ri;
        if (single_precision)
        {
            // Eigen::JacobiSVD<Eigen::Matrix3d> sol(si, Eigen::ComputeFullU | Eigen::ComputeFullV);
            // ri = sol.matrixV() * sol.matrixU().transpose();
            igl::polar_svd3x3(si, ri);
        }
        assert(ri.determinant() >= 0);
        R.block(0, r * dim, dim, dim) = ri.block(0, 0, dim, dim).transpose();
    }
}

inline SolverStatus arap_layer(const Eigen::MatrixXd &V,
                               const Eigen::MatrixXi &F,
                               const Eigen::VectorXi &b,
                               const Eigen::MatrixXd &bc,
                               const Eigen::SparseMatrix<double> &Aeq,
                               const Eigen::VectorXd &Beq,
                               const Eigen::SparseMatrix<double> &Aieq,
                               const Eigen::VectorXd &Bieq,
                               const Eigen::VectorXd &p_lx,
                               const Eigen::VectorXd &p_ux,
                               ARAPData &data,
                               Eigen::MatrixXd &U)
{
    using namespace std;
    using namespace Eigen;

    int n = V.rows();
    int dim = V.cols();
    assert(dim == 3);

    SolverStatus ret = SOLVER_STATUS_ERROR;
    data.dim = V.cols();
    U.resize(V.rows(), V.cols());
    U = V;
    //----------------------------------Precompute Laplace & Covariance Scatter Matrix-----------------------
    SparseMatrix<double> L;
    igl::cotmatrix(V, F, L);
    igl::ARAPEnergyType eff_energy = igl::ARAP_ENERGY_TYPE_SPOKES_AND_RIMS;
    // igl::ARAPEnergyType eff_energy = igl::ARAP_ENERGY_TYPE_SPOKES;

    // covariance scatter matrix CSM: dim*#V x dim*#V
    // fit rotations
    igl::covariance_scatter_matrix(V, F,
                                   eff_energy,
                                   data.CSM);

    SparseMatrix<double> G_sum, G_sum_dim;
    igl::speye(n, G_sum);
    igl::repdiag(G_sum, data.dim, G_sum_dim);
    data.CSM = (G_sum_dim * data.CSM).eval();
    // rhs K
    igl::arap_rhs(V, F, data.dim, eff_energy, data.K);
    SparseMatrix<double> Q = (-L).eval();

    //------Active Set with Quadratic Programing to solve system with inequality constraints-----------------------
    VectorXd lx, ux;
    if (p_lx.size() == 0)
    {
        lx = VectorXd::Constant(n, 1, -std::numeric_limits<double>::max());
    }
    else
    {
        lx = p_lx;
    }

    if (p_ux.size() == 0)
    {
        ux = VectorXd::Constant(n, 1, std::numeric_limits<double>::max());
    }
    else
    {
        ux = p_ux;
    }
    const int nk = bc.rows();

    VectorXi as_lx = VectorXi::Constant(n, 1, false);
    VectorXi as_ux = VectorXi::Constant(n, 1, false);
    VectorXi as_ieq = VectorXi::Constant(Aieq.rows(), 1, false);

    int iter = 0;
    VectorXd Z = U.col(2);
    VectorXd old_Z = VectorXd::Constant(n, 1, std::numeric_limits<double>::max());
    while (true)
    {
        int new_as_lx = 0;
        int new_as_ux = 0;
        int new_as_ieq = 0;
        // 1. check which unknowns violate the constraints and should be activated in this iteration
        if (Z.size() > 0)
        {
            for (int i = 0; i < n; i++)
            {
                if (Z(i) < lx(i))
                {
                    new_as_lx += (as_lx(i) ? 0 : 1);
                    as_lx(i) = true;
                    break;
                }

                if (Z(i) > ux(i))
                {
                    new_as_ux += (as_ux(i) ? 0 : 1);
                    as_ux(i) = true;
                    break;
                }
            }

            if (Aieq.rows() > 0)
            {
                VectorXd AieqZ;
                AieqZ = Aieq * Z;
                for (int a = 0; a < Aieq.rows(); a++)
                {
                    if (AieqZ(a) > Bieq(a))
                    {
                        new_as_ieq += (as_ieq(a) ? 0 : 1);
                        as_ieq(a) = true;
                    }
                }
            }

            const double diff = (Z - old_Z).squaredNorm();
            printf("Iter%d: energy=%.6f\n", iter, diff);
            if (diff < data.solution_diff_threshold)
            {
                ret = SOLVER_STATUS_CONVERGED;
                printf("Iter%d: ................Converged\n", iter);
                break;
            }
            old_Z = Z;
            // cout << Z << endl;
        }

        const int as_lx_count = std::count(as_lx.data(), as_lx.data() + n, true);
        const int as_ux_count = std::count(as_ux.data(), as_ux.data() + n, true);
        const int as_ieq_count = std::count(as_ieq.data(), as_ieq.data() + as_ieq.size(), true);
        printf("Iter%d: as_lx=%d as_ux=%d, as_ieq=%d\n", iter, as_lx_count, as_ux_count, as_ieq_count);

        // 2. prepare fixed value, lower and upper bound
        VectorXi known_i;
        known_i.resize(nk + as_lx_count + as_ux_count, 1);
        MatrixXd Y_i;
        Y_i.resize(nk + as_lx_count + as_ux_count, dim);
        {
            known_i.block(0, 0, b.rows(), b.cols()) = b;
            Y_i.block(0, 0, bc.rows(), bc.cols()) = bc;
            cout << __FILE__ << " " << __LINE__ << " nk=" << nk << endl;
            int k = nk;
            for (int z = 0; z < n; z++)
            {
                if (as_lx(z))
                {
                    known_i(k) = z;
                    for (int d = 0; d < dim; d++)
                    {
                        Y_i(k, d) = lx(z);
                    }
                    k++;
                }
            }

            for (int z = 0; z < n; z++)
            {
                if (as_ux(z))
                {
                    known_i(k) = z;
                    for (int d = 0; d < dim; d++)
                    {
                        Y_i(k, d) = ux(z);
                    }
                    k++;
                }
            }
            assert(k == Y_i.rows());
            assert(k == known_i.rows());
        }

        // 3. Gather active constraints
        //    3.1 filter active constraints
        VectorXi as_ieq_list(as_ieq_count, 1);
        VectorXd Beq_i;
        Beq_i.resize(Beq.rows() + as_ieq_count, 1);
        Beq_i.head(Beq.rows()) = Beq;
        {
            int k = 0;
            for (int a = 0; a < as_ieq.size(); a++)
            {
                if (as_ieq(a))
                {
                    assert(k < as_ieq_list.size());
                    as_ieq_list(k) = a;
                    Beq_i(Beq.rows() + k, 0) = Bieq(k, 0);
                    k++;
                }
            }
            assert(k == as_ieq_count);
        }
        //    3.2 extract active constraint rows and append them to equality equation
        SparseMatrix<double> Aeq_i, Aieq_i;
        igl::slice(Aieq, as_ieq_list, 1, Aieq_i);
        igl::cat(1, Aeq, Aieq_i, Aeq_i);
        // 4. solve unknowns
        VectorXd sol;
        if (known_i.size() == Q.rows()) // Everything is fixed
        {
            Z.resize(Q.rows(), 1);
            igl::slice_into(Y_i.col(2).eval(), known_i, 1, Z);
            sol.resize(0, Y_i.cols());
            assert(Aeq_i.rows() == 0 && "All fixed but linearly constrained");
        }
        else
        {
            // cout << __FILE__ << " " << __LINE__ << " U:" << U.rows() << " V:" << V.rows() << " bc:" << bc.rows() << endl;
            // ---------4.1 Local Step, fit best rotation matrix
            for (int bi = 0; bi < bc.rows(); bi++)
            {
                // cout << bi << "|" << b(bi) << "|" << bc.row(bi) << endl;
                U.row(b(bi)) = bc.row(bi);
            }
            const auto &Udim = U.replicate(data.dim, 1);
            MatrixXd S = data.CSM * Udim;
            S /= S.array().abs().maxCoeff();
            int Rdim = data.dim;
            MatrixXd R(Rdim, data.CSM.rows()); // 3 x 3n
            fit_rotations(S, true, R);
            MatrixXd eff_R = R;
            int num_rots = data.K.cols() / Rdim / Rdim; // num_vertex

            VectorXd Rcol;
            igl::columnize(eff_R, num_rots, 2, Rcol);
            VectorXd Bcol = -data.K * Rcol;
            // ---------4.2 Solve, z-dim should conform to the inequality constraint
            for (int c = 0; c < data.dim; c++)
            {
                // cout << "Solve for column " << c << endl;
                // 4.1 factoring/precomputation, provided LHS matrix and boundary indices
                SparseMatrix<double> Aeq_c;
                if (c == 2)
                {
                    Aeq_c = Aeq_i;
                }
                if (!igl::min_quad_with_fixed_precompute(Q, b, Aeq_c, true, data.solver_data))
                {
                    cerr << "Error: min_quad_with_fixed precomputation failed." << endl;
                    if (iter > 0 && Aeq_i.rows() > Aeq.rows())
                    {
                        cerr << "  *Are you sure rows of [Aeq;Aieq] are linearly independent?*" << endl;
                    }
                    ret = SOLVER_STATUS_ERROR;
                    break;
                }
                // ---------4.2.2 Global Step, find optimal vertex positions
                VectorXd Uc, Bc, bcc, Beq_c;
                if (c == 2) // z-dim
                {
                    Beq_c = Beq_i;
                }
                Bc = Bcol.block(c * n, 0, n, 1);
                if (bc.size() > 0)
                {
                    bcc = bc.col(c);
                }
                if (!igl::min_quad_with_fixed_solve(data.solver_data, Bc, bcc, Beq_c, Uc, sol))
                {
                    cerr << "Error: min_quad_with_fixed solve failed." << endl;
                    ret = SOLVER_STATUS_ERROR;
                    break;
                }
                U.col(c) = Uc;
                assert(data.solver_data.Auu_sym);

                // ---------4.2.3 active set post-process: inactivate variables already conforming to constraints
                if (c == 2)
                {
                    Z = Uc;
                    SparseMatrix<double> Qk;
                    igl::slice(Q, known_i, 1, Qk);
                    VectorXd Bk;
                    igl::slice(Bc, known_i, Bk);
                    MatrixXd Lambda_known_i = -(0.5 * Qk * Z + 0.5 * Bk);
                    // reverse lambda values for lx
                    Lambda_known_i.block(nk, 0, as_lx_count, 1) =
                        (-1 * Lambda_known_i.block(nk, 0, as_lx_count, 1)).eval();
                    // Extract Lagrange multipliers for Aieq_i (always at back of sol)
                    VectorXd Lambda_Aieq_i(Aieq_i.rows(), 1);
                    for (int l = 0; l < Aieq_i.rows(); l++)
                    {
                        Lambda_Aieq_i(Aieq_i.rows() - 1 - l) = sol(sol.rows() - 1 - l);
                    }
                    // remove from active set
                    for (int l = 0; l < as_lx_count; l++)
                    {
                        if (Lambda_known_i(nk + l) < data.inactive_threshold)
                        {
                            as_lx(known_i(nk + l)) = false;
                        }
                    }
                    for (int u = 0; u < as_ux_count; u++)
                    {
                        if (Lambda_known_i(nk + as_lx_count + u) < data.inactive_threshold)
                        {
                            as_ux(known_i(nk + as_lx_count + u)) = false;
                        }
                    }
                    for (int a = 0; a < as_ieq_count; a++)
                    {
                        if (Lambda_Aieq_i(a) < data.inactive_threshold)
                        {
                            as_ieq(int(as_ieq_list(a))) = false;
                        }
                    }
                }
            }
        }

        iter++;
        if (data.max_iter > 0 && iter >= data.max_iter)
        {
            ret = SOLVER_STATUS_MAX_ITER;
            cout << "Solve without converged, exceed maximum iteration\n"
                 << endl;
            break;
        }
    }
}
