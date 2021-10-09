#pragma once

#include "gather_positions_and_connectivity.h"

#include <igl/project_to_line.h>
#include <igl/EPS.h>
#include <igl/verbose.h>
#include <igl/slice_into.h>
#include <igl/colon.h>
#include <igl/boundary_conditions.h>
#include <unordered_map>

inline bool boundary_conditions_points(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi & /*Ele*/,
    const Eigen::MatrixXd &C,
    const Eigen::VectorXi &P,
    Eigen::VectorXi &b,
    Eigen::MatrixXd &bc)
{
    using namespace Eigen;
    using namespace std;

    if (P.size() == 0)
    {
        printf("^%s: Error: no point handles found\n", __FUNCTION__);
        return false;
    }

    vector<int> bci;
    vector<int> bcj;
    vector<double> bcv;

    // loop over points
    for (int p = 0; p < P.rows(); p++)
    {
        VectorXd pos = C.row(P(p));
        // loop over domain vertices
        double min_dis = std::numeric_limits<double>::max();
        int min_vidx = -1;
        for (int i = 0; i < V.rows(); i++)
        {
            // Find samples just on pos
            //Vec3 vi(V(i,0),V(i,1),V(i,2));
            // EIGEN GOTCHA:
            // double sqrd = (V.row(i)-pos).array().pow(2).sum();
            // Must first store in temporary
            VectorXd vi = V.row(i);
            double sqrd = (vi - pos).squaredNorm();
            if (sqrd <= min_dis)
            {
                min_dis = sqrd;
                min_vidx = i;
            }
        }

        if (min_vidx >= 0)
        {
            bci.push_back(min_vidx);
            bcj.push_back(p);
            bcv.push_back(1.0);
            // cout << "p" << p << "=" << pos.transpose()
            //      << " v" << min_vidx << "=" << V.row(min_vidx) << endl;
        }
    }

    // find unique boundary indices
    vector<int> vb = bci;
    sort(vb.begin(), vb.end());
    vb.erase(unique(vb.begin(), vb.end()), vb.end());
    assert(vb.size() == P.rows() && " point handles and corresponding vertices are not consistent!\n");

    b.resize(vb.size());
    bc = MatrixXd::Zero(vb.size(), P.size());
    // Map from boundary index to index in boundary
    map<int, int> bim;
    int i = 0;
    // Also fill in b
    // cout << "mapping\n";
    for (vector<int>::iterator bit = vb.begin(); bit != vb.end(); bit++)
    {
        b(i) = *bit;
        bim[*bit] = i;
        // cout << "p" << bim[*bit] << "-v" << b(i) << endl;
        i++;
    }

    // Build BC
    // cout << __FILE__ << " " << __LINE__ << " Build BC" << endl;
    for (i = 0; i < (int)bci.size(); i++) // A vertex might lies on multiple Bone handles
    {
        assert(bim.find(bci[i]) != bim.end());
        bc(bim[bci[i]], bcj[i]) = bcv[i];
        // cout << i << ": " << bim[bci[i]] << ", " << bci[i] << ", " << bcj[i] << ", " << bcv[i] << endl;
    }

    // cout << __FILE__ << " " << __LINE__ << " bc\n"
    //      << bc << endl;

    // Normalize across rows so that conditions sum to one
    for (i = 0; i < bc.rows(); i++)
    {
        double sum = bc.row(i).sum();
        assert(sum != 0 && "Some boundary vertex getting all zero BCs");
        bc.row(i).array() /= sum;
    }

    if (bc.size() == 0)
    {
        printf("^%s: Error: boundary conditions are empty.\n", __FUNCTION__);
        return false;
    }

    // If there's only a single boundary condition, the following tests
    // are overzealous.
    if (bc.cols() == 1)
    {
        // If there is only one weight function,
        // then we expect that there is only one handle.
        assert(P.rows() == 1);
        return true;
    }

    // Check that every Weight function has at least one boundary value of 1 and
    // one value of 0
    for (i = 0; i < bc.cols(); i++)
    {
        double min_abs_c = bc.col(i).array().abs().minCoeff();
        double max_c = bc.col(i).maxCoeff();
        if (min_abs_c > igl::FLOAT_EPS)
        {
            cout << __FILE__ << " " << __LINE__ << " Error: handle " << to_string(i) << " does not receive 0 weight\n";
            return false;
        }
        if (max_c < (1 - igl::FLOAT_EPS))
        {
            cout << __FILE__ << " " << __LINE__ << " Error: handle " << to_string(i) << " does not receive 1 weight, it corresponds to multiple weight index!\n";
            return false;
        }
    }

    return true;
}

// project vertex to Bone Edges, if Bone is far from Mesh Surface for triangular mesh, there won't be boundary conditions
// b:  #b list of boundary indices (indices into V of vertices which have known, fixed values
// bc: #b(#mesh_vertices near the bone) by #weights(#bones) list of known/fixed values for boundary vertices
inline bool boundary_conditions_edges(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi & /*Ele*/,
    const Eigen::MatrixXd &C,
    const Eigen::MatrixXi &BE,
    const double dis_thresh,
    Eigen::VectorXi &b,
    Eigen::MatrixXd &bc)
{
    using namespace Eigen;
    using namespace std;

    if (BE.rows() == 0)
    {
        printf("^%s: Error: no edge handles found\n", __FUNCTION__);
        return false;
    }

    vector<int> bci; // mesh vertex indices
    // vector<int> bcj; // bone index
    // vector<double> bcv;
    std::unordered_map<int, std::vector<int>> vbm;    // vertex-bone map
    std::unordered_map<int, std::vector<double>> dis; // distance
    // loop over bone edges
    // If one vertex lies in 'near-range' of multiple bones
    //       choose the nearest bone
    for (int e = 0; e < BE.rows(); e++)
    {
        // loop over domain vertices
        for (int i = 0; i < V.rows(); i++)
        {
            // Find samples from tip up to tail
            VectorXd tip = C.row(BE(e, 0));
            VectorXd tail = C.row(BE(e, 1));

            if ((tip - tail).norm() < 1e-8)
            {
                printf("Warning! The Bone is Zero-Length!!!\n");
                bci.push_back(i);
                vbm[i].push_back(e);
                Pnt3 p = V.row(i).transpose();
                dis[i].push_back((p - tip).norm());
                continue;
            }
            // Compute parameter along bone and squared distance
            double t, sqrd;
            igl::project_to_line(
                V(i, 0), V(i, 1), V(i, 2),
                tip(0), tip(1), tip(2),
                tail(0), tail(1), tail(2),
                t, sqrd);
            // std::cout << "bone" << e << ", tip=" << tip.transpose() << " tail=" << tail.transpose()
            //           << " t=" << t << " sqrd=" << sqrd << endl;
            // // if (t >= -FLOAT_EPS && t <= (1.0f + FLOAT_EPS) && sqrd <= FLOAT_EPS)
            if (t >= -igl::FLOAT_EPS && t <= (1.0f + igl::FLOAT_EPS) && sqrd <= dis_thresh)
            {
                bci.push_back(i);
                // bcj.push_back(e);
                // bcv.push_back(1.0);
                vbm[i].push_back(e);
                dis[i].push_back(sqrd);
                // std::cout << "bone" << e << ", tip=" << tip.transpose() << " tail=" << tail.transpose()
                //           << " t=" << t << " sqrd=" << sqrd << " mesh vidx=" << i << std::endl;
            }
        }
    }

    // find unique boundary indices
    vector<int> vb = bci;
    sort(vb.begin(), vb.end());
    vb.erase(unique(vb.begin(), vb.end()), vb.end());

    b.resize(vb.size());
    bc = MatrixXd::Zero(vb.size(), BE.rows());
    // Map from boundary index(mesh vertex index) to index in boundary
    std::unordered_map<int, int> bim; // vidx, index in b
    int i = 0;
    for (vector<int>::iterator bit = vb.begin(); bit != vb.end(); bit++)
    {
        b(i) = *bit; // vidx
        bim[*bit] = i;
        i++;
    }

    // M_DEBUG << " Build BC" << endl;
    // for (i = 0; i < (int)bci.size(); i++) // A vertex might lies on multiple Bone handles
    // {
    //     assert(bim.find(bci[i]) != bim.end());
    //     bc(bim[bci[i]], bcj[i]) = bcv[i];
    //     // cout << i << ": " << bim[bci[i]] << ", " << bci[i] << ", " << bcj[i] << ", " << bcv[i] << endl;
    // }

    // M_DEBUG << " Build BC" << endl;
    for (int vidx : vb)
    {
        std::vector<int> &bone_idxs = vbm[vidx];
        std::vector<double> &dis_to_bone = dis[vidx];
        int bone_min;
        if (bone_idxs.size() > 1)
        {
            double dis_min = std::numeric_limits<double>::max();
            for (int i = 0; i < dis_to_bone.size(); ++i)
            {
                if (dis_to_bone[i] < dis_min)
                {
                    dis_min = dis_to_bone[i];
                    bone_min = bone_idxs[i];
                }
            }
        }
        else
        {
            assert(bone_idxs.size() == 1 && "Mesh vertex should correspond to at least  one handle!");
            bone_min = bone_idxs[0];
        }
        // bim: boundary index mapping for mesh vertex
        bc(bim[vidx], bone_min) = 1.0;
    }

    // Normalize across rows so that conditions sum to one
    for (i = 0; i < bc.rows(); i++)
    {
        double sum = bc.row(i).sum();
        assert(sum != 0 && "Some boundary vertex getting all zero BCs");
        bc.row(i).array() /= sum;
    }

    if (bc.size() == 0)
    {
        printf("^%s: Error: boundary conditions are empty.\n", __FUNCTION__);
        return false;
    }

    // If there's only a single boundary condition, the following tests
    // are overzealous.
    if (bc.cols() == 1)
    {
        // If there is only one weight function,
        // then we expect that there is only one handle.
        assert(BE.rows() == 1);
        return true;
    }

    // Check that every Weight function has at least one boundary value of 1 and
    // one value of 0
    for (i = 0; i < bc.cols(); i++)
    {
        double min_abs_c = bc.col(i).array().abs().minCoeff();
        double max_c = bc.col(i).maxCoeff();
        if (min_abs_c > igl::FLOAT_EPS)
        {
            cout << " Error: handle " << to_string(i) << " does not receive 0 weight\n";
            return false;
        }
        if (max_c < (1 - igl::FLOAT_EPS))
        {
            cout << " Error: handle " << to_string(i) << " does not receive 1 weight, it corresponds to multiple weight index!\n";
            return false;
        }
    }

    return true;
}

inline bool boundary_conditions_points(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &Ele,
    const std::vector<Bone *> &BR,
    Eigen::VectorXi &b,
    Eigen::MatrixXd &bc)
{
    using namespace std;
    using namespace Eigen;

    MatrixXd C;
    VectorXi P;
    MatrixXi BE;
    MatrixXi CE;
    VectorXi WI;
    gather_positions_and_connectivity(BR, C, P, BE, WI);

    // Compute boundary conditions in C,P,BE,CE style
    boundary_conditions_points(V, Ele, C, P, b, bc);
    return true;
}

inline bool boundary_conditions_edges(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &Ele,
    const std::vector<Bone *> &BR,
    const double dis_thresh,
    Eigen::VectorXi &b,
    Eigen::MatrixXd &bc)
{
    using namespace std;
    using namespace Eigen;

    MatrixXd C;
    VectorXi P;
    MatrixXi BE;
    MatrixXi CE;
    VectorXi WI;
    gather_positions_and_connectivity(BR, C, P, BE, WI);

    // Compute boundary conditions in C,P,BE,CE style
    boundary_conditions_edges(V, Ele, C, BE, dis_thresh, b, bc);
    for (int c = 0; c < bc.cols(); c++)
    {
        M_DEBUG << bc.col(c).sum() << " vertices lying on bone" << c << endl;
    }
    return true;
}

inline bool boundary_conditions(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &Ele,
    const std::vector<Bone *> &BR,
    const double dis_thresh,
    Eigen::VectorXi &b,
    Eigen::MatrixXd &bc)
{
    using namespace std;
    using namespace Eigen;

    MatrixXd C;
    VectorXi P;
    MatrixXi BE;
    MatrixXi CE;
    VectorXi WI;
    gather_positions_and_connectivity(BR, C, P, BE, WI);

    // Compute boundary conditions in C,P,BE,CE style
    igl::boundary_conditions_nearest(V, Ele, C, P, BE, CE, dis_thresh, b, bc);
    return true;
}