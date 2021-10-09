#pragma once

#include "Bone.h"
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

// Gather the positions of each bone and the connectivity as matlab style
// matrices (V,E), or like a graph
//
// Inputs:
//   BR  list of bone tree roots
// Outputs:
//   V  vertices over which edges are defined, # vertices by dim
//   P  list of roots with weights
//   BE  edge list, # edges by 2
//   WI  #P+#BE  list of weight indices

inline void gather_positions_and_connectivity(
    const std::vector<Bone *> &BR,
    Eigen::MatrixXd &V,
    Eigen::VectorXi &P,
    Eigen::MatrixXi &BE,
    Eigen::VectorXi &WI)
{
    using namespace std;
    vector<Bone *> B;
    B = gather_bones(BR);

    // Build map from bone pointers to index in B
    unordered_map<const Bone *, int> B2I;
    B2I[NULL] = -1;
    int i = 0;
    for (vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        B2I[*bit] = i;
        i++;
    }

    // count weighted roots, for point_handles
    int weighted_roots = 0;
    for (vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        if ((*bit)->is_root() && (*bit)->get_wi() >= 0)
        {
            weighted_roots++;
        }
    }

    // Resize list of vertices, one for each "Bone" including roots
    // Resize list of edges, one for each bone segment, so excluding roots
    V.resize(B.size(), 3);
    BE.resize(B.size() - BR.size(), 2);
    // Only consider point handles at weighted roots
    P.resize(weighted_roots);
    WI.resize(weighted_roots + BE.rows());
    int e = 0;
    int p = 0;
    M_DEBUG << "Bones=" << B.size() << " BE=" << BE.rows() << " Points=" << P.rows() << " WI=" << WI.rows() << endl;
    // loop over bones
    for (vector<Bone *>::iterator bit = B.begin(); bit != B.end(); ++bit)
    {
        Bone *b = *bit;
        // Store position
        V.row(B2I[b]) = b->rest_tip();
        // M_DEBUG << "handle" << b->get_wi() << " V[" << B2I[b] << "]=" << V.row(B2I[b]) << endl;
        // If not root, then store connectivity
        if (!b->is_root())
        {
            BE(e, 0) = B2I[b->get_parent()];
            BE(e, 1) = B2I[b];

            // Bone edges are expected to have weight indices
            // assert(b->get_wi() >= 0);
            WI(P.size() + e) = b->get_wi();
            // M_DEBUG << "bone" << e << ": " << BE(e, 0) << "-" << BE(e, 1) << " WI=" << WI(P.size() + e) << endl;
            e++;
        }
        else if (b->get_wi() >= 0)
        {
            // Point Handle
            P(p) = B2I[b];
            WI(p) = b->get_wi();
            // M_DEBUG << "point" << p << ": " << P(p) << " WI=" << WI(p) << endl;
            p++;
        }
    }
}
