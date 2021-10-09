#pragma once

#include "Bone.h"
#include <vector>

// Distribute transformations from stack of affine transformations into bone
// forest roots transformations (in the last_T field)
//
// Inputs:
//   T  #rows by 4*num_handles horizontal stack of transformations
//   BR  list of bone tree roots
// Output:
//   BR  list of bone tree roots updated with last_T from T
// Returns true on success, false on error
inline bool distribute_transformations(
    const Eigen::MatrixXf &T,
    std::vector<Bone *> &BR);

// Implementation
#include <list>

inline bool distribute_transformations(
    const Eigen::MatrixXd &T,
    std::vector<Bone *> &BR)
{
    int num_handles = T.cols() / 4;
    std::vector<Bone *> B = gather_bones(BR);

    map<const Bone *, int> B2I;
    B2I[NULL] = -1;
    int i = 0;
    for (vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        B2I[*bit] = i;
        i++;
    }

    // loop over bones
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->wi_is_set())
        {
            int wi = b->get_wi();
            if (wi >= num_handles)
            {
                fprintf(
                    stderr,
                    "Error: verify_bone_roots() max weight index of bones (%d)"
                    " >= number of handles (%d)\n",
                    wi,
                    num_handles);
                return false;
            }
            if (wi >= 0)
            {
                int idx = B2I[b] - BR.size();
                // cout << __FILE__ << " " << __LINE__ << "handle" << wi << " B2I=" << B2I[b] << " " << idx << endl;
                // compute current transformation at bone
                // place transform into bone's last_T
                b->last_T.affine() = T.block(0, idx * 4, 3, 4);
            }
        }
        b->is_tip_changed = true;
    }

    return true;
}

inline bool distribute_transformations_vertical(
    const Eigen::MatrixXd &T,
    std::vector<Bone *> &BR)
{
    int num_handles = T.rows() / 4;
    std::vector<Bone *> B = gather_bones(BR);

    // loop over bones
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->wi_is_set())
        {
            int wi = b->get_wi();
            if (wi >= num_handles)
            {
                fprintf(
                    stderr,
                    "Error: verify_bone_roots() max weight index of bones (%d)"
                    " >= number of handles (%d)\n",
                    wi,
                    num_handles);
                return false;
            }
            if (wi >= 0)
            {
                // compute current transformation at bone
                // place transform into bone's last_T
                b->last_T.affine() = T.block(b->get_wi() * 4, 0, 4, 3).transpose();
                b->is_tip_changed = true;
            }
        }
    }

    return true;
}
