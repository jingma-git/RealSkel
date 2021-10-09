#include "Bone.h"
#include <vector>

// Gather transformations from bone forest roots into stack of affine
// transformations
// Inputs:
//   BR  list of bone tree roots
//   use_last_T  use last_T transformation instead of .affine()
//   T  #rows by 4*num_handles horizontal stack of transformations
// Output:
//   T  #rows by 4*num_handles horizontal stack of transformations
// Returns true on success, false on error
inline bool gather_transformations(
    const std::vector<Bone *> &BR,
    const bool use_last_T,
    Eigen::MatrixXd &T);

#include <list>

inline bool gather_transformations(
    const std::vector<Bone *> &BR,
    const bool use_last_T,
    Eigen::MatrixXd &T)
{

    std::vector<Bone *> B = gather_bones(BR);
    int num_handles = 0;
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->get_wi() >= 0)
        {
            num_handles++;
        }
    }
    T.resize(3, num_handles * 4);

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
                    "Error: gather_transformations() max weight index of bones (%d)"
                    " >= number of handles (%d)\n",
                    wi,
                    num_handles);
                return false;
            }
            if (wi >= 0)
            {
                // compute current transformation at bone
                Tform3 bT;
                if (use_last_T)
                {
                    bT = b->last_T;
                }
                else
                {
                    bT = b->affine();
                }
                int idx = B2I[b] - BR.size();
                // M_DEBUG << "handle" << wi << " B2I=" << B2I[b] << " " << idx << endl;
                // cout << T.block(0, idx * 4, 3, 4) << endl;
                // place transform into T stack
                T.block(0, idx * 4, 3, 4) = bT.affine().matrix();
                // M_DEBUG << ": handle" << wi << " affine=\n";
                // cout << T.block(0, b->get_wi() * 4, 3, 4) << endl;
            }
        }
    }

    return true;
}

inline bool gather_transformations_vertical(
    const std::vector<Bone *> &BR,
    const bool use_last_T,
    Eigen::MatrixXd &T)
{

    std::vector<Bone *> B = gather_bones(BR);
    int num_handles = 0;
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->get_wi() >= 0)
        {
            num_handles++;
        }
    }
    T.resize(num_handles * 4, 3);

    // loop over bones
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->wi_is_set())
        {
            int wi = b->get_wi();
            if (wi >= 0)
            {
                // compute current transformation at bone
                Tform3 bT;
                if (use_last_T)
                {
                    bT = b->last_T;
                }
                else
                {
                    bT = b->affine();
                }
                // place transform into T stack
                T.block(b->get_wi() * 4, 0, 4, 3) = bT.affine().matrix().transpose();
                // cout << __FILE__ << " " << __LINE__ << ": handle" << wi << " affine=\n";
                // cout << T.block(0, b->get_wi() * 4, 3, 4) << endl;
            }
        }
    }

    return true;
}