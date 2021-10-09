#pragma once
#include "Bone.h"
#include <vector>

// Gather list of free weight function indices. Weight functions are assumed to
// be free unless a corresponding "Bone" (handle) should exist and specifies
// that the weight function is not free (remember "not free" means
// "interpolated", but not necessarily "fixed")
// Inputs:
//   BR  list of bone tree roots
//   m  number of weight functions
// Outputs:
//   free  list of free weight indices
// Returns true on success and false on error
inline bool gather_free(
    std::vector<Bone *> &BR,
    const int m,
    Eigen::Matrix<int, Eigen::Dynamic, 1> &free);

inline bool gather_free(
    std::vector<Bone *> &BR,
    const int m,
    Eigen::Matrix<int, Eigen::Dynamic, 1> &free)
{
    // clear output
    std::vector<Bone *> B = gather_bones(BR);

    // Mask for free weights
    std::vector<bool> free_mask(m, true);
    // count of free
    int c = m;

    // loop over bones
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        if (b->wi_is_set())
        {
            int wi = b->get_wi();
            if (wi >= m)
            {
                fprintf(
                    stderr,
                    "Error: gather_free() max weight index of bones (%d)"
                    " >= number of weight functions (%d)\n",
                    wi,
                    m);
                return false;
            }
            if (wi >= 0 && b->tip_dof_type != DOF_TYPE_FREE)
            {
                free_mask[wi] = false;
                // decrement count
                c--;
            }
        }
    }
    assert(c >= 0);

    // resize output
    free.resize(c, 1);

    // loop over mask (weight functions)
    for (int i = 0; i < m; i++)
    {
        if (free_mask[i])
        {
            // decrement index
            c--;
            free(c) = i;
        }
    }
    assert(c == 0);

    return true;
}