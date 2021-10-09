#pragma once

#include "Bone.h"
#include <vector>

// Gather list of fixed weight function indices. Weight functions are assumed to
// be fixed unless a corresponding "Bone" (handle) should exist and specifies
// that the weight function is not fixed (remember "not fixed" means
// transformations has full degrees of freedom, but may still be constrained to
// interpolate, so not necessarily "free")
// Inputs:
//   BR  list of bone tree roots
//   m  number of weight functions
// Outputs:
//   fixed  list of fixed weight indices
// Returns true on success and false on error
inline bool gather_fixed(
    std::vector<Bone *> &BR,
    const int m,
    const DegreeOfFreedomType dof_type,
    Eigen::Matrix<int, Eigen::Dynamic, 1> &fixed);

inline bool gather_fixed(
    std::vector<Bone *> &BR,
    const int m,
    const DegreeOfFreedomType dof_type,
    Eigen::Matrix<int, Eigen::Dynamic, 1> &fixed)
{
    std::vector<Bone *> B = gather_bones(BR);

    // Maks for fixed weights
    std::vector<bool> fixed_mask(m, false);
    // count of fixed
    int c = 0;
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
                    "Error: gather_fixed() max weight index of bones (%d)"
                    " >= number of weight functions (%d)\n",
                    wi,
                    m);
                return false;
            }
            if (wi >= 0 && b->tip_dof_type == dof_type)
            {
                fixed_mask[wi] = true;
                // increment count
                c++;
            }
        }
    }
    assert(c >= 0);

    // resize output
    fixed.resize(c, 1);

    // loop over mask (weight functions)
    for (int i = 0; i < m; i++)
    {
        if (fixed_mask[i])
        {
            // decrement index
            c--;
            fixed(c) = i;
        }
    }
    assert(c == 0);

    return true;
}
