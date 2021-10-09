#pragma once

#include "Bone.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Gather positional constraints rhs entries implied by bone forest
//
// Inputs:
//   BR  list of bone tree roots
//   m  number of handles/weight functions (might be more than the number of
//     weight influencing bones)
//   dim  number of dimensions for constraints, 2 or 3
// Output:
//   Beq  dim*#constraint_points constraint values.
// Returns true on success, false on error
//
// See also gather_positional_constraints_system
inline bool gather_positional_constraints_rhs(
    const std::vector<Bone *> &BR,
    const int m,
    const int dim,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &B_eq);

// Append constraints for a given position and influencing weight function (one
// constraint for each dimension)
//
// Inputs:
//   p  >dim position vector
//   m  number of handles/weight functions (might be more than the number of
//     weight influencing bones)
//   dim  number of dimensions for constraints, 2 or 3, if less than dimension
//     of p then first dim coordinates are used
//   B_eq  current B_eq (see above) with only num_pc non zeros entries at the
//     top, and at least dim rows free after those
//   num_pc  current number of non-zero rows at the top of dyn_A_eq
// Outputs:
//   B_eq  updated B_eq
//   num_pc  updated number of non-zero rows at the top of dyn_A_eq
//     (num_pc += dim)
//
inline void append_to_constraints_rhs(
    const Vec3 &p,
    const int wi,
    const int m,
    const int dim,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &B_eq,
    int &num_pc);

inline bool gather_positional_constraints_rhs(
    const std::vector<Bone *> &BR,
    const int m,
    const int dim,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &B_eq)
{
    std::vector<Bone *> B = gather_bones(BR);

    // The maximum number of constraints per weight function is in general the
    // number of dimensions+1 (completely fixed).
    // Resize with an over estimate of the number of constraints
    B_eq.resize(m * dim * (dim + 1));
    // Should probably use some hueristic to guess how many non-zeros to reserve here

    // Current count of constraints (non zero rows in dyn_A_eq)
    int num_pc = 0;

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
                    "Error: gather_positional_constraints_rhs() max weight index of bones (%d)"
                    " >= number of handles (%d)\n",
                    wi,
                    m);
                return false;
            }
            if (wi >= 0)
            {
                // Add positional constraint for position constraint tips
                if (b->tip_dof_type == DOF_TYPE_FIXED_POSITION)
                {
                    // always add tip constraint (even for roots)
                    append_to_constraints_rhs(b->tip(false, false), wi, m, dim, B_eq, num_pc);
                }
                // Add tail if not fixed
                if (b->tip_dof_type != DOF_TYPE_FIXED_ALL)
                {
                    // If tail exists and is not FREE then also add positional constraint
                    // for tail
                    if (!b->is_root() &&
                        (b->get_parent()->tip_dof_type == DOF_TYPE_FIXED_POSITION ||
                         b->get_parent()->tip_dof_type == DOF_TYPE_FIXED_ALL))
                    {
                        // if non root then also add constraint for tail
                        Vec3 tail = b->get_parent()->tip(false, false);
                        append_to_constraints_rhs(tail, wi, m, dim, B_eq, num_pc);
                    }
                }
            }
        }
    }

    // Only keep top num_pc
    B_eq.conservativeResize(num_pc);

    return true;
}

inline void append_to_constraints_rhs(
    const Vec3 &p,
    const int /*wi*/,
    const int /*m*/,
    const int dim,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &Beq,
    int &num_pc)
{
    // Loop over rows corresponding to this positional constraint
    // Also rows in transformation entries
    for (int r = 0; r < dim; r++)
    {
        Beq(num_pc) = p(r);
        // increment number of position constraints
        num_pc++;
    }
}
