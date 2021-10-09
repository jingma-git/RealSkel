#pragma once

#include "Bone.h"
#include "gather_positions_and_connectivity.h"
#include <igl/sample_edges.h>
#include <vector>

// Gather samples from bone forest roots using the current draw tips
// For each root we add 1 sample. For each non-root add k-1 samples starting at
// the tip working back toward the tail.
//
// Inputs:
//   BR  list of bone tree roots
//   k number of samples
// Output:
//   S  #samples by #dim
// Returns true on success, false on error
inline bool gather_samples(
    const std::vector<Bone *> &BR,
    const int k,
    Eigen::MatrixXd &S);

inline bool gather_samples(
    const std::vector<Bone *> &BR,
    const int k,
    Eigen::MatrixXd &S)
{
    using namespace Eigen;
    MatrixXd V;
    VectorXi P;
    MatrixXi BE;
    VectorXi WI;

    M_DEBUG << "begin gather samples" << endl;
    gather_positions_and_connectivity(BR, V, P, BE, WI);
    igl::sample_edges(V, BE, k, S);
    M_DEBUG << "gather samples successfully!" << endl;
    return true;
}