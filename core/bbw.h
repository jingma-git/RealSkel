#pragma once

#include <igl/bbw.h>

// inline void compute_weights(const Eigen::MatrixXd &V, W)
// {

//     // compute BBW weights matrix
//     igl::BBWData bbw_data;
//     // only a few iterations for sake of demo
//     bbw_data.active_set_params.max_iter = 8;
//     bbw_data.verbosity = 2;
//     if (!igl::bbw(V, T, b, bc, bbw_data, W))
//     {
//         return EXIT_FAILURE;
//     }
// }