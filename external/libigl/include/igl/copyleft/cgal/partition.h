#pragma once
#ifndef IGL_COPYLEFT_CGAL_ORIENT_3D_H
#define IGL_COPYLEFT_CGAL_ORIENT_3D_H

#include "../../igl_inline.h"
#include <vector>
#include <Eigen/Eigen>
// #include <Eigen/Eigen>
namespace igl
{
    namespace copyleft
    {
        namespace cgal
        {
            // Inputs:
            // Outputs:
            IGL_INLINE void partition(
                const std::vector<Eigen::Vector3d> &polygon_pts,
                std::vector<std::vector<Eigen::Vector3d>> &partition_polys);
        }
    }
}

#ifndef IGL_STATIC_LIBRARY
#include "partition.cpp"
#endif
#endif