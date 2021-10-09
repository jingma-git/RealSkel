#pragma once
#include <Eigen/Eigen>

struct Ray
{
    Eigen::Vector3d origin;
    Eigen::Vector3d dir;

    Ray() {}
    Ray(const Eigen::Vector3d &o, const Eigen::Vector3d &d) : origin(o), dir(d) {}
};