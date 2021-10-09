#pragma once
#include <Eigen/Eigen>

struct Plane
{
    double a, b, c, d;
    Eigen::Vector3d p; // point on plane

    Plane() : a(0), b(0), c(0), d(0) {}
    Plane(double a_, double b_, double c_, double d_)
        : a(a_), b(b_), c(c_), d(d_) {}
    Plane(const Eigen::Vector3d &n, const Pnt3 &p_)
        : a(n[0]), b(n[1]), c(n[2]),
          d(-p_[0] * n[0] - p_[1] * n[1] - p_[2] * n[2]),
          p(p_) {}

    Plane(const Eigen::Vector3d &n, double d_)
        : a(n[0]), b(n[1]), c(n[2]), d(d_) {}

    Eigen::Vector3d normal() const
    {
        return Eigen::Vector3d(a, b, c);
    }

    bool on_xy()
    {
        return fabs(c) < 1e-8;
    }
};

inline std::ostream &operator<<(std::ostream &out, const Plane &p)
{
    out << "Plane a=" << p.a << " b=" << p.b << " c=" << p.c << " d=" << p.d;
    return out;
}

inline Plane triag_to_plane(const Eigen::Vector3d &p0,
                            const Eigen::Vector3d &p1,
                            const Eigen::Vector3d &p2)
{
    using namespace Eigen;
    // Vector3d normal = ((p1 - p0).cross(p2 - p0)).normalized();
    Vector3d normal = ((p1 - p0).cross(p2 - p0));
    double d = -(p0.x() * normal[0] + p0.y() * normal[1] + p0.z() * normal[2]);
    return Plane(normal, d);
}