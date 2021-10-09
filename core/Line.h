#pragma once

#include "common.h"

struct Line
{
    Pnt3 p0;
    Pnt3 p1;
    Line() {}
    Line(const Pnt3 &p0_, const Pnt3 &p1_) : p0(p0_), p1(p1_) {}
    Vec3 dir() const { return p1 - p0; }

    Line trimmed_line(double trim_ratio)
    {
        return Line(p0 + trim_ratio * dir(), p1 - trim_ratio * dir());
    }
};

inline std::ostream &operator<<(std::ostream &out, const Line &l)
{
    out << l.p0.transpose() << "|" << l.p1.transpose();
    return out;
}
