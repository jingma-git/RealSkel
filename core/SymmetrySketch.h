#pragma once
#include "common.h"
#include "Line.h"

enum SymAxis
{
    SYM_AXIS_X,
    SYM_AXIS_Y
};

class SymmetrySketch
{
public:
    void set_view_dir(const Vec3 &view_dir) { view_dir_ = view_dir; }
    void set_sym_axis(int sym_axis) { sym_axis_ = sym_axis; }
    void add_pt(const Pnt3 &p);
    void to_polygon(std::vector<Pnt3> &polygon);
    void clear()
    {
        l0_.clear();
        l1_.clear();
        max_sym_len_ = std::numeric_limits<double>::max();
    }
    Line sym_line() { return Line(((l0_.front() + l1_.front()) * 0.5),
                                  ((l0_.back() + l1_.back()) * 0.5)); }
    double max_sym_len()
    {
        return max_sym_len_;
    }

private:
    void add_sym_pt(const Pnt3 &p);
    Pnt3 stPt_;
    std::vector<Pnt3> l0_;
    std::vector<Pnt3> l1_;
    int sym_axis_ = SYM_AXIS_Y;
    Vec3 view_dir_;
    double max_sym_len_ = std::numeric_limits<double>::max();
};