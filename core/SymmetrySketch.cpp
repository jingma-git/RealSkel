#include "SymmetrySketch.h"

void SymmetrySketch::add_pt(const Pnt3 &p)
{
    if (l0_.size() == 1)
    {
        stPt_ = l0_.front();
    }
    add_sym_pt(p);
}

void SymmetrySketch::add_sym_pt(const Pnt3 &p)
{
    Pnt3 p_sym = p;
    if (IsZero(view_dir_.x()) && IsZero(view_dir_.y())) // x, y
    {
        if (sym_axis_ == SYM_AXIS_X)
        {
            p_sym.y() = 2 * stPt_.y() - p.y();
        }
        else if (sym_axis_ == SYM_AXIS_Y)
        {
            p_sym.x() = 2 * stPt_.x() - p.x();
        }
    }
    else if (IsZero(view_dir_.y()) && IsZero(view_dir_.z())) // y, z(x)
    {
        if (sym_axis_ == SYM_AXIS_X)
        {
            p_sym.y() = 2 * stPt_.y() - p.y();
        }
        else if (sym_axis_ == SYM_AXIS_Y)
        {
            p_sym.z() = 2 * stPt_.z() - p.z();
        }
    }
    else if (IsZero(view_dir_.x()) && IsZero(view_dir_.z())) // x, z(y)
    {
        if (sym_axis_ == SYM_AXIS_X)
        {
            p_sym.z() = 2 * stPt_.z() - p.z();
        }
        else if (sym_axis_ == SYM_AXIS_Y)
        {
            p_sym.x() = 2 * stPt_.x() - p.x();
        }
    }
    l0_.push_back(p);
    l1_.push_back(p_sym);
    double len = (p - p_sym).norm();
    if (len > max_sym_len_)
        max_sym_len_ = len;
    // M_DEBUG << "add_symPt=" << p.transpose() << " p_sym=" << p_sym.transpose() << endl;
}

void SymmetrySketch::to_polygon(std::vector<Pnt3> &polygon)
{
    int n = static_cast<int>(l0_.size());
    for (int i = 0; i < n; ++i)
    {
        polygon.push_back(l0_[i]);
    }
    for (int i = n - 1; i >= 0; --i)
    {
        polygon.push_back(l1_[i]);
    }
}