#pragma once

#include "common.h"

class Slice
{
public:
    Slice() : pos_corner_idx_(-1),
              neg_corner_idx_(-1) {}
    Slice(const Pnt3 &cent, const Pnt3 &cut_dir)
        : cent_(cent),
          cut_dir_(cut_dir),
          pos_corner_idx_(-1),
          neg_corner_idx_(-1),
          normal_(cut_dir.y(), -cut_dir.x(), 0)
    {
    }

    void set_cent(const Pnt3 &cent) { cent_ = cent; }
    void set_cut_dir(const Pnt3 &cut_dir)
    {
        cut_dir_ = cut_dir;
        normal_ = Vec3(cut_dir.y(), -cut_dir.x(), 0);
    }
    const Pnt3 &cent() const { return cent_; }
    const Vec3 &cut_dir() const { return cut_dir_; }
    const Vec3 &normal() const { return normal_; }

    void clear();

    void add_point(const Pnt3 &p) { pts_.push_back(p); }
    const Pnt3 &point(size_t i) const { return pts_[i]; }
    size_t points_size() const { return pts_.size(); }
    void add_seg(int i) { seg_ids_.push_back(i); }
    int seg_id(int i) const { return seg_ids_[i]; }

    const Pnt3 &pos_corner() const
    {
        if (pos_corner_idx_ == -1)
            find_corners();
        return pts_[pos_corner_idx_];
    }

    const int &pos_seg() const
    {
        if (pos_corner_idx_ == -1)
            find_corners();
        return seg_ids_[pos_corner_idx_];
    }

    const Pnt3 &neg_corner() const
    {
        if (neg_corner_idx_ == -1)
            find_corners();
        return pts_[neg_corner_idx_];
    }

    const int &neg_seg() const
    {
        if (neg_corner_idx_ == -1)
            find_corners();
        return seg_ids_[neg_corner_idx_];
    }

    double diameter() const
    {
        return (pos_corner() - neg_corner()).norm();
    }

private:
    // p0 is the fathest point along cut direction, p1 is the fathest point along opposite cut direction
    void find_corners() const;

    Pnt3 cent_;
    Vec3 cut_dir_;
    Vec3 normal_;
    std::vector<Pnt3> pts_;    // in 2D, it is intersection between perpendicular line and polygon contour
    std::vector<int> seg_ids_; // segment id of intersections
    mutable int pos_corner_idx_, neg_corner_idx_;
};

std::ostream &operator<<(std::ostream &out, const Slice &s);

void rotate_trans(const Slice &from, Slice &to);