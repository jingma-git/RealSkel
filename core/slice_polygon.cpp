#include "slice_polygon.h"

void Slice::clear()
{
    pts_.clear();
    pos_corner_idx_ = -1;
    neg_corner_idx_ = -1;
}

void Slice::find_corners() const
{
    int n = static_cast<int>(pts_.size());
    if (n == 1)
    {
        Vec3 dir = pts_[0] - cent_;

        pos_corner_idx_ = 0;
        neg_corner_idx_ = 0;
    }
    else if (n == 2)
    {
        Vec3 dir = pts_[0] - cent_;
        if (dir.dot(cut_dir_) > 0)
        {
            pos_corner_idx_ = 0;
            neg_corner_idx_ = 1;
        }
        else
        {
            pos_corner_idx_ = 1;
            neg_corner_idx_ = 0;
        }
    }
    else
    {
        int i0, i1;
        i0 = i1 = -1;
        double dis0, dis1;
        dis0 = dis1 = std::numeric_limits<double>::max();
        for (int i = 0; i < n; ++i)
        {
            Vec3 vpc = pts_[i] - cent_;
            double dis = vpc.norm();
            if (vpc.dot(cut_dir_) > 0)
            {
                if (dis < dis0)
                {
                    dis0 = dis;
                    i0 = i;
                }
            }
            else
            {
                if (dis < dis1)
                {
                    dis1 = dis;
                    i1 = i;
                }
            }
        }

        pos_corner_idx_ = i0;
        neg_corner_idx_ = i1;
    }
}

std::ostream &operator<<(std::ostream &out, const Slice &s)
{
    out << "cent=" << s.cent().transpose() << ", pts=" << s.points_size() << "\n";
    for (size_t i = 0; i < s.points_size(); ++i)
    {
        out << "(" << s.point(i).transpose() << ") ";
    }
    out << "\n\n";
    return out;
}

void rotate_trans(const Slice &from, Slice &to)
{
    using namespace Eigen;
    double cosa = (from.cut_dir()).dot(to.cut_dir());
    double sina = std::sqrt(1. - cosa * cosa);
    Matrix3d R;
    R << cosa, sina, 0,
        -sina, cosa, 0,
        0, 0, 1;

    const Pnt3 from_cent = (from.pos_corner() + from.neg_corner()) * 0.5;
    Vec3 trans = to.cent() - from_cent;
    const Pnt3 p0 = (from.pos_corner() - from_cent); // move to origin
    const Pnt3 p1 = (from.neg_corner() - from_cent);
    to.add_point(R * p0 + from_cent + trans);
    to.add_point(R * p1 + from_cent + trans);
}