#include "polygon_partition.h"
#include "quicksort.h"
#include "CSketchManager.h"

typedef typename PolyPartition::SSBranchT SSBranchT;
using namespace std;

PolyPartition::PolyPartition(const std::vector<Pnt3> &polygon, std::list<SSBranchT> &branches)
    : polygon_(polygon), branches_(branches)
{
    verts_.reserve(polygon_.size());
    int n = static_cast<int>(polygon.size());
    for (int i = 0; i < n; ++i)
    {
        verts_.emplace_back(i, polygon_[i]);
    }
    bbox_ = GetBBox(polygon_);
    diag_len_ = bbox_.diaglen();

   /* label_convexity();
    find_reflex_clusters();
    set_id_sharp_clusters();*/
}

void PolyPartition::partition() // greedy cut, ToDO: make this private
{
    if (branches_.size() == 1)
    {
        process_tt_branch(branches_.front());
        return;
    }
    // make JTBranch come before JJBranch, and make long branch comes first in each partition
    BranchIter it_pivot = std::partition(branches_.begin(), branches_.end(), [](const SSBranchT &b)
                                         { return b.type() == JT_BRANCH; });
    quicksort(branches_.begin(), it_pivot);
    quicksort(it_pivot, branches_.end());

    M_DEBUG << "after sorting:\n";
    int i = 0;
    for (auto it = branches_.begin(); it != branches_.end(); ++it)
    {
        SSBranchT &b = *it;
        M_DEBUG << b << endl;
    }

    // process JTBranch
    for (BranchIter bit = branches_.begin(); bit != it_pivot; ++bit)
    {
        process_jt_branch(*bit);
    }

    // process JJBranch
    for (BranchIter bit = it_pivot; bit != branches_.end(); ++bit)
    {
        if (bit->len() < 60)
        {
            M_DEBUG << *bit << " is too short to be sliced! b.len()=" << bit->len() << endl;
            continue;
        }

        process_jj_branch(*bit);
    }
}

void PolyPartition::process_tt_branch(SSBranchT &b)
{
    // Input
    SSBranchT *bptr = &b;

    // Output
    CSpline &spline = branch_splines_[bptr];
    vector<Pnt3> &dense_samples = branch_densamples_[bptr];
    vector<Pnt3> &uniform_samples = branch_unisamples_[bptr];
    vector<Slice> &slices = branch_slices_[bptr];
    vector<Slice> &vslices = branch_vslices_[bptr];
    vector<int> &segs1st = branch_segs1st_[bptr];
    vector<int> &segs2nd = branch_segs2nd_[bptr];
    vector<Pnt3> &branch_polygon = branch_polygons_[bptr];
    vector<Pnt3> &branch_smthpoly = branch_smthpolys_[bptr];
    vector<int> &branch_silidx = branch_silidxs_[bptr];

    // Algorithm
    // 1. interpolate spline
    spline.ChangeMode(CSpline::SPLMODE_SPLINE);
    for (size_t i = 0; i < b.psize(); ++i)
    {
        spline.AddCtrlPoint(b.p(i));
    }

    // 2. uniformaly sample along continous spline
    sample_spline(spline, dense_samples, uniform_samples);

    // 3. find intersection between perpendicular line (passing each edge's end point on branch)
    //    and the contour
    slice_along_unisamples(uniform_samples, slices);
    cal_vslices(slices, vslices);

    // 4. build polygon enclosing branch
    // find_branch_segs(bptr, segs1st, segs2nd);
    branch_polygon.reserve(polygon_.size());
    branch_smthpoly.reserve(polygon_.size());
    for (size_t i = 0; i < polygon_.size(); ++i)
    {
        branch_polygon.push_back(polygon_[i]);
        branch_silidx.push_back(i);
        branch_smthpoly.push_back(polygon_[i]);
    }
}

void PolyPartition::process_jt_branch(SSBranchT &b)
{
    // Input
    SSBranchT *bptr = &b;

    // Output
    CSpline &spline = branch_splines_[bptr];
    vector<Pnt3> &dense_samples = branch_densamples_[bptr];
    vector<Pnt3> &uniform_samples = branch_unisamples_[bptr];
    vector<Slice> &slices = branch_slices_[bptr];
    vector<Slice> &vslices = branch_vslices_[bptr];
    vector<int> &segs1st = branch_segs1st_[bptr];
    vector<int> &segs2nd = branch_segs2nd_[bptr];
    vector<Pnt3> &branch_polygon = branch_polygons_[bptr];
    vector<int> &branch_silidx = branch_silidxs_[bptr];

    // Algorithm
    // 1. interpolate spline
    spline.ChangeMode(CSpline::SPLMODE_SPLINE);
    size_t st_idx = (b.pnts[0] - b.pnts[1]).norm() / b.len() < 0.25 ? 1 : 0;
    for (size_t i = st_idx; i < b.pnts.size(); ++i)
    {
        spline.AddCtrlPoint(b.pnts[i]);
    }

    // 2. uniformaly sample along continous spline
    sample_spline(spline, dense_samples, uniform_samples);

    // 3. find intersection between perpendicular line (passing each edge's end point on branch)
    //    and the contour
    slice_along_unisamples(uniform_samples, slices);
    cal_vslices(slices, vslices);

    // 4. build polygon enclosing branch
    find_branch_segs(bptr, segs1st, segs2nd);
    create_jtbranch_polygon(bptr, segs1st, segs2nd, branch_polygon, branch_silidx);
    vector<Pnt3> &branch_smthpoly = branch_smthpolys_[bptr];
    create_jtbranch_smthpoly(branch_polygon, branch_smthpoly);
}

void PolyPartition::process_jj_branch(SSBranchT &b)
{
    // Input
    SSBranchT *bptr = &b;

    // Output
    CSpline &spline = branch_splines_[bptr];
    vector<Pnt3> &dense_samples = branch_densamples_[bptr];
    vector<Pnt3> &uniform_samples = branch_unisamples_[bptr];
    vector<Slice> &slices = branch_slices_[bptr];
    vector<int> &segs1st = branch_segs1st_[bptr];
    vector<int> &segs2nd = branch_segs2nd_[bptr];
    vector<Pnt3> &branch_polygon = branch_polygons_[bptr];
    vector<int> &branch_silidx = branch_silidxs_[bptr];

    // Algorithm
    // 1. interpolate spline
    spline.ChangeMode(CSpline::SPLMODE_SPLINE);
    size_t st_idx = (b.p(0) - b.p(1)).norm() / b.len() < 0.25 ? 1 : 0;
    size_t end_idx = (b.lastp() - b.lastp2nd()).norm() / b.len() < 0.25 ? b.psize() - 2 : b.psize() - 1;
    for (size_t i = st_idx; i <= end_idx; ++i)
    {
        spline.AddCtrlPoint(b.pnts[i]);
    }

    // 2. uniformaly sample along continous spline
    sample_spline(spline, dense_samples, uniform_samples);

    // 3. find intersection between perpendicular line (passing each edge's end point on branch)
    //    and the contour
    slice_along_unisamples(uniform_samples, slices);

    // 4. build polygon enclosing branch
    find_branch_segs(bptr, segs1st, segs2nd);
    create_jjbranch_polygon(bptr, segs1st, segs2nd, branch_polygon, branch_silidx);
}

void PolyPartition::sample_spline(CSpline &spline,
                                  std::vector<Pnt3> &dense_samples,
                                  std::vector<Pnt3> &uniform_samples)
{
    vector<vector<Pnt3>> polyLines;
    CSketchManager::ConvertSplineToPolyLines(spline, polyLines);
    CSketchManager::GetPolyLine(polyLines, dense_samples);
    double polyline_len = CalLineLength(dense_samples);
    DiscreteLineByLen(dense_samples, polyline_len, 10, uniform_samples);
}

void PolyPartition::slice_along_unisamples(const std::vector<Pnt3> &uniform_samples,
                                           std::vector<Slice> &slices)
{
    int n = static_cast<int>(uniform_samples.size());
    slices.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const Pnt3 &p0 = uniform_samples[i];
        Vec3 dir;
        if (i == n - 1)
        {
            Vec3 tagent = (p0 - uniform_samples[i - 1]).normalized();
            dir = Vec3(-tagent.y(), tagent.x(), 0);
        }
        else
        {
            Vec3 tagent = (uniform_samples[i + 1] - p0).normalized();
            dir = Vec3(-tagent.y(), tagent.x(), 0);
        }

        // perpendicular line
        // intersections by shooting along dir and -dir
        Slice slice(p0, dir);
        Pnt3 pos = p0 + dir * diag_len_;
        Pnt3 neg = p0 - dir * diag_len_;
        int m = static_cast<int>(polygon_.size());
        for (int j = 0; j < m; ++j)
        {
            const Pnt3 &st = polygon_[j];
            const Pnt3 &end = polygon_[(j + 1) % m];
            if (IsTwoLineIntersectIn2D(st, end, pos, neg))
            {
                Pnt3 psect = GetTwoLineInterPt(st, end, pos, neg);
                slice.add_point(psect);
                slice.add_seg(j);
            }
            else
            {
                int st_sign = (st - p0).dot(dir) > 0 ? 1 : -1;
                int end_sign = (end - p0).dot(dir) > 0 ? 1 : -1;
                Pnt3 st_offset = p0 + st_sign * (st - p0).norm() * dir;
                Pnt3 end_offset = p0 + end_sign * (end - p0).norm() * dir;
                if ((st_offset - st).norm() < 1)
                {
                    slice.add_point(st);
                    slice.add_seg(j);
                }
                else if ((end_offset - end).norm() < 1)
                {
                    slice.add_point(end);
                    slice.add_seg(j);
                }
            }
        }

        // M_DEBUG << "slice" << slices.size() << ": " << slice << endl;
        if (slice.points_size() >= 2) // ToDO: this is just a hacking, need to think a better solution
            slices.push_back(slice);
    }

    // // make sure each slice has at least two intersections with polygon_
    // // ToDo: remove
    // for (int i = 0; i < n; ++i)
    // {
    //     if (slices[i].points_size() < 2)
    //         printf("slice%d point_size()=%d\n", i, static_cast<int>(slices[i].points_size()));
    //     assert(slices[i].points_size() >= 2 && "slice must have at least tow intersections!\n");
    // }
}

void PolyPartition::cal_vslices(const std::vector<Slice> &slices,
                                std::vector<Slice> &vslices)
{
    int n = static_cast<int>(slices.size());
    vslices.resize(n);
    Pnt3 p(0, 0, 0);
    Vec3 dir(1, 0, 0), cut_dir(0, 1, 0);
    vslices[0].set_cent(p);
    vslices[0].set_cut_dir(cut_dir);
    for (int i = 0; i < n - 1; ++i)
    {
        double step = (slices[i + 1].cent() - slices[i].cent()).norm();
        double half_len = 0.5 * (slices[i].pos_corner() - slices[i].neg_corner()).norm();
        vslices[i].add_point(p + half_len * cut_dir);
        vslices[i].add_point(p - half_len * cut_dir);
        // rotate_trans(slices[i], vslices[i]);
        p += step * dir;
        vslices[i + 1].set_cent(p);
        vslices[i + 1].set_cut_dir(cut_dir);
    }
    // rotate_trans(slices[n - 1], vslices[n - 1]);
    double half_len = 0.5 * (slices[n - 1].pos_corner() - slices[n - 1].neg_corner()).norm();
    vslices[n - 1].add_point(p + half_len * cut_dir);
    vslices[n - 1].add_point(p - half_len * cut_dir);
}

void PolyPartition::find_branch_segs(SSBranchT *b,
                                     std::vector<int> &segs1st,
                                     std::vector<int> &segs2nd)
{
    std::vector<Slice> &slices = branch_slices_[b];
    int num_slices = static_cast<int>(slices.size());
    if (slices.size() > 3)
    {
        // 1.  trim front and back slices that deviates
        SliceIter first = slices.begin();
        SliceIter last = slices.end() - 1;
        SliceIter head_delim = slices.begin() + num_slices / 3;
        SliceIter tail_delim = last - num_slices / 3;

        // find the first continous slices position
        while (first != last)
        {
            SliceIter next = first + 1;

            int pos_distance = std::abs(next->pos_seg() - first->pos_seg());
            int neg_distance = std::abs(next->neg_seg() - first->neg_seg());

            if (pos_distance <= 1 && neg_distance <= 1)
            {
                if (first != slices.begin())
                {
                    double ratio = next->diameter() / first->diameter();
                    M_DEBUG << "first != slices.begin() start ratio=" << ratio
                            << " first->diameter=" << first->diameter()
                            << " next->diameter=" << next->diameter() << endl;
                }

                break;
            }
            ++first;
        }

        // double check to prevent there are contious long-start slices
        for (SliceIter it = first; it != last; ++it)
        {
            SliceIter next = it + 1;

            int pos_distance = std::abs(next->pos_seg() - it->pos_seg());
            int neg_distance = std::abs(next->neg_seg() - it->neg_seg());
            double ratio = next->diameter() / it->diameter();

            if ((pos_distance > 1 || neg_distance > 1) && fabs(ratio - 1) > 0.4)
            {
                M_DEBUG << "start ratio=" << ratio
                        << " it->diameter=" << it->diameter()
                        << " next->diameter=" << next->diameter() << endl;

                first = next;
                break;
            }

            if (it == head_delim || next == last)
                break;
        }

        // find last continous slice position
        while (last - 1 != first)
        {
            SliceIter next = last - 1;
            int pos_distance = std::abs(next->pos_seg() - last->pos_seg());
            int neg_distance = std::abs(next->neg_seg() - last->neg_seg());
            if (pos_distance <= 1 && neg_distance <= 1)
            {
                if (last != slices.end() - 1)
                {
                    double ratio = next->diameter() / last->diameter();
                    M_DEBUG << "last != slices.end() end ratio=" << ratio
                            << " last->diameter=" << last->diameter()
                            << " next->diameter=" << next->diameter() << endl;
                }
                break;
            }
            --last;
        }

        // double check to prevent there are contious long-end slices
        for (SliceIter it = last; it - 1 != first; --it)
        {
            SliceIter next = it - 1;
            int pos_distance = std::abs(next->pos_seg() - it->pos_seg());
            int neg_distance = std::abs(next->neg_seg() - it->neg_seg());
            double ratio = next->diameter() / it->diameter();

            if ((pos_distance > 1 || neg_distance > 1) && fabs(ratio - 1) > 0.4)
            {
                M_DEBUG << "end ratio=" << ratio
                        << " it->diameter=" << it->diameter()
                        << " next->diameter=" << next->diameter() << endl;
                last = next;
                break;
            }

            if (it == tail_delim)
                break;
        }

        slices.erase(last + 1, slices.end()); // !!! remove elements on trail first, otherwise will cause vector shuffle
        slices.erase(slices.begin(), first);
    }

    // 2. find contious positive&negative segments
    std::vector<int> pos_segs, neg_segs;
    for (SliceIter it = slices.begin(); it != slices.end(); ++it)
    {
        if (std::find(pos_segs.begin(), pos_segs.end(), it->pos_seg()) == pos_segs.end())
            pos_segs.push_back(it->pos_seg());
        if (std::find(neg_segs.begin(), neg_segs.end(), it->neg_seg()) == neg_segs.end())
            neg_segs.push_back(it->neg_seg());
    }

    M_DEBUG << "pos_segs" << endl;
    std::copy(pos_segs.begin(), pos_segs.end(), std::ostream_iterator<int>(std::cout, " "));
    cout << endl;
    M_DEBUG << "neg_segs" << endl;
    std::copy(neg_segs.begin(), neg_segs.end(), std::ostream_iterator<int>(std::cout, " "));
    M_DEBUG << endl;

    // pos_segs.erase(std::unique(pos_segs.begin(), pos_segs.end()), pos_segs.end());
    // neg_segs.erase(std::unique(neg_segs.begin(), neg_segs.end()), neg_segs.end());
    auto pos_it = std::find(pos_segs.begin(), pos_segs.end(), 0);
    auto neg_it = std::find(neg_segs.begin(), neg_segs.end(), 0);

    if (pos_it != pos_segs.end())
    {
        M_DEBUG << "pos_segs has 0" << endl;
        std::sort(neg_segs.begin(), neg_segs.end());
        std::vector<int>::iterator next = pos_it + 1;
        if (next != pos_segs.end())
            if (*(pos_it + 1) != 1) // 0 should be followed by 1, otherwise, the line is not ccw
            {
                std::reverse(pos_segs.begin(), pos_segs.end());
            }
        segs1st = pos_segs;
        segs2nd = neg_segs;
    }
    else if (neg_it != neg_segs.end() && neg_it != neg_segs.begin())
    {
        M_DEBUG << "neg_segs has 0" << endl;
        std::sort(pos_segs.begin(), pos_segs.end());
        std::vector<int>::iterator next = neg_it + 1;
        if (next != neg_segs.end())
            if (*(neg_it + 1) != 1) // 0 should be followed by 1, otherwise, the line is not ccw
            {
                std::reverse(neg_segs.begin(), neg_segs.end());
            }
        segs1st = neg_segs;
        segs2nd = pos_segs;
    }
    else
    {
        M_DEBUG << "pos and neg segs are normal" << endl;
        std::sort(pos_segs.begin(), pos_segs.end());
        std::sort(neg_segs.begin(), neg_segs.end());
        segs1st = (pos_segs[0] < neg_segs[0] ? pos_segs : neg_segs);
        segs2nd = (pos_segs[0] < neg_segs[0] ? neg_segs : pos_segs);
    }
}

void PolyPartition::connect(const std::vector<int> &segs1st,
                            const std::vector<int> &segs2nd,
                            std::vector<int> &branch_silidx)
{
    for (int i : segs1st)
    {
        branch_silidx.push_back(i);
    }
    for (int i = segs1st.back() + 1; i < segs2nd.front(); ++i)
    {
        branch_silidx.push_back(i);
    }
    for (int i : segs2nd)
    {
        branch_silidx.push_back(i);
    }
}

void PolyPartition::connect(const std::vector<int> &segs1st,
                            const std::vector<int> hole,
                            const std::vector<int> &segs2nd,
                            std::vector<int> &branch_silidx)
{
    for (int i : segs1st)
    {
        branch_silidx.push_back(i);
    }
    for (int i : hole)
    {
        branch_silidx.push_back(i);
    }
    for (int i : segs2nd)
    {
        branch_silidx.push_back(i);
    }
}

void PolyPartition::create_jtbranch_polygon(SSBranchT *b,
                                            std::vector<int> &segs1st,
                                            std::vector<int> &segs2nd,
                                            std::vector<Pnt3> &branch_polygon,
                                            std::vector<int> &branch_silidx)
{
    std::vector<int> lhole, rhole; // left hole, right hole, image smaller points on the lower half of polygon
    branch_polygon.reserve(segs1st.size() + segs2nd.size() + 4);
    branch_silidx.reserve(segs1st.size() + segs2nd.size() + 4);
    auto it = std::find(segs1st.begin(), segs1st.end(), 0);
    const Pnt3 &nodeP = b->st->pos;
    double rhole_dis = (polygon_[segs1st.back()] - nodeP).norm();
    double lhole_dis = (polygon_[segs1st.front()] - nodeP).norm();
    if (it != segs1st.end())
    { // hole doesn't contain 0
        // 23 24 25 26 27 0 1 2 3
        // 13 14 15 16 17 18
        int rhole_size = segs2nd.front() - segs1st.back() - 1;
        int lhole_size = segs1st.front() - segs2nd.back();

        if (rhole_size < lhole_size)
        {
            if (rhole_dis > lhole_dis)
            {
                connect(segs1st, segs2nd, branch_silidx); // common case
            }
            else
            {
                connect(segs2nd, segs1st, branch_silidx);
            }
        }
        else
        {
            if (rhole_dis > lhole_dis)
            {
                connect(segs1st, segs2nd, branch_silidx);
            }
            else
            {
                connect(segs2nd, segs1st, branch_silidx); // common case
            }
        }
        M_DEBUG << "rhole_size=" << rhole_size << " lhole_size=" << lhole_size
                << " rhole_dis=" << rhole_dis << " lhole_dis=" << lhole_dis << endl;
        M_DEBUG << "branch_silidx\n";
        std::copy(branch_silidx.begin(), branch_silidx.end(), std::ostream_iterator<int>(std::cout, " "));
        cout << endl;
    }
    else
    {
        int n = static_cast<int>(polygon_.size());
        int i = segs1st.back() + 1;
        int val = i % n;
        while (val != segs2nd.front())
        {
            rhole.push_back(val);
            val = (++i) % n;
        }

        i = segs2nd.back() + 1;
        val = i % n;
        while (val != segs1st.front())
        {
            lhole.push_back(val);
            val = (++i) % n;
        }

        if (rhole.size() < lhole.size())
        {
            if (rhole_dis > lhole_dis)
            {
                connect(segs1st, rhole, segs2nd, branch_silidx);
            }
            else
            {
                connect(segs2nd, lhole, segs1st, branch_silidx); // common case
            }
        }
        else
        {
            if (rhole_dis > lhole_dis)
            {
                connect(segs1st, rhole, segs2nd, branch_silidx);
            }
            else
            {
                connect(segs2nd, lhole, segs1st, branch_silidx); // common case
            }
        }

        M_DEBUG << "rhole_size=" << rhole.size() << " lhole_size=" << lhole.size()
                << " rhole_dis=" << rhole_dis << " lhole_dis=" << lhole_dis << endl;
        M_DEBUG << "branch_silidx\n";
        std::copy(branch_silidx.begin(), branch_silidx.end(), std::ostream_iterator<int>(std::cout, " "));
        cout << endl;
    }

    // for (int i : branch_silidx)
    // {
    //     branch_polygon.push_back(polygon_[i]);
    // }
    // int end_i = (branch_silidx.back() + 1) % static_cast<int>(polygon_.size());
    // branch_polygon.push_back(polygon_[end_i]);
    // branch_polygon.push_back(nodeP);

    Pnt3 stP, endP;
    const Pnt3 &stPP = polygon_[branch_silidx[0]];
    const Slice &slice = branch_slices_[b].front();
    if ((slice.pos_corner() - stPP).norm() < (slice.neg_corner() - stPP).norm())
    {
        stP = slice.pos_corner();
        endP = slice.neg_corner();
    }
    else
    {
        stP = slice.neg_corner();
        endP = slice.pos_corner();
    }
    branch_polygon.push_back(stP);
    for (size_t i = 1; i < branch_silidx.size(); ++i)
    {
        int id = branch_silidx[i];
        branch_polygon.push_back(polygon_[id]);
    }
    branch_polygon.push_back(endP);
    branch_polygon.push_back(nodeP);
    // if (IsLineSelfIntersect(branch_polygon))
    // {
    //     assert(false && "Oh oops! Polygon self-intersect!\n");
    // }
}

void PolyPartition::create_jtbranch_smthpoly(const std::vector<Pnt3> &branch_poly,
                                             std::vector<Pnt3> &branch_smthpoly)
{
    const std::vector<Pnt3> &pts = branch_poly;
    int n = static_cast<int>(pts.size());
    CSpline spline;
    spline.ChangeMode(CSpline::SPLINE_MODE::SPLMODE_SPLINE);
    spline.AddCtrlPoint(pts[n - 2]);
    spline.AddCtrlPoint(pts[n - 1]);
    spline.AddCtrlPoint(pts[0]);
    std::vector<std::vector<Pnt3>> polyLines;
    std::vector<Pnt3> polyline;
    CSketchManager::ConvertSplineToPolyLines(spline, polyLines);
    CSketchManager::GetPolyLine(polyLines, polyline);

    std::vector<Pnt3> &smth_poly = branch_smthpoly;
    smth_poly.reserve(pts.size() + polyline.size() + 10);

    for (int i = 0; i < n - 1; ++i)
    {
        smth_poly.push_back(pts[i]);
    }
    for (size_t i = 1; i < polyline.size() - 1; ++i)
    {
        smth_poly.push_back(polyline[i]);
    }
}

/*
void PolyPartition::create_jtbranch_polygon(SSBranchT *b,
                                            std::vector<int> &segs1st,
                                            std::vector<int> &segs2nd,
                                            std::vector<Pnt3> &branch_polygon,
                                            std::vector<int> &branch_silidx)
{

    // if (seg1st.back() > segs2nd.front()) //ToDO: remove
    // {
    M_DEBUG << "create_jtbranch_polygon" << endl;
    std::copy(segs1st.begin(), segs1st.end(), std::ostream_iterator<int>(std::cout, " "));
    cout << endl;
    std::copy(segs2nd.begin(), segs2nd.end(), std::ostream_iterator<int>(std::cout, " "));
    cout << endl;
    // assert(segs2nd.front() >= seg1st.back() && "Seg2nd.front() must be greater than Seg1st.back()!");
    // }
    int n = static_cast<int>(segs1st.size() + segs2nd.size());
    branch_polygon.reserve(n + 4);
    branch_silidx.reserve(n + 4);

    bool has_zero_in_filled_range = false;
    if ((segs2nd.front() - segs1st.back()) > n)
    {
        int count = (n - segs2nd.back()) + segs1st.front(); // how many points in between

        if (count < n)
        {
            has_zero_in_filled_range = true;
            std::swap(segs1st, segs2nd);
        }
    }

    // add head to polygon...
    for (int i : segs1st)
    {
        branch_silidx.push_back(i);
    }
    // fill up hole between seg1st and seg2nd
    if (has_zero_in_filled_range)
    {
        for (int i = segs1st.back() + 1; i < static_cast<int>(polygon_.size()); ++i)
        {
            branch_silidx.push_back(i);
        }
        for (int i = 0; i < segs2nd.front(); ++i)
        {
            branch_silidx.push_back(i);
        }
    }
    else
    {
        for (int i = segs1st.back() + 1; i < segs2nd.front(); ++i)
        {
            branch_silidx.push_back(i);
        }
    }
    // add tail to polygon
    for (int i : segs2nd)
    {
        branch_silidx.push_back(i);
    }

    int k = segs2nd.back();
    int i = (k + 1) % static_cast<int>(polygon_.size());
    if (!verts_[i].is_reflex())
    {
        int j = segs1st.front();
        double ratio = (polygon_[i] - polygon_[j]).norm() / (polygon_[k] - polygon_[j]).norm();
        M_DEBUG << "segs1st-segs2nd ratio=" << ratio << endl;
        if (fabs(ratio - 1) < 0.2)
        {
            branch_silidx.push_back(i);
        }
    }

    M_DEBUG << "branch_silidx\n";
    std::copy(branch_silidx.begin(), branch_silidx.end(), std::ostream_iterator<int>(std::cout, " "));
    cout << endl;

    for (int i : branch_silidx)
    {
        branch_polygon.push_back(polygon_[i]);
    }
    branch_polygon.push_back(b->st->pos);

    if (IsLineSelfIntersect(branch_polygon))
    {
        assert(false && "Oh oops! Polygon self-intersect!\n");
    }
}
*/

void PolyPartition::create_jjbranch_polygon(SSBranchT *b,
                                            std::vector<int> &segs1st,
                                            std::vector<int> &segs2nd,
                                            std::vector<Pnt3> &branch_polgyon,
                                            std::vector<int> &branch_silidx)
{
}

void PolyPartition::label_convexity()
{
    int n = static_cast<int>(polygon_.size());
    for (int i = 0; i < n; ++i)
    {
        const Pnt3 &prev = polygon_[(n+i - 1) % n];
        const Pnt3 &cur = polygon_[i];
        const Pnt3 &next = polygon_[(n+i + 1) % n];
        Convexity sign = static_cast<Convexity>(convexity(prev, cur, next));
        double angle = GetAngleOf2Vector(prev - cur, next - cur);
        verts_[i].set_sign(sign);
        verts_[i].set_angle(angle);
    }
}

int PolyPartition::convexity(const Pnt3 &p1, const Pnt3 &p2, const Pnt3 &p3)
{
    // https://www.geeksforgeeks.org/orientation-3-ordered-points/
    double det = (p2.y() - p1.y()) * (p3.x() - p2.x()) - (p2.x() - p1.x()) * (p3.y() - p2.y());

    if (det > 0)
        return REFLEX;
    else if (det == 0)
        return FLAT;
    else
        return CONVEX;
}

void PolyPartition::find_reflex_clusters()
{
    int n = static_cast<int>(verts_.size());
    int num_convex = 0;
    int cluster_id = -1;
    for (int i = 0; i < n; ++i)
    {
        if (verts_[i].is_reflex())
        {
            if (i == 0 && num_convex == 0) // the first vert in polygon is reflex
            {
                ++cluster_id;
                ReflexCluster cluster(cluster_id);
                clusters_.push_back(cluster);
            }

            if (num_convex)
            {
                num_convex = 0;
                ++cluster_id;
                ReflexCluster cluster(cluster_id);
                clusters_.push_back(cluster);
            }

            verts_[i].set_id_reflex_cluster(cluster_id);
            clusters_[cluster_id].add_vert(i);
        }
        else
        {
            ++num_convex;
        }
    }
}

void PolyPartition::set_id_sharp_clusters()
{
    for (auto it = clusters_.begin(); it != clusters_.end(); ++it)
    {
        double min_angle = std::numeric_limits<double>::max();
        for (int i = 0; i < it->vertices.size(); ++i)
        {
            int id = it->vertices[i];
            if (vert_angle(id) < min_angle)
            {
                min_angle = vert_angle(id);
                it->id_sharp = id;
            }
        }
    }
}

void PolyPartition::cut(int i0, int i1, SSBranchT *b)
{
    for (int i = i0; i <= i1; ++i)
    {
        verts_[i].set_branch(b);
    }
    isCut_[b] = true;
}