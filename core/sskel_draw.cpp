#include "sskel_draw.h"
// #define PUT_TEXT

static cv::Scalar red(0, 0, 255),
    white(255, 255, 255),
    blue(255, 0, 0),
    lime(0, 255, 0),
    black(0, 0, 0),
    yellow(0, 255, 255),
    saddle_brown(19, 69, 139),
    pink(255, 0, 255),
    cyan(255, 255, 0),
    gray(128, 128, 128),
    purple(128, 0, 128),
    teal(128, 128, 0),
    green(0, 128, 0),
    navy(128, 0, 0),
    maroon(0, 0, 128),
    olive(0, 128, 128),
    indian_red(92, 92, 205),
    orange(165, 0, 255),
    lime_green(50, 205, 50),
    light_sea_green(170, 178, 32),
    blue_vilot(226, 43, 138);

const int IMG_SIZE = 480;

std::vector<cv::Scalar> colors = {
    red,
    lime,
    blue,
    saddle_brown,
    pink,
    cyan,
    gray,
    purple,
    teal,
    green,
    navy,
    maroon,
    olive,
    indian_red,
    orange,
    lime_green,
    light_sea_green,
    blue_vilot};

typedef typename SkelDrawer::SSBranchT SSBranchT;
// assistant functions for drawing pictures for my paper

// the first to call, initialze external variables scl, bbox ect. in this namespace
void CvDrawer::set_sil(const std::vector<Pnt3> &contour)
{
    sil.reserve(contour.size());
    for (const Pnt3 &p : contour)
    {
        sil.push_back(Pnt3(p.x(), -p.y(), 0));
    }

    BBox2 bbox = GetBBox(sil);
    xmin = bbox.xmin;
    ymin = bbox.ymin;
    xmax = bbox.xmax;
    ymax = bbox.ymax;

    int width = xmax - xmin;
    int height = ymax - ymin;
    if (contour.size() > 60)
        img_w *= 2;
    scl = (double)img_w / (double)(width);
    img_h = height * scl;

    for (Pnt3 &p : sil)
    {
        p[0] = (p[0] - xmin) * scl + padding;
        p[1] = (p[1] - ymin) * scl + padding;
    }
}

cv::Point CvDrawer::toCvPoint(const Pnt3 &p)
{
    double x = (p.x() - xmin) * scl + padding;
    double y = (-p.y() - ymin) * scl + padding;
    return cv::Point(x, y);
}

cv::Mat CvDrawer::draw_sil()
{
    cv::Mat img(img_h + padding * 2, img_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));

    int n = static_cast<int>(sil.size());
    std::vector<cv::Point> pts;
    pts.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        pts.push_back(cv::Point(sil[i].x(), sil[i].y()));
    }

    for (int i = 0; i < n; ++i)
    {
        cv::line(img, pts[i], pts[(i + 1) % n], black, 1);
        cv::circle(img, pts[i], 3, red, -1);
#ifdef PUT_TEXT
        cv::putText(img, std::to_string(i), (pts[i] + pts[(i + 1) % n]) * 0.5,
                    cv::FONT_HERSHEY_SIMPLEX, 1, red);
#endif
    }
    sil_img = img;
    return img;
}

void CvDrawer::draw_points(cv::Mat &img,
                           const std::vector<Pnt3> &pts,
                           const cv::Scalar &color,
                           double radius)
{
    for (const Pnt3 &p : pts)
    {
        cv::Point cvP = toCvPoint(p);
        cv::circle(img, cvP, radius, color, -1);
        // cv::putText(img, std::to_string(n->idx), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, white);
    }
}

void CvDrawer::draw_line(cv::Mat &img,
                         const std::vector<Pnt3> &pts,
                         const cv::Scalar &color,
                         int line_width)
{
    for (size_t i = 0; i < pts.size() - 1; ++i)
    {
        const Pnt3 &p0 = pts[i];
        const Pnt3 &p1 = pts[i + 1];
        cv::Point cvP0 = toCvPoint(p0);
        cv::Point cvP1 = toCvPoint(p1);

        cv::line(img, cvP0, cvP1, color, line_width);
    }
}

void SkelDrawer::set_bcolors(std::list<SSBranchT> &branches)
{
    int i = 0;
    for (auto it = branches.begin(); it != branches.end(); ++it)
    {
        SSBranchT &b = *it;
        bcolors[&b] = colors[i++];
    }
}

void SkelDrawer::set_branches(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &bsmthpolys,
                              std::unordered_map<SSBranchT *, std::vector<Pnt3>> &uni_lines,
                              std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices)
{
    for (auto it = bsmthpolys.begin(); it != bsmthpolys.end(); ++it)
    {
        SSBranchT *b = it->first;
        bimgs[b] = BImage(bsmthpolys[b], uni_lines[b]);
        bimgs[b].draw_sil();
        bimgs[b].draw_axis();
        bimgs[b].draw_slices(branch_slices[b], bcolors[b]);
    }
}

cv::Mat SkelDrawer::draw_sketch_line(const std::vector<Pnt3> &pts)
{
    cv::Mat img(img_h + padding * 2, img_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));
    draw_line(img, pts, black, 1);
    return img;
}

cv::Mat SkelDrawer::draw_straight_skeleton(SsPtr ss,
                                           bool only_skeleton,
                                           cv::Mat *bg_img)
{
    typedef typename SSkeletor::Point_2 Point;
    typedef typename SSkeletor::Vertex_const_handle Vertex_const_handle;
    typedef typename SSkeletor::Halfedge_const_handle Halfedge_const_handle;
    typedef typename SSkeletor::Halfedge_const_iterator Halfedge_const_iterator;
    cv::Mat img(img_h + padding * 2, img_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));
    if (bg_img)
        img = bg_img->clone();
    cv::Scalar color;

    for (Halfedge_const_iterator h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
    {
        Vertex_const_handle v0 = h->vertex();
        Vertex_const_handle v1 = h->opposite()->vertex();
        Point p0 = v0->point();
        Point p1 = v1->point();
        p0 = Point((p0.x() - xmin) * scl + padding, (-p0.y() - ymin) * scl + padding);
        p1 = Point((p1.x() - xmin) * scl + padding, (-p1.y() - ymin) * scl + padding);
        cv::Point cp0(p0.x(), p0.y());
        cv::Point cp1(p1.x(), p1.y());

        if (h->is_bisector() && h->id() % 2 == 0 && !h->has_infinite_time() && !h->opposite()->has_infinite_time())
        {
            // cout << "bisector " << h->id()
            //      << " v" << v0->id() << ": " << p0 << " isborder=" << v0->is_contour()
            //      << " v" << v1->id() << ": " << p1 << " isborder=" << v1->is_contour() << endl;
            if (v0->is_skeleton() && v1->is_skeleton())
                color = red;
            else
                color = green;

            if (only_skeleton)
            {
                if (v0->is_skeleton() && v1->is_skeleton())
                {
                    cv::line(img, cp0, cp1, color, 2);
                }
            }
            else
            {
                if (v0->is_skeleton() && v1->is_skeleton())
                {
                    cv::line(img, cp0, cp1, color, 2);
                }
                else
                {
                    cv::line(img, cp0, cp1, color, 1);
                }
            }

            cv::circle(img, cp0, 3, blue, -1);
        }

        if (h->is_border())
        {
            cv::line(img, cp0, cp1, black, 1);
            cv::circle(img, cp0, 3, red, -1);
        }
    }

    for (Halfedge_const_iterator h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
    {
        Vertex_const_handle v0 = h->vertex();
        Vertex_const_handle v1 = h->opposite()->vertex();
        Point p0 = v0->point();
        Point p1 = v1->point();
        p0 = Point((p0.x() - xmin) * scl + padding, (-p0.y() - ymin) * scl + padding);
        p1 = Point((p1.x() - xmin) * scl + padding, (-p1.y() - ymin) * scl + padding);
        cv::Point cp0(p0.x(), p0.y());
        cv::Point cp1(p1.x(), p1.y());

        if (h->is_bisector() && h->id() % 2 == 0 && !h->has_infinite_time() && !h->opposite()->has_infinite_time())
        {
            cv::circle(img, cp0, 3, blue, -1);
        }

        if (h->is_border())
        {
            cv::circle(img, cp0, 3, red, -1);
        }
    }

    return img;
}

void SkelDrawer::draw_defining_contour(SsPtr ss,
                                       cv::Mat *bg_img)
{
    typedef typename SSkeletor::Point_2 Point;
    typedef typename SSkeletor::Vertex_const_handle Vertex_const_handle;
    typedef typename SSkeletor::Halfedge_const_handle Halfedge_const_handle;
    typedef typename SSkeletor::Halfedge_const_iterator Halfedge_const_iterator;

    cv::Scalar color;
    int i = 0;
    for (Halfedge_const_iterator h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
    {
        Vertex_const_handle v0 = h->vertex();
        Vertex_const_handle v1 = h->opposite()->vertex();
        Point p0 = v0->point();
        Point p1 = v1->point();
        p0 = Point((p0.x() - xmin) * scl + padding, (-p0.y() - ymin) * scl + padding);
        p1 = Point((p1.x() - xmin) * scl + padding, (-p1.y() - ymin) * scl + padding);
        cv::Point cp0(p0.x(), p0.y());
        cv::Point cp1(p1.x(), p1.y());

        if (h->is_bisector() && h->id() % 2 == 0 && !h->has_infinite_time() && !h->opposite()->has_infinite_time())
        {
            if (v0->is_skeleton() && v1->is_skeleton())
            {
                cv::Mat img = bg_img->clone();
                cv::circle(img, cp0, 3, cyan, -1);
                // defining contours
                auto hit = v0->defining_contour_halfedges_begin();
                auto be_it = v0->defining_contour_halfedges_begin();

                do
                {
                    Vertex_const_handle def_v0 = (*hit)->vertex();
                    Vertex_const_handle def_v1 = (*hit)->opposite()->vertex();

                    Point def_p0 = def_v0->point();
                    Point def_p1 = def_v1->point();
                    def_p0 = Point((def_p0.x() - xmin) * scl + padding, (-def_p0.y() - ymin) * scl + padding);
                    def_p1 = Point((def_p1.x() - xmin) * scl + padding, (-def_p1.y() - ymin) * scl + padding);
                    cv::Point def_cp0(def_p0.x(), def_p0.y());
                    cv::Point def_cp1(def_p1.x(), def_p1.y());
                    cv::line(img, def_cp0, def_cp1, pink, 2);
                    ++hit;
                } while (hit != be_it);

                Vertex_const_handle def_v0 = (*hit)->vertex();
                Point def_p0 = def_v0->point();
                def_p0 = Point((def_p0.x() - xmin) * scl + padding, (-def_p0.y() - ymin) * scl + padding);
                cv::Point def_cp0(def_p0.x(), def_p0.y());
                cv::circle(img, def_cp0, 5, lime, -1);
                cv::imwrite("output/define_" + std::to_string(i++) + ".png", img);
            }
        }
    }
}

cv::Mat SkelDrawer::draw_sskel(const SSkel<SSNode> &sskel, cv::Mat *bg_img)
{
    cv::Mat img;
    if (bg_img)
        img = bg_img->clone();
    else
        img = sil_img.clone();

    for (EdgeT *e : sskel.edges)
    {
        cv::Point p0 = toCvPoint(e->node0->pos);
        cv::Point p1 = toCvPoint(e->node1->pos);
        cv::line(img, p0, p1, red, 2);
    }

    for (SSNode *n : sskel.nodes)
    {
        cv::Point p = toCvPoint(n->pos);
        cv::circle(img, p, 3, blue, -1);
    }

    return img;
}

cv::Mat SkelDrawer::draw_splines(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &lines,
                                 cv::Mat bg_img)
{
    cv::Mat img = bg_img.clone();

    for (auto it = lines.begin(); it != lines.end(); ++it)
    {
        draw_line(img, it->second, bcolors[it->first], 2);
    }
    return img;
}

cv::Mat SkelDrawer::draw_uniform_samples(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &lines,
                                         cv::Mat bg_img)
{
    cv::Mat img = bg_img.clone();
    for (auto it = lines.begin(); it != lines.end(); ++it)
    {
        draw_points(img, it->second, bcolors[it->first]);
    }
    return img;
}

void SkelDrawer::draw_one_branch_slices(cv::Mat &img,
                                        const std::vector<Slice> &slices,
                                        const cv::Scalar &color)
{
    int n = static_cast<int>(slices.size());
    for (int i = 0; i < n; ++i)
    {
        const Slice &s = slices[i];

        const Pnt3 &p = s.cent();
        const Pnt3 &p0 = s.pos_corner();
        const Pnt3 &p1 = s.neg_corner();
        Pnt3 cent = (p0 + p1) / 2;

        cv::Point cvP = toCvPoint(p);
        cv::Point cvP0 = toCvPoint(p0);
        cv::Point cvP1 = toCvPoint(p1);
        cv::Point cvCent = toCvPoint(cent);

        // cv::circle(img, cvP, 1, red, 1, cv::LINE_4);
        cv::line(img, cvP0, cvP1, color, 1);
        cv::circle(img, cvCent, 1, color, 1, cv::LINE_4);

        //cv::putText(img, std::to_string(n->idx), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, white);
    }
}

static cv::Point cal_cvP(const Pnt3 &p, int xmin, int ymin, double scale, int padding)
{
    double x = (p.x() - xmin) * scale + padding;
    double y = (-p.y() - ymin) * scale + padding;
    return cv::Point(x, y);
}

void SkelDrawer::draw_one_branch_vslices(cv::Mat &img,
                                         const std::vector<Slice> &slices,
                                         const cv::Scalar &color)
{
    int n = static_cast<int>(slices.size());
    double xmin, ymin, xmax, ymax;
    cal_bbox(slices, xmin, ymin, xmax, ymax);

    int width = xmax - xmin;
    int height = ymax - ymin;
    int image_w = img_w;
    double scale = (double)img_w / (double)(width);
    double image_h = height * scale;

    img = cv::Mat(image_h + padding * 2, image_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < n; ++i)
    {
        const Slice &s = slices[i];
        const Pnt3 &cent = s.cent();
        const Pnt3 &p0 = s.pos_corner();
        const Pnt3 &p1 = s.neg_corner();
        cv::Point cvCent = cal_cvP(cent, xmin, ymin, scale, padding);
        cv::Point cvP0 = cal_cvP(p0, xmin, ymin, scale, padding);
        cv::Point cvP1 = cal_cvP(p1, xmin, ymin, scale, padding);
        cv::circle(img, cvCent, 3, color, -1);
        cv::line(img, cvP0, cvP1, color, 1);
    }
}

void SkelDrawer::cal_bbox(const std::vector<Slice> &slices,
                          double &xmin,
                          double &ymin,
                          double &xmax,
                          double &ymax)
{
    xmin = ymin = std::numeric_limits<double>::max();
    xmax = ymax = std::numeric_limits<double>::min();

    auto find_bbox = [&](const Pnt3 &p)
    {
        if (p.x() < xmin)
            xmin = p.x();
        if (p.x() > xmax)
            xmax = p.x();
        if (-p.y() < ymin)
            ymin = -p.y();
        if (-p.y() > ymax)
            ymax = -p.y();
    };
    for (const Slice &s : slices)
    {
        find_bbox(s.pos_corner());
        find_bbox(s.neg_corner());
    }
}

cv::Mat SkelDrawer::draw_branch_slices(std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices,
                                       cv::Mat bg_img)
{
    cv::Mat img = bg_img.clone();
    for (auto it = branch_slices.begin(); it != branch_slices.end(); ++it)
    {
        draw_one_branch_slices(img, it->second, bcolors[it->first]);
    }
    return img;
}

std::unordered_map<SSBranchT *, cv::Mat> &SkelDrawer::draw_branch_vslices(std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices)
{
    M_DEBUG << "branch_vslices=" << branch_slices.size() << endl;
    for (auto it = branch_slices.begin(); it != branch_slices.end(); ++it)
    {
        SSBranchT *b = it->first;
        std::vector<Slice> &slices = it->second;
        draw_one_branch_vslices(bvslice_imgs[b], slices, bcolors[b]);
    }
    return bvslice_imgs;
}

void SkelDrawer::draw_one_branch_polygon(cv::Mat &img,
                                         const std::vector<Pnt3> &pts,
                                         const std::vector<int> &idxs,
                                         const cv::Scalar &color)
{
    int n = static_cast<int>(pts.size());
    for (int i = 0; i < n; ++i)
    {
        const Pnt3 &p0 = pts[i];
        const Pnt3 &p1 = pts[(i + 1) % n];
        cv::Point cvP0 = toCvPoint(p0);
        cv::Point cvP1 = toCvPoint(p1);
        cv::circle(img, cvP0, 3, color, -1);
        cv::line(img, cvP0, cvP1, color, 2);
#ifdef PUT_TEXT
        if (i < static_cast<int>(idxs.size()))
            cv::putText(img, std::to_string(idxs[i]), cvP0, cv::FONT_HERSHEY_PLAIN, 1, color);
#endif
    }
}

cv::Mat SkelDrawer::draw_branch_polygons(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_polygons,
                                         std::unordered_map<SSBranchT *, std::vector<int>> &branch_polyidxs,
                                         cv::Mat *bg_img)
{
    cv::Mat img(img_h + padding * 2, img_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));
    if (bg_img)
        img = bg_img->clone();
    for (auto it = branch_polygons.begin(); it != branch_polygons.end(); ++it)
    {
        SSBranchT *b = it->first;
        draw_one_branch_polygon(img, it->second, branch_polyidxs[b], bcolors[b]);
    }
    return img;
}

cv::Mat SkelDrawer::draw_branch_smthpolys(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_smthpolys,
                                          cv::Mat *bg_img)
{
    cv::Mat img(img_h + padding * 2, img_w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));
    if (bg_img)
        img = bg_img->clone();
    for (auto it = branch_smthpolys.begin(); it != branch_smthpolys.end(); ++it)
    {
        SSBranchT *b = it->first;
        draw_one_branch_smthpoly(img, it->second, bcolors[b]);
    }

    return img;
}

void SkelDrawer::draw_one_branch_smthpoly(cv::Mat &img,
                                          const std::vector<Pnt3> &pts,
                                          const cv::Scalar &color)
{
    std::vector<std::vector<cv::Point>> cvPolys;
    cvPolys.resize(1);
    cvPolys[0].reserve(pts.size());
    for (const Pnt3 &p : pts)
    {
        cvPolys[0].push_back(toCvPoint(p));
    }
    cv::Mat img_copy = sil_img.clone();
    cv::fillPoly(img_copy, cvPolys, cv::Scalar(color[0], color[1], color[2], 0.3));
    cv::addWeighted(img_copy, 0.2, img, 0.8, 0, img);
}

cv::Mat SkelDrawer::draw_one_branch_vslices_dp(SSBranchT *b,
                                               int iter,
                                               const std::vector<Slice> &slices,
                                               int maxi,
                                               int stIdx,
                                               int endIdx)
{
    double xmin, ymin, xmax, ymax;
    cal_bbox(slices, xmin, ymin, xmax, ymax);
    double scale = (double)img_w / (double)(xmax - xmin);
    cv::Scalar color(blue);

    cv::Mat img;
    draw_one_branch_vslices(img, slices, gray);

    cv::Mat trapozoid = img.clone();
    cv::Point stP0 = cal_cvP(slices[stIdx].pos_corner(), xmin, ymin, scale, padding);
    cv::Point stP1 = cal_cvP(slices[stIdx].neg_corner(), xmin, ymin, scale, padding);
    cv::Point enP0 = cal_cvP(slices[endIdx].pos_corner(), xmin, ymin, scale, padding);
    cv::Point enP1 = cal_cvP(slices[endIdx].neg_corner(), xmin, ymin, scale, padding);
    std::vector<std::vector<cv::Point>> polys;
    polys.resize(1);
    polys[0].push_back(stP0);
    polys[0].push_back(stP1);
    polys[0].push_back(enP1);
    polys[0].push_back(enP0);
    cv::fillPoly(trapozoid, polys, gray);
    cv::addWeighted(trapozoid, 0.2, img, 0.8, 0, img);
    std::vector<int> dp_idxs = bimgs[b].dp_idxs;
    for (size_t i = 0; i < dp_idxs.size() - 1; ++i)
    {
        int i0 = dp_idxs[i];
        int i1 = dp_idxs[i + 1];
        cv::Point i0St = cal_cvP(slices[i0].pos_corner(), xmin, ymin, scale, padding);
        cv::Point i0En = cal_cvP(slices[i0].neg_corner(), xmin, ymin, scale, padding);
        cv::Point i1St = cal_cvP(slices[i1].pos_corner(), xmin, ymin, scale, padding);
        cv::Point i1En = cal_cvP(slices[i1].neg_corner(), xmin, ymin, scale, padding);
        cv::line(img, i0St, i1St, red, 2);
        cv::line(img, i0En, i1En, red, 2);
    }

    int i = maxi;
    const Slice &s = slices[i];
    const Pnt3 &cent = s.cent();
    const Pnt3 &p0 = s.pos_corner();
    const Pnt3 &p1 = s.neg_corner();
    cv::Point cvCent = cal_cvP(cent, xmin, ymin, scale, padding);
    cv::Point cvP0 = cal_cvP(p0, xmin, ymin, scale, padding);
    cv::Point cvP1 = cal_cvP(p1, xmin, ymin, scale, padding);
    cv::circle(img, cvCent, 5, color, -1);
    cv::line(img, cvP0, cvP1, color, 2);

    return img;
}

cv::Mat SkelDrawer::draw_one_branch_dp(SSBranchT *b,
                                       int iter,
                                       int maxi,
                                       int stIdx,
                                       int endIdx)
{
    return bimgs[b].draw_dp(maxi, stIdx, endIdx);
}

BImage::BImage(const std::vector<Pnt3> &smthpoly,
               const std::vector<Pnt3> &unisample)
{
    xmin = ymin = std::numeric_limits<double>::max();
    xmax = ymax = std::numeric_limits<double>::min();
    for (const Pnt3 &p : smthpoly)
    {
        double x = p.x();
        double y = -p.y();
        if (x < xmin)
        {
            xmin = x;
        }
        if (y < ymin)
        {
            ymin = y;
        }
        if (x > xmax)
        {
            xmax = x;
        }
        if (y > ymax)
        {
            ymax = y;
        }
    }

    double width = xmax - xmin;
    double height = ymax - ymin;
    scl = width > height ? (double)IMG_SIZE / width : (double)IMG_SIZE / height;
    img_w = width * scl;
    img_h = height * scl;

    sil.reserve(smthpoly.size());
    for (const Pnt3 &p : smthpoly)
    {
        sil.push_back(toCvPoint(p));
    }

    axis.reserve(unisample.size());
    for (const Pnt3 &p : unisample)
    {
        axis.push_back(toCvPoint(p));
    }

    img = cv::Mat(img_h + padding * 2, img_w + padding * 2, CV_8UC3, white);
}

void BImage::draw_sil(cv::Scalar color, int line_width)
{
    int n = static_cast<int>(sil.size());
    for (int i = 0; i < n; ++i)
    {
        cv::line(img, sil[i], sil[(i + 1) % n], color, line_width);
    }
}

void BImage::draw_axis(cv::Scalar color)
{
    int n = static_cast<int>(axis.size());
    for (int i = 0; i < n - 1; ++i)
    {
        cv::line(img, axis[i], axis[i + 1], color, 1);
    }
}

cv::Mat BImage::draw_dp(int maxi, int stIdx, int endIdx)
{
    std::vector<int> new_idxs({stIdx, maxi, endIdx});
    for (int i : new_idxs)
    {
        if (std::find(dp_idxs.begin(), dp_idxs.end(), i) == dp_idxs.end())
        {
            dp_idxs.push_back(i);
        }
    }
    std::sort(dp_idxs.begin(), dp_idxs.end());

    cv::Point &st = axis[stIdx];
    cv::Point &end = axis[endIdx];

    cv::Mat dp_img = img.clone();

    // line
    cv::line(dp_img, st, end, gray, 1);
    for (size_t i = 0; i < dp_idxs.size() - 1; ++i)
    {
        int i0 = dp_idxs[i];
        int i1 = dp_idxs[i + 1];
        cv::line(dp_img, axis[i0], axis[i1], red, 2);
    }
    // cirlces
    for (size_t i = 0; i < dp_idxs.size(); ++i)
    {
        int i0 = dp_idxs[i];
        cv::circle(dp_img, axis[i0], 5, blue, -1);
    }
    return dp_img;
}

void BImage::draw_slices(const std::vector<Slice> &slices, const cv::Scalar &color)
{
    slice_img = img.clone();
    cv::Mat poly_img = img.clone();
    std::vector<std::vector<cv::Point>> polys;
    polys.push_back(sil);
    cv::fillPoly(poly_img, polys, color);
    cv::addWeighted(poly_img, 0.2, slice_img, 0.8, 0, slice_img);

    int n = static_cast<int>(slices.size());
    for (int i = 0; i < n; ++i)
    {
        const Slice &s = slices[i];

        const Pnt3 &p = s.cent();
        const Pnt3 &p0 = s.pos_corner();
        const Pnt3 &p1 = s.neg_corner();
        Pnt3 cent = (p0 + p1) / 2;

        cv::Point cvP = toCvPoint(p);
        cv::Point cvP0 = toCvPoint(p0);
        cv::Point cvP1 = toCvPoint(p1);
        cv::Point cvCent = toCvPoint(cent);

        // cv::circle(img, cvP, 1, red, 1, cv::LINE_4);
        cv::line(slice_img, cvP0, cvP1, color, 1);
        // cv::circle(slice_img, cvCent, 1, color, 1, cv::LINE_4);

        cv::line(img, cvP0, cvP1, gray, 1);
        // cv::circle(img, cvCent, 1, gray, 1, cv::LINE_4);

        //cv::putText(img, std::to_string(n->idx), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, white);
    }
}