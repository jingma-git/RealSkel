#pragma once
#include "polygon_partition.h"
#include <opencv2/opencv.hpp>
// assistant functions for drawing pictures for my paper

struct BImage // branch image
{
public:
    BImage() {}
    BImage(const std::vector<Pnt3> &smthpoly,
           const std::vector<Pnt3> &unisample);

    cv::Point toCvPoint(const Pnt3 &p)
    {
        double x = (p.x() - xmin) * scl + padding;
        double y = (-p.y() - ymin) * scl + padding;
        return cv::Point(x, y);
    }
    void draw_sil(cv::Scalar color = cv::Scalar(0, 0, 0), int line_width = 1);
    void draw_axis(cv::Scalar color = cv::Scalar(0, 0, 0));
    cv::Mat draw_dp(int maxi, int stIdx, int endIdx);
    void draw_slices(const std::vector<Slice> &slices, const cv::Scalar &color);

    int img_w;
    int img_h;
    double scl;
    int padding = 20;

    double xmin, ymin, xmax, ymax;
    cv::Mat img;
    cv::Mat slice_img;

    std::vector<cv::Point> sil;
    std::vector<cv::Point> axis;
    std::vector<int> dp_idxs;
};

class CvDrawer
{
public:
    // the first to call, initialze external variables scl, bbox ect. in this namespace
    void set_sil(const std::vector<Pnt3> &contour);
    // the untility function based on scl, bbox to transform gl_coordinate to cv_coordindate
    cv::Point toCvPoint(const Pnt3 &p);
    cv::Mat draw_sil();

    void draw_points(cv::Mat &img,
                     const std::vector<Pnt3> &pts,
                     const cv::Scalar &color,
                     double radius = 1);
    void draw_line(cv::Mat &img,
                   const std::vector<Pnt3> &pts,
                   const cv::Scalar &color,
                   int line_width = 1);

public:
    int img_w = 480;
    int img_h;
    double scl;
    const int padding = 20;

    double xmin, ymin, xmax, ymax;
    std::vector<Pnt3> sil;
    cv::Mat sil_img;
};

class SkelDrawer : public CvDrawer
{
public:
    typedef SSEdge<SSNode> EdgeT;
    typedef typename SSkeletor::SsPtr SsPtr;
    typedef typename PolyPartition::SSBranchT SSBranchT;

    // must be called if you want to visualize branch-related function in a ordered manner
    void set_bcolors(std::list<SSBranchT> &ordered_branches);
    void set_branches(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_smthpolys,
                      std::unordered_map<SSBranchT *, std::vector<Pnt3>> &uni_lines,
                      std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices);
    cv::Mat &bimg(SSBranchT *b) { return bimgs[b].img; }

    //----------drawing----------------------------------
    cv::Mat draw_sketch_line(const std::vector<Pnt3> &pts);

    cv::Mat draw_straight_skeleton(SsPtr ss, bool only_skeleton, cv::Mat *bg_img = nullptr);

    void draw_defining_contour(SsPtr ss, cv::Mat *bg_img = nullptr);

    cv::Mat draw_sskel(const SSkel<SSNode> &sskel, cv::Mat *bg_img = nullptr);

    cv::Mat draw_splines(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &splines,
                         cv::Mat bg_img);

    cv::Mat draw_uniform_samples(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &uni_lines,
                                 cv::Mat bg_img);

    cv::Mat draw_branch_slices(std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices,
                               cv::Mat bg_img);

    std::unordered_map<SSBranchT *, cv::Mat> &draw_branch_vslices(std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices);

    cv::Mat draw_branch_polygons(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_polygons,
                                 std::unordered_map<SSBranchT *, std::vector<int>> &branch_polyidxs,
                                 cv::Mat *bg_img = nullptr);

    cv::Mat draw_branch_smthpolys(std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_smthpolys,
                                  cv::Mat *bg_img = nullptr);

    cv::Mat draw_one_branch_vslices_dp(SSBranchT *b,
                                       int iter,
                                       const std::vector<Slice> &slices,
                                       int maxi,
                                       int stIdx,
                                       int endIdx);

    cv::Mat draw_one_branch_dp(SSBranchT *b,
                               int iter,
                               int maxi,
                               int stIdx,
                               int endIdx);

private:
    void draw_one_branch_slices(cv::Mat &img,
                                const std::vector<Slice> &slices,
                                const cv::Scalar &color);
    void draw_one_branch_vslices(cv::Mat &img,
                                 const std::vector<Slice> &slices,
                                 const cv::Scalar &color);
    void cal_bbox(const std::vector<Slice> &slices,
                  double &xmin,
                  double &ymin,
                  double &xmax,
                  double &ymax);
    void draw_one_branch_polygon(cv::Mat &img,
                                 const std::vector<Pnt3> &pts,
                                 const std::vector<int> &idxs,
                                 const cv::Scalar &color);

    void draw_one_branch_smthpoly(cv::Mat &img,
                                  const std::vector<Pnt3> &pts,
                                  const cv::Scalar &color);

public:
    std::unordered_map<SSBranchT *, cv::Mat> bvslice_imgs;
    std::unordered_map<SSBranchT *, cv::Scalar> bcolors;
    std::unordered_map<SSBranchT *, BImage> bimgs;
};