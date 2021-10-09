#include "part_generator.h"
#include "XFunctions.h"

#include <igl/copyleft/cgal/partition.h>
#include <opencv2/opencv.hpp>

PartGenerator::PartGenerator(const std::vector<Pnt3> &outer_contour_)
    : outer_contour(outer_contour_),
      part_idxs(outer_contour_.size(), -1)
{
}

void PartGenerator::gen_part_contour(std::vector<Pnt3> &part_contour)
{
    partition();
}

void PartGenerator::partition()
{
    //     double line_len = CalLineLength(outer_contour);
    //     std::vector<Pnt3> simp_polyline;
    //     SimplifyPolyLine(sketchLine, simp_polyline, 10);
    //     if ((simp_polyline.back() - simp_polyline.front()).norm() < 0.1)
    //     {
    //         simp_polyline.pop_back();
    //     }

    //     igl::copyleft::cgal::partition(simp_polyline, part_polys);
    //
}

void PartGenerator::gen_smooth_contour(const std::vector<Pnt3> &part_poly,
                                       std::vector<Pnt3> &part_contour)
{
}