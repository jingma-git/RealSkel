#pragma once

#include "common.h"
#include "TMesh.h"

class PartGenerator
{
public:
    PartGenerator(const std::vector<Pnt3> &outer_contour_);
    void gen_part_contour(std::vector<Pnt3> &part_contour);

private:
    void partition();
    void gen_smooth_contour(const std::vector<Pnt3> &part_poly,
                            std::vector<Pnt3> &part_contour);

public:
    const std::vector<Pnt3> &outer_contour;
    std::vector<int> part_idxs;
    std::vector<std::vector<Pnt3>> part_polys;
    std::vector<TMesh> part_meshes;
};