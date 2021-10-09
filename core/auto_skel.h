#pragma once

#include "common.h"
#include "trace_skeleton.h"
//ToDO: seems ZhangSun's thinning algo only works when the digital pattern is 'thin',
//      if the digital pattern (W/H) ratio near 1, the final skeleton will collapse to a 'point'
void cal_medial_axis_from_polygon(const std::vector<Pnt3> &pts, std::vector<std::vector<Pnt3>> &axis);
void cal_skel_from_polygon(const std::vector<Pnt3> &pts, std::vector<std::vector<Pnt3>> &skel, float tol);
// void cal_skel_from_cdtMesh(TMesh, std::vector<std::vector<Pnt3>> &skel, float tol);
