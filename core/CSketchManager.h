#pragma once

#include "CSpline.h"
#include "XFunctions.h"

class CSketchManager
{
public:
    static void ConvertSplineToPolyLines(CSpline &spline, vector<vector<Pnt3>> &sketchPolyLines);
    static void GetPolyLine(vector<vector<Pnt3>> &sketchPolyLines, vector<Pnt3> &polyline);
    static bool DiscretizeAppointedPolyLine(CSpline &spline, vector<Pnt3> &cur_polyline, int cur_index, int discrete_num);
};