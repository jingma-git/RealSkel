#include "CSketchManager.h"

void CSketchManager::ConvertSplineToPolyLines(CSpline &spline, vector<vector<Pnt3>> &sketchPolyLines)
{
    sketchPolyLines.clear();
    int cptCount = spline.GetCtrlPointCount();

    if (cptCount < 2)
        return;
    spline.m_bSplInitiated = false;

    int i, j;
    if (cptCount == 2)
    {
        vector<Pnt3> polyline;
        polyline.push_back(spline.GetCtrlPoint(0));
        polyline.push_back(spline.GetCtrlPoint(1));
        sketchPolyLines.push_back(polyline);
    }
    else
    {
        int clineCount = cptCount - 1;
        if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
        {
            clineCount = cptCount;
        }
        Float Len = 0.0;
        for (i = 0; i < clineCount; i++)
        {
            Len += (spline.GetCtrlPoint(i) - spline.GetCtrlPoint((i + 1) % cptCount)).norm();
        }
        for (i = 0; i < clineCount; i++)
        {

            Float clineLength = (spline.GetCtrlPoint(i) - spline.GetCtrlPoint((i + 1) % cptCount)).norm();
            int divCount = (int)(clineLength / Len * clineCount * 10 + 3); // insert more points

            // ------------------------------------------compute tension-------------------------------------------------------
            Float curLength = (spline.GetCtrlPoint(i) - spline.GetCtrlPoint((i + 1) % cptCount)).norm();
            Float ratio = 1.0;
            // ------------------tension 1------------------------
            // if length_raitio is larger, the tension is smaller
            // compared with next segment
            if (i < clineCount - 1)
            {
                Float nextLength = (spline.GetCtrlPoint((i + 1) % cptCount) - spline.GetCtrlPoint((i + 2) % cptCount)).norm();
                Float ratioTemp = curLength / nextLength;
                if (ratioTemp < 0.9)
                {
                    ratio = ratioTemp;
                }
            }
            else
            {
                if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
                {
                    Float nextLength = (spline.GetCtrlPoint((i + 1) % cptCount) - spline.GetCtrlPoint((i + 2) % cptCount)).norm();
                    Float ratioTemp = curLength / nextLength;
                    if (ratioTemp < 0.9)
                    {
                        ratio = ratioTemp;
                    }
                }
            }

            // compared with previous segment
            if (i > 0)
            {
                Float preLength = (spline.GetCtrlPoint(i - 1) - spline.GetCtrlPoint(i)).norm();
                Float ratioTemp = curLength / preLength;
                if (ratioTemp < 0.9 && ratioTemp < ratio)
                {
                    ratio = ratioTemp;
                }
            }
            else
            {
                if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
                {
                    Float preLength = (spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0)).norm();
                    Float ratioTemp = curLength / preLength;
                    if (ratioTemp < 0.9 && ratioTemp < ratio)
                    {
                        ratio = ratioTemp;
                    }
                }
            }

            Float tension1 = 0.0;
            tension1 = 1 - sqrt(sin(0.5 * M_PI * ratio));
            // ------------------tension 2------------------------
            Float ang = M_PI * 176. / 180.;
            // compared with next angle
            if (i < clineCount - 1)
            {
                Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(i) - spline.GetCtrlPoint(i + 1),
                                                  spline.GetCtrlPoint((i + 2) % cptCount) - spline.GetCtrlPoint(i + 1));
                if (angTemp < ang)
                {
                    ang = angTemp;
                }
            }
            else
            {
                if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
                {
                    Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0),
                                                      spline.GetCtrlPoint(1) - spline.GetCtrlPoint(0));
                    if (angTemp < ang)
                    {
                        ang = angTemp;
                    }
                }
            }
            // compared with previous angle
            if (i > 0)
            {
                Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(i - 1) - spline.GetCtrlPoint(i),
                                                  spline.GetCtrlPoint((i + 1) % cptCount) - spline.GetCtrlPoint(i));
                if (angTemp < ang)
                {
                    ang = angTemp;
                }
            }
            else
            {
                if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
                {
                    Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0),
                                                      spline.GetCtrlPoint(1) - spline.GetCtrlPoint(0));
                    if (angTemp < ang)
                    {
                        ang = angTemp;
                    }
                }
            }
            // if angle is larger, the tension is smaller
            Float tension2 = 0.0;
            tension2 = 1 - sqrt(sin(0.5 * ang));

            Float tension = 0.0;
            tension = tension1 > tension2 ? tension1 : tension2;
            vector<Pnt3> polyline;
            // cout << __FILE__ << " " << __LINE__ << " divCount=" << divCount << " tension=" << tension << endl;
            for (j = 0; j <= divCount; j++)
            {
                polyline.push_back(spline.GetPoint(i, j / static_cast<Float>(divCount), tension));
            }
            sketchPolyLines.push_back(polyline);
        }
    }
}

void CSketchManager::GetPolyLine(vector<vector<Pnt3>> &sketchPolyLines, vector<Pnt3> &polyline)
{
    polyline.clear();
    polyline.push_back(sketchPolyLines.front().front());
    int n = sketchPolyLines.size();
    for (int i = 0; i < n; i++)
    {
        vector<Pnt3> &polylineTmp = sketchPolyLines[i];
        for (int j = 1; j < polylineTmp.size(); j++)
        {
            if ((polylineTmp[j] - polyline.back()).norm() > 1.0e-4)
            {
                polyline.push_back(polylineTmp[j]);
            }
        }
    }
}

bool CSketchManager::DiscretizeAppointedPolyLine(CSpline &spline, vector<Pnt3> &cur_polyline, int cur_index, int discrete_num)
{
    int cptCount = spline.GetCtrlPointCount();

    cur_polyline.clear();
    vector<Pnt3> &polyline = cur_polyline;
    if (cptCount < 2)
        return false;

    if (cptCount == 2)
    {

        polyline.push_back(spline.GetCtrlPoint(0));
        polyline.push_back(spline.GetCtrlPoint(1));

        return true;
    }

    int clineCount = cptCount - 1;
    if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
    {
        clineCount = cptCount;
    }

    Float clineLength = (spline.GetCtrlPoint(cur_index) - spline.GetCtrlPoint((cur_index + 1) % cptCount)).norm();
    int divCount = discrete_num;

    //compute tension
    Float curLength = (spline.GetCtrlPoint(cur_index) - spline.GetCtrlPoint((cur_index + 1) % cptCount)).norm();
    Float ratio = 1.0;

    if (cur_index < clineCount - 1)
    {
        Float nextLength = (spline.GetCtrlPoint((cur_index + 1) % cptCount) - spline.GetCtrlPoint((cur_index + 2) % cptCount)).norm();
        Float ratioTemp = curLength / nextLength;

        if (ratioTemp < 0.9)
        {
            ratio = ratioTemp;
        }
    }
    else
    {
        if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
        {
            Float nextLength = (spline.GetCtrlPoint(0) - spline.GetCtrlPoint(1)).norm();
            Float ratioTemp = curLength / nextLength;

            if (ratioTemp < 0.9)
            {
                ratio = ratioTemp;
            }
        }
    }

    if (cur_index > 0)
    {
        Float preLength = (spline.GetCtrlPoint(cur_index - 1) - spline.GetCtrlPoint(cur_index)).norm();
        Float ratioTemp = curLength / preLength;

        if (ratioTemp < 0.9 && ratioTemp < ratio)
        {
            ratio = ratioTemp;
        }
    }
    else
    {
        if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
        {
            Float preLength = (spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0)).norm();
            Float ratioTemp = curLength / preLength;

            if (ratioTemp < 0.9 && ratioTemp < ratio)
            {
                ratio = ratioTemp;
            }
        }
    }

    Float tension1 = 0.0;
    tension1 = 1 - sqrt(sin(0.5 * M_PI * ratio));

    Float ang = static_cast<Float>(M_PI * 176. / 180.);
    if (cur_index < clineCount - 1)
    {
        Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cur_index) - spline.GetCtrlPoint(cur_index + 1),
                                          spline.GetCtrlPoint((cur_index + 2) % cptCount) - spline.GetCtrlPoint(cur_index + 1));
        if (angTemp < ang)
        {
            ang = angTemp;
        }
    }
    else
    {
        if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
        {
            Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0),
                                              spline.GetCtrlPoint(1) - spline.GetCtrlPoint(0));
            if (angTemp < ang)
            {
                ang = angTemp;
            }
        }
    }

    if (cur_index > 0)
    {
        Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cur_index - 1) - spline.GetCtrlPoint(cur_index),
                                          spline.GetCtrlPoint((cur_index + 1) % cptCount) - spline.GetCtrlPoint(cur_index));
        if (angTemp < ang)
        {
            ang = angTemp;
        }
    }
    else
    {
        if (spline.GetMode() == spline.SPLMODE_CLOSED_SPLINE)
        {
            Float angTemp = GetAngleOf2Vector(spline.GetCtrlPoint(cptCount - 1) - spline.GetCtrlPoint(0),
                                              spline.GetCtrlPoint(1) - spline.GetCtrlPoint(0));
            if (angTemp < ang)
            {
                ang = angTemp;
            }
        }
    }

    Float tension2 = 0.0;
    tension2 = 1 - sqrt(sin(0.5 * ang));

    Float tension = 0.0;
    tension = tension1 > tension2 ? tension1 : tension2;

    //get polyline
    for (int j = 0; j < divCount; j++)
    {
        polyline.push_back(spline.GetPoint(cur_index, j / static_cast<Float>(divCount), tension));
    }

    return true;
}
