// std
#include <fstream>

// igl
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/readOBJ.h>
#include <igl/barycenter.h>
#include <igl/doublearea.h>
// cgal
#include <cgal_adapter.h>

#include "XFunctions.h"
#define ANSI_DECLARATORS
#define REAL double
#define VOID int

extern "C"
{
#include <triangle.h>
}

Float CalLineLength(const vector<Pnt2> &linePts)
{
    assert(linePts.size() >= 2);

    double lenSum = 0.0;
    for (size_t i = 0; i < linePts.size() - 1; i++)
    {
        lenSum += (linePts[i + 1] - linePts[i]).norm();
    }
    return static_cast<Float>(lenSum);
}

Float CalLineLength(const vector<Pnt3> &linePts)
{
    assert(linePts.size() >= 2);

    double lenSum = 0.0;
    for (size_t i = 0; i < linePts.size() - 1; i++)
    {
        lenSum += (linePts[i + 1] - linePts[i]).norm();
    }
    return static_cast<Float>(lenSum);
}

Float CalPolygonLength(const vector<Pnt3> &linePts)
{
    double lenSum = 0.0;
    int n = static_cast<int>(linePts.size());
    for (int i = 0; i < n; ++i)
    {
        lenSum += (linePts[(i + 1) % n] - linePts[i]).norm();
    }
    return static_cast<Float>(lenSum);
}

void SetZVal(vector<Pnt3> &line, double z)
{
    for (Pnt3 &p : line)
    {
        p(2) = z;
    }
}

void TranslateLine(vector<Pnt3> &line, const Vec3 &offset)
{
    for (Pnt3 &p : line)
    {
        p += offset;
    }
}

void RotateLine(vector<Pnt3> &line, const Eigen::Matrix3d &R)
{
    for (Pnt3 &p : line)
    {
        p = R * p;
    }
}

void ScaleLine(vector<Pnt3> &line, double scl)
{
    for (Pnt3 &p : line)
    {
        p = scl * p;
    }
}

void SimplifyPolyLineDP(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float tol)
{
    int i;
    int oriSize = static_cast<int>(oriPArr.size());
    if (oriSize <= 2) //no need simplify
    {
        resPArr.push_back(oriPArr.front());
        resPArr.push_back(oriPArr.back());
        return;
    }

    Float tol2 = tol * tol; // tolerance squared
    vector<Pnt3> PArr;
    vector<bool> PArrMark;

    // std::cout << "...SimplifyPolyLineDP" << std::endl;
    int m = 0;
    for (m = 0; m < 10; m++)
    {
        // std::cout << "..................iter" << m << std::endl;
        // STAGE 1.  Vertex Reduction within tolerance of prior vertex cluster
        PArr.push_back(oriPArr.front()); // start at the beginning
        for (i = 1; i < oriSize - 1; i++)
        {
            if ((oriPArr.at(i) - PArr.back()).squaredNorm() < tol2)
                continue;

            PArr.push_back(oriPArr.at(i));
            // M_DEBUG << i << ", " << PArr.back().transpose() << endl;
        }

        if ((oriPArr.back() - PArr.back()).squaredNorm() < tol2 && static_cast<int>(PArr.size()) > 1)
        {
            PArr.pop_back();
        }

        PArr.push_back(oriPArr.back()); //finish at the end, end pt should reserve

        // STAGE 2.  Douglas-Peucker polyline simplification
        int curSize = static_cast<int>(PArr.size());
        //std::cout << "PArr=" << PArr.size() << std::endl;
        PArrMark.resize(curSize, false);
        PArrMark.front() = true;
        PArrMark.back() = true; // mark the first and last vertices
        SimplifyDP(PArr, 0, curSize - 1, PArrMark, tol);

        //STAGE 3.   copy PArr to resPArr.
        for (i = 0; i < curSize; ++i)
        {
            if (PArrMark.at(i))
                resPArr.push_back(PArr.at(i));
        }
        //std::cout << "resPArr=" << resPArr.size() << std::endl;
        if (resPArr.size() == 2) // simplified to a straight line
        {
            return;
        }

        if (static_cast<int>(resPArr.size()) >= 3) // resPArr.size()) >= 4
        {
            break;
        }
        else
        {
            PArr.clear();
            resPArr.clear();
            tol *= 0.8;
            tol2 = tol * tol;
        }
    }
}
//----------------------------------------Line and Segment--------------------------------------------------------------------
// delete some points which is neraly on a straight line or which is very close to another one
void SimplifyPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float tol)
{
    SimplifyPolyLineDP(oriPArr, resPArr, tol);
    if (static_cast<int>(resPArr.size()) < 4)
    {
        resPArr.clear();
        MakePolyline(oriPArr, resPArr, 3);
    }
}

//---------------------------------------------------------------
// Name:		SimplifyDP
// Description: This is the Douglas-Peucker recursive simplification routine
//				It just marks vertices that are part of the simplified polyline
//				for approximating the polyline subchain PArr[stIdx] to PArr[endIdx].
// Argument:	PArr  : polyline array of vertex points
//				stIdx  : start indice for the subchain
//				endIdx : end indice for the subchain
//				PArrMark : array of markers matching vertex array PArr
//				tol : approximation tolerance
//----------------------------------------------------------------
void SimplifyDP(const vector<Pnt3> &PArr, int stIdx, int endIdx, vector<bool> &PArrMark, Float tol)
{
    if (endIdx <= stIdx + 1) // there is nothing to simplify
        return;

    // check for adequate approximation by segment S from PArr[stIdx] to PArr[endIdx]
    int maxi = stIdx;                          // index of vertex farthest from S
    Float maxd2 = 0;                           // distance squared of farthest vertex
    Float tol2 = tol * tol;                    // tolerance squared
    Vec3 u = PArr.at(endIdx) - PArr.at(stIdx); // segment direction vector
    Float cu = u.squaredNorm();                // segment length squared

    // test each vertex PArr[i] for max distance from S
    Vec3 w;
    Pnt3 Pb;          // base of perpendicular from PArr[i] to S
    Float b, cw, dv2; // dv2 = distance PArr[i] to S squared

    for (int i = stIdx + 1; i < endIdx; ++i)
    {
        // compute distance squared
        w = PArr[i] - PArr[stIdx];
        cw = w.dot(u);
        if (cw <= 0)
            dv2 = (PArr[i] - PArr[stIdx]).squaredNorm();
        else if (cu <= cw)
            dv2 = (PArr[i] - PArr[endIdx]).squaredNorm();
        else
        {
            b = cw / cu;
            Pb = PArr[stIdx] + b * u;
            dv2 = (PArr[i] - Pb).squaredNorm();
        }
        // test with current max distance squared
        if (dv2 >= maxd2)
        {
            // PArr[i] is a new max vertex
            maxi = i;
            maxd2 = dv2;
        }
    }

    if (maxd2 > tol2) // error is worse than the tolerance
    {
        // split the polyline at the farthest vertex from S
        PArrMark[maxi] = 1; // mark pArr[maxi] for the simplified polyline
        // recursively simplify the two subpolylines at pArr[maxi]
        SimplifyDP(PArr, stIdx, maxi, PArrMark, tol);  // polyline PArr[stIdx] to PArr[maxi]
        SimplifyDP(PArr, maxi, endIdx, PArrMark, tol); // polyline PArr[maxi] to PArr[endIdx]
    }

    // else the approximation is OK, so ignore intermediate vertices
    return;
}

/********************************************************************
* FUNCTION NAME :MakePolyline
* AUTHOR		:
* DATE			:2007-03-26
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:Make the polyline
* PARAMETER     :
*		Input 	    :lines:-- points inputed to disperse; nseg:-- line segment number to disperse
*		Return		:
*		Output:	    :pline:-- points dispersed with the segment number is give
* Version		:1.0
********************************************************************/
void MakePolyline(const vector<Pnt3> &lines, vector<Pnt3> &pline, int nseg)
{
    int i, k;
    int np;
    int npt = static_cast<int>(lines.size());
    Float length, slen;
    Float *len;
    if (npt == 0)
    {
        return;
    }
    pline.resize(nseg + 1);
    for (i = 0; i < nseg + 1; i++)
        pline[i] = Pnt3(0, 0, 0);

    len = new Float[npt - 1];

    length = 0;
    for (i = 1; i < npt; i++)
    {
        len[i - 1] = (lines[i] - lines[i - 1]).norm();
        length += len[i - 1];
    }

    slen = length / nseg;

    pline[0] = lines[0];
    pline[nseg] = lines[npt - 1];

    np = 0;
    length = 0;
    k = 1;
    for (i = 0; i < npt - 1; i++)
    {
        length += len[i];
        if (len[i] < 1.0e-6)
        {
            continue;
        }
        if (length >= slen)
        {
            do
            {
                length = length - slen;
                pline[k] = lines[i + 1] + (lines[i] - lines[i + 1]) * length / len[i];
                k++;
            } while (length >= slen);
        }
    }

    if (k == nseg)
    {
        pline[k] = lines[npt - 1];
    }
    delete[] len;
}

// insert points when distance of 2 points is too long
void UniformPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float avgLength)
{
    int i = 0, j = 0, iPolyNum = 0;
    int iOriVSize = static_cast<int>(oriPArr.size());
    Float fDis = 0;
    vector<Pnt3> vtsPoly, vtsList;
    vtsList.resize(2);
    resPArr.clear();

    for (i = 0; i < iOriVSize - 1; i++)
    {
        const Pnt3 &vt1 = oriPArr[i];
        const Pnt3 &vt2 = oriPArr[i + 1];
        fDis = (vt1 - vt2).norm();

        vtsList[0] = vt1;
        vtsList[1] = vt2;

        resPArr.push_back(vtsList[0]);
        if (fDis > avgLength * 1.5)
        {
            vtsPoly.clear();
            iPolyNum = int(fDis / avgLength + 0.5);
            MakePolyline(vtsList, vtsPoly, iPolyNum);
            if (iPolyNum >= 2)
            {
                for (j = 1; j < static_cast<int>(vtsPoly.size()) - 1; j++)
                {
                    resPArr.push_back(vtsPoly[j]);
                }
            }
        }
    }

    resPArr.push_back(oriPArr.back());
}

// laplcian
void SmoothPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, double d, int iter)
{
    int n = static_cast<int>(oriPArr.size());
    vector<Pnt3> tmpPArr;
    tmpPArr.resize(n);
    resPArr.resize(n);

    for (int i = 0; i < oriPArr.size(); i++)
    {
        tmpPArr[i] = oriPArr[i];
    }

    while (iter--)
    {
        for (int i = 0; i < n; ++i)
        {
            const Pnt3 &p_prev = tmpPArr[(n + i - 1) % n];
            const Pnt3 &p_next = tmpPArr[(n + i + 1) % n];
            const Pnt3 &p = tmpPArr[i];
            Vec3 off = (p_prev + p_next) * 0.5 - p;
            resPArr[i] = p + d * (off);
            // cout << (n + i - 1) % n << " p_prev=" << p_prev.transpose() << " "
            //      << (n + i + 1) % n << " p_next=" << p_next.transpose() << " "
            //      << " p=" << p.transpose() << " "
            //      << " off=" << off.transpose() << endl;
        }

        for (int i = 0; i < n; i++)
        {
            tmpPArr[i] = resPArr[i];
        }
    }
}

// bi_lap
void SmoothPolyLineBiLaplician(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, double d, int iter)
{
    int n = static_cast<int>(oriPArr.size());
    vector<Pnt3> tmpPArr;
    tmpPArr.resize(n);
    resPArr.resize(n);

    for (int i = 0; i < oriPArr.size(); i++)
    {
        tmpPArr[i] = oriPArr[i];
    }

    while (iter--)
    {
        for (int i = 0; i < n; ++i)
        {
            const Pnt3 &pA = tmpPArr[(n + i - 2) % n];
            const Pnt3 &pB = tmpPArr[(n + i - 1) % n];
            const Pnt3 &pD = tmpPArr[(n + i + 1) % n];
            const Pnt3 &pE = tmpPArr[(n + i + 2) % n];
            const Pnt3 &p = tmpPArr[i];
            resPArr[i] = (2.0 / 3.0) * (pB + pD - pA * 0.25 - pE * 0.25);
        }

        for (int i = 0; i < n; i++)
        {
            tmpPArr[i] = resPArr[i];
        }
    }
}

void SmoothLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, int iter)
{
    double d = 0.2;
    int n = oriPArr.size();
    vector<Pnt3> tmpPArr;
    tmpPArr.resize(oriPArr.size());
    resPArr.resize(oriPArr.size());

    for (int i = 0; i < oriPArr.size(); i++)
    {
        tmpPArr[i] = oriPArr[i];
    }

    // cout << __FILE__ << " " << __LINE__ << " ...............smoothing..." << endl;
    resPArr[0] = oriPArr[0];
    resPArr[n - 1] = oriPArr[n - 1];
    while (iter--)
    {
        // cout << __FILE__ << " " << __LINE__ << "-------iter" << iter << endl;
        for (int i = 1; i < tmpPArr.size() - 1; i++)
        {
            const Pnt3 &p_prev = tmpPArr[(i - 1)];
            const Pnt3 &p_next = tmpPArr[i + 1];
            const Pnt3 &p = tmpPArr[i];
            resPArr[i] = p + d * ((p_prev + p_next) * 0.5 - p);
            // cout << i << " p=" << p.transpose() << " resPArr[i]=" << resPArr[i].transpose() << " err=" << (resPArr[i] - p).norm()
            //      << " " << ((p_prev + p_next) * 0.5 - p).transpose() << endl;
        }

        for (int i = 0; i < resPArr.size(); i++)
        {
            tmpPArr[i] = resPArr[i];
        }
    }
}

void DiscreteLineByLen(const vector<Pnt2> &linePts, const double &lineLen, const double &divLen, vector<Pnt2> &pts)
{
    // remaining_len = 0.0
    // while True:
    // for each segment on linePts:
    //     if len(previous_remained_part + current_segment)>=fDiv:
    //        try to march 'fDiv', keep tracking remaining_len on current segment after marching
    //        while (remaining_len >= fDiv)
    //            add current point to final output
    //            keep marching on current segment
    //        if (remaining_len < fDiv)
    //            add current point to final output
    //            move to the next segment
    // special processsing for (the last remained_part on the last_segment)
    pts.clear();
    assert(lineLen > 0 && divLen > 0 && linePts.size() >= 2);

    double fLenSum = lineLen;
    int nDiv = static_cast<int>(lineLen / divLen + 0.5);
    const double fDiv = lineLen / (double)nDiv;

    double curLen = 0.0; // the remaining length on current segment after split
    Pnt2 cur = linePts.front();
    pts.push_back(cur);
    int pos = 0;

    while (true)
    {
        double fLen = curLen, segLen = fDiv;
        bool bEnd = true;

        // 1. Discrete the original segment on linePts
        for (size_t i = pos; i < linePts.size() - 1; ++i)
        {
            float dis = sqrt((linePts[i] - linePts[i + 1]).squaredNorm());
            fLen += dis; //fLen: previous remaining length (for previous segment) + current_segment_length

            if (fLen >= fDiv) // divide (previous_remained_part + current_segment)
            {
                bEnd = false;
                double r = (fDiv + dis - fLen) / dis;
                cur = linePts[i] * (1 - r) + linePts[i + 1] * r;
                // cur = Pnt2(linePts[i].x() * (1 - r) + linePts[i + 1].x() * r, linePts[i].y() * (1 - r) + linePts[i + 1].y() * r);
                curLen = fLen - fDiv;
                int prePos = pos;
                pos = i + 1;

                // if (sqrt((cur - pts.back()).squaredNorm()) / fDiv < 0.8) // large curvature
                // {
                //     float fDivTemp = float(fDiv * 0.8);
                //     for (int j = i + 1; j < prePos; --j)
                //     {
                //         float dis = sqrt((linePts[j] - linePts[j - 1]).squaredNorm());
                //         fLen -= dis;
                //         if (fLen <= fDivTemp)
                //         {
                //             float r = (fDivTemp - fLen) / dis;
                //             cur = Pnt2(linePts[j - 1].x() * (1 - r) + linePts[j].x() * r, linePts[j - 1].y() * (1 - r) + linePts[j].y() * r);
                //             curLen = fLen + dis - fDivTemp;
                //             pos = j;
                //             break;
                //         }
                //     }
                //     cout << "fLen=" << fLen << " fDivTemp=" << fDivTemp << " lineLen=" << lineLen << " fDiv=" << fDiv << endl;
                //     assert(fLen <= fDivTemp);
                //     segLen = fDivTemp;
                // }

                pts.push_back(cur);
                while (curLen >= fDiv)
                {
                    // the segment is very long, need to split again
                    r = fDiv / curLen;
                    cur = cur * (1 - r) + linePts[pos] * r;
                    // cur = Pnt2(cur.x() * (1 - r) + linePts[pos].x() * r, cur.y() * (1 - r) + linePts[pos].y() * r);
                    assert(sqrt((cur - pts.back()).squaredNorm()) / fDiv > 0.95); //cur and pts.back() are on straight line
                    pts.push_back(cur);
                    curLen -= fDiv;
                }

                break;
            }
        }

        // 2. Check the last division
        if (bEnd)
        {
            if (nDiv <= 1 && pts.size() == 1)
                pts.push_back(linePts.back());
            else if (fLen / fDiv < 0.1)
                pts.back() = linePts.back();
            else if (fLen / fDiv > 0.6)
                pts.push_back(linePts.back());
            else
            {
                fLen = 0.0;
                double fDivTemp = double(segLen / 2.0);
                for (int j = linePts.size() - 1; j >= 0; --j)
                {
                    double dis = sqrt((linePts[j] - linePts[j - 1]).squaredNorm());
                    fLen += dis;
                    if (fLen >= fDivTemp)
                    {
                        double r = (dis + fDivTemp - fLen) / dis;
                        cur = linePts[j] * (1 - r) + linePts[j - 1] * r;
                        // cur = Pnt2(linePts[j].x() * (1 - r) + linePts[j - 1].x() * r, linePts[j].y() * (1 - r) + linePts[j - 1].y() * r);
                        pts.back() = cur;
                        break;
                    }
                }
                pts.push_back(linePts.back());
            }

            break;
        }
    }
    // cout << "final pts num=" << pts.size() << endl;
    // for (size_t i = 0; i < pts.size(); i++)
    // {
    //     cout << "[" << pts[i] << "] ";
    // }
    // cout << endl;
    assert(pts.size() >= 2);
}

void DiscreteLineByLen(const vector<Pnt3> &linePts, const double &lineLen, const double &divLen, vector<Pnt3> &pts)
{
    // remaining_len = 0.0
    // while True:
    // for each segment on linePts:
    //     if len(previous_remained_part + current_segment)>=fDiv:
    //        try to march 'fDiv', keep tracking remaining_len on current segment after marching
    //        while (remaining_len >= fDiv)
    //            add current point to final output
    //            keep marching on current segment
    //        if (remaining_len < fDiv)
    //            add current point to final output
    //            move to the next segment
    // special processsing for (the last remained_part on the last_segment)
    pts.clear();
    assert(lineLen > 0 && divLen > 0 && linePts.size() >= 2);

    double fLenSum = lineLen;
    int nDiv = static_cast<int>(lineLen / divLen + 0.5);
    const double fDiv = lineLen / (double)nDiv;

    double curLen = 0.0; // the remaining length on current segment after split
    Pnt3 cur = linePts.front();
    pts.push_back(cur);
    int pos = 0;

    while (true)
    {
        double fLen = curLen, segLen = fDiv;
        bool bEnd = true;

        // 1. Discrete the original segment on linePts
        for (size_t i = pos; i < linePts.size() - 1; ++i)
        {
            double dis = sqrt((linePts[i] - linePts[i + 1]).squaredNorm());
            fLen += dis; //fLen: previous remaining length (for previous segment) + current_segment_length

            if (fLen >= fDiv) // divide (previous_remained_part + current_segment)
            {
                bEnd = false;
                double r = (fDiv + dis - fLen) / dis;
                cur = linePts[i] * (1 - r) + linePts[i + 1] * r;
                curLen = fLen - fDiv;
                int prePos = pos;
                pos = i + 1;

                pts.push_back(cur);
                while (curLen >= fDiv)
                {
                    // the segment is very long, need to split again
                    r = fDiv / curLen;
                    cur = cur * (1 - r) + linePts[pos] * r;
                    // assert(sqrt((cur - pts.back()).squaredNorm()) / fDiv > 0.95); //cur and pts.back() are on straight line
                    pts.push_back(cur);
                    curLen -= fDiv;
                }

                break;
            }
        }

        // 2. Check the last division
        if (bEnd)
        {
            if (nDiv <= 1 && pts.size() == 1)
                pts.push_back(linePts.back());
            else if (fLen / fDiv < 0.1)
                pts.back() = linePts.back();
            else if (fLen / fDiv > 0.6)
                pts.push_back(linePts.back());
            else
            {
                fLen = 0.0;
                double fDivTemp = double((segLen + fLen) / 2.0);
                for (int j = linePts.size() - 1; j >= 0; --j)
                {
                    double dis = (linePts[j] - linePts[j - 1]).norm();
                    fLen += dis;
                    if (fLen >= fDivTemp)
                    {
                        float r = (dis + fDivTemp - fLen) / dis;
                        cur = linePts[j] * (1 - r) + linePts[j - 1] * r;
                        pts.back() = cur;
                        break;
                    }
                }
                pts.push_back(linePts.back());
            }

            break;
        }
    }
    // cout << "final pts num=" << pts.size() << endl;
    // for (size_t i = 0; i < pts.size(); i++)
    // {
    //     cout << "[" << pts[i] << "] ";
    // }
    // cout << endl;
    assert(pts.size() >= 2);
}

bool IsTwoLineIntersectIn2D(const Pnt3 &ln1StPt, const Pnt3 &ln1EndPt, const Pnt3 &ln2StPt, const Pnt3 &ln2EndPt)
{
    Vec3 m1 = ln1StPt - ln2StPt;
    Vec3 m2 = ln1EndPt - ln2StPt;
    Vec3 m3 = ln2EndPt - ln2StPt;
    if (m1.cross(m3).z() * m2.cross(m3).z() > 0)
        return false;
    m1 = ln2StPt - ln1StPt;
    m2 = ln2EndPt - ln1StPt;
    m3 = ln1EndPt - ln1StPt;
    if ((m1.cross(m3)).dot(m2.cross(m3)) > 0)
        return false;
    return true;
}

bool IsLineSelfIntersect(const vector<Pnt2> &linePts)
{
    if (linePts.size() < 3)
        return false;

    bool isEnclose = ((linePts[0] - linePts.back()).norm() < 1.0e-6);
    int pNum = isEnclose ? linePts.size() - 1 : linePts.size();

    for (int i = 0; i < linePts.size() - 1; i++)
    {

        // bounding box check btw current_line and next_line
        for (int k = i + 2; k < pNum - 1; k++)
        {
            float minX = min(linePts[i].x(), linePts[i + 1].x());
            float maxX = max(linePts[i].x(), linePts[i + 1].x());
            float minY = min(linePts[i].y(), linePts[i + 1].y());
            float maxY = max(linePts[i].y(), linePts[i + 1].y());

            bool flag1 = (linePts[k].x() < minX && linePts[k + 1].x() < minX);
            bool flag2 = (linePts[k].x() > maxX && linePts[k + 1].x() > maxX);
            bool flag3 = (linePts[k].y() < minY && linePts[k + 1].y() < minY);
            bool flag4 = (linePts[k].y() > maxY && linePts[k + 1].x() > maxY);
            if (flag1 || flag2 || flag3 || flag4)
                continue;
            Pnt3 p0(linePts[i].x(), linePts[i].y(), 0);
            Pnt3 p1(linePts[i + 1].x(), linePts[i + 1].y(), 0);
            Pnt3 p2(linePts[k].x(), linePts[k].y(), 0);
            Pnt3 p3(linePts[k + 1].x(), linePts[k + 1].y(), 0);
            if (IsTwoLineIntersectIn2D(p0, p1, p2, p3))
                return true;
        }
    }
    return false;
}

bool IsLineSelfIntersect(const vector<Pnt3> &linePts)
{
    if (linePts.size() < 3)
        return false;

    bool isEnclose = ((linePts[0], linePts.back()).norm() < 1.0e-6);
    int pNum = isEnclose ? linePts.size() - 1 : linePts.size();

    for (int i = 0; i < linePts.size() - 1; i++)
    {

        // bounding box check btw current_line and next_line
        for (int k = i + 2; k < pNum - 1; k++)
        {
            float minX = min(linePts[i].x(), linePts[i + 1].x());
            float maxX = max(linePts[i].x(), linePts[i + 1].x());
            float minY = min(linePts[i].y(), linePts[i + 1].y());
            float maxY = max(linePts[i].y(), linePts[i + 1].y());

            bool flag1 = (linePts[k].x() < minX && linePts[k + 1].x() < minX);
            bool flag2 = (linePts[k].x() > maxX && linePts[k + 1].x() > maxX);
            bool flag3 = (linePts[k].y() < minY && linePts[k + 1].y() < minY);
            bool flag4 = (linePts[k].y() > maxY && linePts[k + 1].x() > maxY);
            if (flag1 || flag2 || flag3 || flag4)
                continue;
            Pnt3 p0(linePts[i].x(), linePts[i].y(), 0);
            Pnt3 p1(linePts[i + 1].x(), linePts[i + 1].y(), 0);
            Pnt3 p2(linePts[k].x(), linePts[k].y(), 0);
            Pnt3 p3(linePts[k + 1].x(), linePts[k + 1].y(), 0);
            if (IsTwoLineIntersectIn2D(p0, p1, p2, p3))
                return true;
        }
    }
    return false;
}

bool IsClockWise(const vector<Pnt3> &linePts)
{
    // https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    // https://www.element84.com/blog/determining-the-winding-of-a-polygon-given-as-a-set-of-ordered-points
    double sum = 0;

    for (int i = 0; i < linePts.size(); i++)
    {
        const Pnt3 &v1 = linePts[i];
        const Pnt3 &v2 = linePts[(i + 1) % linePts.size()];
        sum += (v2.x() - v1.x()) * (v2.y() + v1.y());
    }
    return sum > 0.0;
}

//---------------------------------------------------------------
// Name:		Intersection
// Description: get the length ratio of the two intersecting lines
// Argument:	v1: line one first point
//				v2  line one second point
//				v3: line two first point
//				v4: line two second point
//				r:  first line ratio: length start point to intersection with length of start point to end point
//				s:  second line ratio: length start point to intersection with length of start point to end point
// Return:		true for intersection
// Author:		unknown
// Date:
// Modified by:
// Updated date:
//----------------------------------------------------------------
bool GetTwoLineIntersectRatio(const Pnt3 &v1, const Pnt3 &v2, const Pnt3 &v3, const Pnt3 &v4, Float &r, Float &s)
{
    // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
    Float temp1, temp2, temp3;
    temp1 = (v1.y() - v3.y()) * (v4.x() - v3.x()) - (v1.x() - v3.x()) * (v4.y() - v3.y());
    temp2 = (v2.x() - v1.x()) * (v4.y() - v3.y()) - (v2.y() - v1.y()) * (v4.x() - v3.x());
    temp3 = (v1.y() - v3.y()) * (v2.x() - v1.x()) - (v1.x() - v3.x()) * (v2.y() - v1.y());

    if (temp2 == 0)
    {
        return false;
    }

    r = temp1 / temp2;
    s = temp3 / temp2;
    return true;
}

//---------------------------------------------------------------
// Name:		GetTwoLineInterPt
// Description: Get intersection point of two lines
// Argument:			ln1StPt, ln1EndPt: the start and end point of line1
//						ln2StPt, ln2EndPt: the start and end point of line2
// Return:		intersect point
//----------------------------------------------------------------
Pnt3 GetTwoLineInterPt(Pnt3 ln1StPt, Pnt3 ln1EndPt, Pnt3 ln2StPt, Pnt3 ln2EndPt)
{
    Float r, s;

    if (GetTwoLineIntersectRatio(ln1StPt, ln1EndPt, ln2StPt, ln2EndPt, r, s))
    {
        return (ln1StPt + r * (ln1EndPt - ln1StPt));
    }
    else
    {
        return ln1EndPt;
    }
}

void FindIntersectionPts(const vector<Pnt3> &line1, const vector<Pnt3> &line2, vector<LineIntersection> &isects)
{
    for (int i = 0; i < line1.size(); i++)
    {
        int l1_st = i, l1_end = (i + 1) % line1.size();
        const Pnt3 &ln1StPt = line1[l1_st];
        const Pnt3 &ln1EndPt = line1[l1_end];
        for (int j = 0; j < line2.size(); j++)
        {
            int l2_st = j, l2_end = (j + 1) % line2.size();
            const Pnt3 &ln2StPt = line2[l2_st];
            const Pnt3 &ln2EndPt = line2[l2_end];
            if (IsTwoLineIntersectIn2D(ln1StPt, ln1EndPt, ln2StPt, ln2EndPt))
            {
                Float r, s;
                if (GetTwoLineIntersectRatio(ln1StPt, ln1EndPt, ln2StPt, ln2EndPt, r, s))
                {
                    Pnt3 pos = ln1StPt + r * (ln1EndPt - ln1StPt);
                    LineIntersection isect(pos, l1_st, l1_end, l2_st, l2_end);
                    isects.push_back(isect);
                }
            }
        }
    }
}

bool IsPointInTriangle(const Pnt3 &testP, const Pnt3 &p0, const Pnt3 &p1, const Pnt3 &p2)
{
    Vec3 f_norm = FaceNorm(p0, p1, p2);
    Pnt3 vert[3] = {p0, p1, p2};
    const Float dis = (Float)1.0e-4;
    Float xMax, xMin, yMax, yMin, zMax, zMin;
    xMax = std::max(vert[0].x(), std::max(vert[1].x(), vert[2].x())) + dis;
    xMin = std::min(vert[0].x(), std::min(vert[1].x(), vert[2].x())) - dis;
    yMax = std::max(vert[0].y(), std::max(vert[1].y(), vert[2].y())) + dis;
    yMin = std::min(vert[0].y(), std::min(vert[1].y(), vert[2].y())) - dis;
    zMax = std::max(vert[0].z(), std::max(vert[1].z(), vert[2].z())) + dis;
    zMin = std::min(vert[0].z(), std::min(vert[1].z(), vert[2].z())) - dis;
    if (testP.x() > xMax || testP.x() < xMin || testP.y() > yMax || testP.y() < yMin || testP.z() > zMax || testP.z() < zMin)
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if ((testP - vert[(i + 1) % 3]).norm() < dis || (testP - vert[i]).norm() < dis)
            continue;
        Vec3 temp_norm = (vert[i] - testP).cross(vert[(i + 1) % 3] - testP);
        if (f_norm.dot(temp_norm) < 0)
            return false;
    }
    return true;
}

//---------------------------------------------------------------
// Name:		IsPointInPolygon
// Description: shooting an infinite starting from test point
// count how many intersections it has with polygon edges,
// if count is even, it is outside, if count is odd, it is inside
//----------------------------------------------------------------
bool IsPointInPolygon(const Pnt3 &p, const std::vector<Pnt3> &poly)
{
    // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
    // http://geomalgorithms.com/a03-_inclusion.html
    // int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
    // {
    //     int i, j, c = 0;
    //     for (i = 0, j = nvert - 1; i < nvert; j = i++)
    //     {
    //         if (((verty[i] > testy) != (verty[j] > testy)) &&
    //             (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
    //             c = !c;
    //     }
    //     return c;
    // }

    int i, j, c = 0;
    int nvert = poly.size();
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if ((poly[i].y() > p.y()) != (poly[j].y() > p.y()) &&
            (p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()))
            c = !c;
    }
    return c;
}

IntersectionType IsPointIntersectTriangle(const Pnt3 &testP, const Pnt3 &p0, const Pnt3 &p1, const Pnt3 &p2)
{
    Vec3 f_norm = FaceNorm(p0, p1, p2);
    Pnt3 vert[3] = {p0, p1, p2};
    const Float dis = (Float)1.0e-4;
    Float xMax, xMin, yMax, yMin, zMax, zMin;
    xMax = std::max(vert[0].x(), std::max(vert[1].x(), vert[2].x())) + dis;
    xMin = std::min(vert[0].x(), std::min(vert[1].x(), vert[2].x())) - dis;
    yMax = std::max(vert[0].y(), std::max(vert[1].y(), vert[2].y())) + dis;
    yMin = std::min(vert[0].y(), std::min(vert[1].y(), vert[2].y())) - dis;
    zMax = std::max(vert[0].z(), std::max(vert[1].z(), vert[2].z())) + dis;
    zMin = std::min(vert[0].z(), std::min(vert[1].z(), vert[2].z())) - dis;
    if (testP.x() > xMax || testP.x() < xMin || testP.y() > yMax || testP.y() < yMin || testP.z() > zMax || testP.z() < zMin)
    {
        return EMPTY;
    }

    for (int i = 0; i < 3; i++)
    {
        if ((testP - vert[(i + 1) % 3]).norm() < dis || (testP - vert[i]).norm() < dis)
        {
            return ON_VERTEX;
        }
        Vec3 temp_norm = (testP - vert[i]).cross(testP - vert[(i + 1) % 3]);
        if (abs(temp_norm.norm()) < 1e-8)
        {
            return ON_EDGE;
        }
        if (f_norm.dot(temp_norm) < 0)
            return EMPTY;
    }
    return ON_FACE;
}

std::pair<IntersectionType, int> IsPointIntersectTriangle(const Pnt3 &testP, const Fd &fd, const TMesh &ms)
{
    Hd hd = ms.halfedge(fd);
    const Pnt3 &p0 = ms.point(ms.source(hd));
    const Pnt3 &p1 = ms.point(ms.target(hd));
    const Pnt3 &p2 = ms.point(ms.target(ms.next(hd)));
    Vec3 f_norm = FaceNorm(p0, p1, p2);
    Pnt3 vert[3] = {p0, p1, p2};
    const Float dis = (Float)1.0e-4;
    Float xMax, xMin, yMax, yMin, zMax, zMin;
    xMax = std::max(vert[0].x(), std::max(vert[1].x(), vert[2].x())) + dis;
    xMin = std::min(vert[0].x(), std::min(vert[1].x(), vert[2].x())) - dis;
    yMax = std::max(vert[0].y(), std::max(vert[1].y(), vert[2].y())) + dis;
    yMin = std::min(vert[0].y(), std::min(vert[1].y(), vert[2].y())) - dis;
    zMax = std::max(vert[0].z(), std::max(vert[1].z(), vert[2].z())) + dis;
    zMin = std::min(vert[0].z(), std::min(vert[1].z(), vert[2].z())) - dis;
    if (testP.x() > xMax || testP.x() < xMin || testP.y() > yMax || testP.y() < yMin || testP.z() > zMax || testP.z() < zMin)
    {
        return {EMPTY, -1};
    }

    for (Hd hd : ms.halfedges_around_face(hd))
    {
        Vd v0 = ms.source(hd);
        Vd v1 = ms.target(hd);
        const Pnt3 &src = ms.point(v0);
        const Pnt3 &tgt = ms.point(v1);
        if ((testP - src).norm() < dis)
        {
            return {ON_VERTEX, v0.idx()};
        }

        if ((testP - tgt).norm() < dis)
        {
            return {ON_VERTEX, v1.idx()};
        }

        Vec3 temp_norm = (src - testP).cross(tgt - testP);
        if (abs(temp_norm.norm()) < 1e-4)
        {
            return {ON_EDGE, ms.edge(hd).idx()};
        }
        if (f_norm.dot(temp_norm) < 0)
            return {EMPTY, -1};
    }

    return {ON_FACE, fd.idx()};
}

Float GetAngleOf2Vector(const Vec3 &v1, const Vec3 &v2)
{
    Float len1, len2, r, val, cos, angle;
    len1 = sqrt(v1.x() * v1.x() + v1.y() * v1.y() + v1.z() * v1.z());
    len2 = sqrt(v2.x() * v2.x() + v2.y() * v2.y() + v2.z() * v2.z());
    r = v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();

    val = len1 * len2;
    if (fabs(val) < 1.0e-6)
    {
        angle = 0;
        return M_PI / 2.0f;
    }
    else
    {
        cos = r / val;
        if (cos > 1.0)
        {
            return 0.0f;
        }
        else if (cos < -1.0)
        {
            return M_PI;
        }
        angle = acos(cos);
    }
    return angle;
}

void DistancePointToLine(const Pnt3 &src, const Pnt3 &dst, const Pnt3 &p, double &dis, double &t)
{
    Vec3 p01 = dst - src;
    Vec3 p02 = p - src;
    double a = p01.norm();
    double b = p02.norm();
    double ab_dot = p01.dot(p02);
    if (fabs(ab_dot - 0) < 1e-8) // perpendicular
    {
        t = 0;
        dis = b;
    }
    else
    {
        double cos = ab_dot / (a * b);
        t = (b * cos) / a;
        Pnt3 proj_p = src + t * p01;
        dis = (p - proj_p).norm();
    }
}

//---------------------------------------- Rotation--------------------------------------
void GetRotMatX(double angle, Eigen::Matrix3d &R)
{
    double cosa = std::cos(angle);
    double sina = std::sin(angle);
    R << 1, 0, 0,
        0, cosa, -sina,
        0, sina, cosa;
}

void GetRotMatY(double angle, Eigen::Matrix3d &R)
{
    double cosa = std::cos(angle);
    double sina = std::sin(angle);
    R << cosa, 0, sina,
        0, 1, 0,
        -sina, 0, cosa;
}

void GetRotMatZ(double angle, Eigen::Matrix3d &R)
{
    double cosa = std::cos(angle);
    double sina = std::sin(angle);

    R << cosa, -sina, 0,
        sina, cosa, 0,
        0, 0, 1;
}

void SymmetrizePtsXY(const std::vector<Pnt3> &pts,
                     std::vector<Pnt3> &sym_pts,
                     double plane_pos)
{
    sym_pts.clear();
    sym_pts.reserve(pts.size());
    for (const Pnt3 &p : pts)
    {
        sym_pts.push_back(Pnt3(p.x(), p.y(), 2 * plane_pos - p.z()));
    }
}

void SymmetrizePtsYZ(const std::vector<Pnt3> &pts,
                     std::vector<Pnt3> &sym_pts,
                     double plane_pos)
{
    sym_pts.clear();
    sym_pts.reserve(pts.size());
    for (const Pnt3 &p : pts)
    {
        sym_pts.push_back(Pnt3(2 * plane_pos - p.x(), p.y(), p.z()));
    }
}

bool IntersectPlane(const Plane &plane, const Ray &ray, Pnt3 &isect)
{
    Vec3 N = plane.normal();
    const Pnt3 &p = plane.p;
    const Pnt3 &o = ray.origin;
    const Vec3 &d = ray.dir;
    double dN_dot = d.dot(N);
    if (fabs(dN_dot - 0) < 1e-14)
        return false;

    double t = (p - o).dot(N) / dN_dot;
    // M_DEBUG << "IntersectPlane" << plane << " t=" << t << endl;

    isect = o + t * d;
    return true;
}

//---------------------------------------- BBOX --------------------------------------------------------------------
bool IsInsideBBox(const BBox3 &bbox, const Pnt3 &p)
{
    return (p.x() >= bbox.xmin && p.x() <= bbox.xmax) &&
           (p.y() >= bbox.ymin && p.y() <= bbox.ymax) &&
           (p.z() >= bbox.zmin && p.z() <= bbox.zmax);
}

//---------------------------------------- Mesh --------------------------------------------------------------------
static bool AddTriedInfoToMesh(TMesh &pMesh, const triangulateio &out)
{
    int i = 0;
    if (out.numberoftriangles == 0 || out.numberofpoints == 0 || out.numberofedges == 0 || out.numberofsegments == 0)
    {
        return false;
    }

    for (i = 0; i < out.numberofpoints; ++i)
    {
        pMesh.add_vertex(Pnt3(out.pointlist[2 * i], out.pointlist[2 * i + 1], 0.0));
    }

    for (i = 0; i < out.numberoftriangles; ++i)
    {
        pMesh.add_face(Vd(out.trianglelist[3 * i]), Vd(out.trianglelist[3 * i + 1]), Vd(out.trianglelist[3 * i + 2]));
    }
    // cout << __FILE__ << " " << __LINE__ << " AddTriedInfoToMesh Successfully!" << endl;

    return true;
}

bool Triangulation(const vector<Pnt2> &contPts, TMesh &mesh, int iMode, float refArea, bool debug)
{
    // debug = true;
    if (contPts.size() < 3)
    {
        return false;
    }

    vector<Pnt2> loop;
    for (const Pnt2 &p : contPts)
    {
        loop.push_back(p);
    }
    if ((loop.front() - loop.back()).norm() < 1e-3)
    {
        cout << __FILE__ << " " << __LINE__ << " Triangulation input polygon line is closed, so pop back" << endl;
        loop.pop_back();
    }

    char p[50];
    if (iMode == Constrained_Delaunay_Split)
    {
        //p-poly
        //q-quality
        //30: angle less than 30 degree
        //a: area
        //B: suppress boundary marker list
        //Q: quiet
        //e: output edge list
        //s: segment should be foreced into the triangulation by recursively splitting at eheir midpoint
        //z: index starting from zero
        //Y: prohibits inserting stenior points
        if (debug)
            sprintf(p, "pq30a%.2fBYesz", refArea); //area of refined triangles
        else
            sprintf(p, "pq30a%.2fBYeszQ", refArea);
    }
    else if (iMode == Constrained)
    {
        if (debug)
            sprintf(p, "pBYez");
        else
            sprintf(p, "pBYezQ");
    }
    else if (iMode == Constrained_Delaunay)
    {
        //p-poly
        //q-quality
        //30: angle less than 30 degree
        //a: area
        //B: suppress boundary marker list
        //Q: quiet
        //e: output edge list
        //s: segment should be foreced into the triangulation by recursively splitting at eheir midpoint
        //z: index starting from zero
        //Y: prohibits inserting stenior points
        if (debug)
            sprintf(p, "pq30a%.2fBYez", refArea); //area of refined triangles
        else
            sprintf(p, "pq30a%.2fBYezQ", refArea);
    }
    else
    {
        sprintf(p, "Bez");
    }

    std::cout << "triangulate command " << p << std::endl;
    try
    {
        triangulateio tri_in, tri_out;

        tri_in.numberofpoints = int(loop.size());
        tri_in.numberofsegments = int(loop.size());
        tri_in.numberofpointattributes = 0;
        tri_in.numberofholes = 0;
        tri_in.pointmarkerlist = NULL;

        tri_in.segmentmarkerlist = NULL;

        tri_in.numberoftriangles = 0;
        tri_in.trianglelist = NULL;
        tri_in.numberoftriangleattributes = 0;
        tri_in.triangleattributelist = NULL;
        tri_in.numberofcorners = 0;
        tri_in.numberofregions = 0;

        tri_in.pointlist = new double[tri_in.numberofpoints * 2];
        tri_in.segmentlist = new int[tri_in.numberofsegments * 2];
        //in->holelist = new REAL[m_in.numberofholes * 2];

        for (int i = 0; i < loop.size(); i++)
        {
            tri_in.pointlist[2 * i] = loop[i].x();
            tri_in.pointlist[2 * i + 1] = loop[i].y();
        }
        for (int i = 0; i < loop.size(); i++)
        {
            tri_in.segmentlist[2 * i] = i;
            tri_in.segmentlist[2 * i + 1] = (i + 1) == int(loop.size()) ? 0 : (i + 1);
        }

        tri_out.pointlist = NULL;
        tri_out.edgelist = NULL;
        tri_out.trianglelist = NULL;
        tri_out.segmentlist = NULL;
        tri_out.segmentmarkerlist = NULL;
        tri_out.pointmarkerlist = NULL;

        //BuildTrianglationInfoFromLoop(loop, tri_in);
        triangulate(p, &tri_in, &tri_out, 0);
        printf("---------------Triangulate done!\n");

        if (!AddTriedInfoToMesh(mesh, tri_out))
        {
            return false;
        }

        delete[] tri_in.pointlist;
        tri_in.pointlist = 0;
        delete[] tri_in.segmentlist;
        tri_in.segmentlist = 0;
        // mesh.ComputeNormals();
        return true;
    }
    catch (std::exception &e)
    {
        return false;
    }
}

bool Triangulation(const vector<Pnt3> &contPts, TMesh &mesh, int iMode, float refArea, bool debug)
{
    // debug = true;
    if (contPts.size() < 3)
    {
        return false;
    }

    vector<Pnt3> loop;
    for (const Pnt3 &p : contPts)
    {
        loop.push_back(p);
    }
    if ((loop.front() - loop.back()).norm() < 1e-3)
    {
        cout << __FILE__ << " " << __LINE__ << " Triangulation input polygon line is closed, so pop back" << endl;
        loop.pop_back();
    }

    char p[50];
    if (iMode == Constrained_Delaunay_Split)
    {
        //p-poly
        //q-quality
        //30: angle less than 30 degree
        //a: area
        //B: suppress boundary marker list
        //Q: quiet
        //e: output edge list
        //s: segment should be foreced into the triangulation by recursively splitting at eheir midpoint
        //z: index starting from zero
        //Y: prohibits inserting stenior points
        if (debug)
            sprintf(p, "pq30a%.2fBYesz", refArea); //area of refined triangles
        else
            sprintf(p, "pq30a%.2fBYeszQ", refArea);
    }
    else if (iMode == Constrained)
    {
        if (debug)
            sprintf(p, "pBYez");
        else
            sprintf(p, "pBYezQ");
    }
    else if (iMode == Constrained_Delaunay)
    {
        //p-poly
        //q-quality
        //30: angle less than 30 degree
        //a: area
        //B: suppress boundary marker list
        //Q: quiet
        //e: output edge list
        //s: segment should be foreced into the triangulation by recursively splitting at eheir midpoint
        //z: index starting from zero
        //Y: prohibits inserting stenior points
        if (debug)
            sprintf(p, "pq30a%.2fBYez", refArea); //area of refined triangles
        else
            sprintf(p, "pq30a%.2fBYezQ", refArea);
    }
    else
    {
        sprintf(p, "Bez");
    }

    std::cout << "triangulate command " << p << std::endl;
    try
    {
        triangulateio tri_in, tri_out;

        tri_in.numberofpoints = int(loop.size());
        tri_in.pointlist = new double[tri_in.numberofpoints * 2];
        tri_in.numberofpointattributes = 0;
        tri_in.pointmarkerlist = NULL;

        tri_in.trianglelist = NULL;
        tri_in.numberoftriangles = 0;
        tri_in.numberofcorners = 0;
        tri_in.numberoftriangleattributes = 0;
        tri_in.triangleattributelist = NULL;

        tri_in.numberofsegments = int(loop.size());
        tri_in.segmentlist = new int[tri_in.numberofsegments * 2];
        tri_in.segmentmarkerlist = NULL;

        tri_in.numberofholes = 0;
        tri_in.holelist = NULL;
        tri_in.numberofregions = 0;

        //in->holelist = new REAL[m_in.numberofholes * 2];

        for (int i = 0; i < loop.size(); i++)
        {
            tri_in.pointlist[2 * i] = loop[i].x();
            tri_in.pointlist[2 * i + 1] = loop[i].y();
        }
        for (int i = 0; i < loop.size(); i++)
        {
            tri_in.segmentlist[2 * i] = i;
            tri_in.segmentlist[2 * i + 1] = ((i + 1) % loop.size());
        }

        tri_out.pointlist = NULL;
        tri_out.trianglelist = NULL;
        tri_out.edgelist = NULL;
        tri_out.segmentlist = NULL;
        tri_out.segmentmarkerlist = NULL;
        tri_out.pointmarkerlist = NULL;

        //BuildTrianglationInfoFromLoop(loop, tri_in);
        triangulate(p, &tri_in, &tri_out, 0);
        printf("---------------Triangulate done!\n");

        if (!AddTriedInfoToMesh(mesh, tri_out))
        {
            return false;
        }

        delete[] tri_in.pointlist;
        tri_in.pointlist = 0;
        delete[] tri_in.segmentlist;
        tri_in.segmentlist = 0;
        // mesh.ComputeNormals();
        return true;
    }
    catch (std::exception &e)
    {
        return false;
    }
}

void SplitFace_2DMesh(TMesh &ms)
{
    // ToDO:
    assert(false);
}

double Volume(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
{
    // this is actually an inaccurate approximation of real volume
    using namespace Eigen;
    using namespace std;
    double sum = 0;
    RowVector3d bl = V.colwise().minCoeff();
    RowVector3d tr = V.colwise().maxCoeff();
    RowVector3d cent = (bl + tr) * 0.5;
    // cout << "cent" << cent << endl;
    for (int i = 0; i < F.rows(); i++)
    {
        const RowVector3d a = V.row(F(i, 0)) - cent;
        const RowVector3d b = V.row(F(i, 1)) - cent;
        const RowVector3d c = V.row(F(i, 2)) - cent;
        sum += a.x() * b.y() * c.z() +
               a.z() * b.x() * c.y() +
               a.y() * c.x() * b.z() -
               c.x() * b.y() * a.z() -
               c.y() * b.z() * a.x() -
               c.z() * b.x() * a.y();
    }
    return sum;
}

Pnt3 Centroid(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
{
    Eigen::VectorXd dblA;
    igl::doublearea(V, F, dblA);
    double meshArea = 0.5 * dblA.sum();
    Eigen::MatrixXd BC;
    igl::barycenter(V, F, BC);
    Eigen::RowVector3d centroid(0, 0, 0);
    for (int i = 0; i < BC.rows(); i++)
    {
        centroid += 0.5 * dblA(i) / meshArea * BC.row(i);
    }
    return centroid.transpose();
}

void WritePly(TMesh &mesh, string fname)
{
    igl::writePLY(fname, mesh.GetVMat(), mesh.GetFMat());
}

void WriteOBJ(TMesh &mesh, string fname)
{
    // Eigen::MatrixXd &V = mesh.GetVMat();
    // Eigen::MatrixXi &F = mesh.GetFMat();
    // if (V.cols() != 3 || F.cols() != 3)
    //     return;

    // igl::writeOBJ(fname, V, F);
    cgal_adapter::write_obj(fname, mesh);
}

void WriteOFF(TMesh &mesh, string fname)
{
    Eigen::MatrixXd &V = mesh.GetVMat();
    Eigen::MatrixXi &F = mesh.GetFMat();
    if (V.cols() != 3 || F.cols() != 3)
        return;

    igl::writeOFF(fname, V, F);
}

void ReadOBJ(TMesh &mesh, string fname)
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // igl::readOBJ(fname, V, F);
    // mesh.CreateHalfedgeMesh(V, F);
    cgal_adapter::read_obj(fname, mesh);
}
