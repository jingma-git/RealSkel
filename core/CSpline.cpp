#include "CSpline.h"
#include "XFunctions.h"
using namespace std;

void CSpline::ChangeMode(int mode)
{
    m_mode = mode;
    m_bSplInitiated = false;
}

void CSpline::AddCtrlPoint(const Pnt3 &p)
{
    P.push_back(p);
    m_bSplInitiated = false;
}

void CSpline::SetZDepth(double depth)
{
    SetZVal(P, depth); // Warning! There might be bugs after setting z-value
}

//----------------------------------------------------------------------
// name:		GetPoint
// function:	Get point based on t
// argument:	int i (point index), float t (discrete split_len on the segment), tension (the whole-line tension)
// return:		Pnt3
// author:
// date:
// update:	    1) set tagent of control point
//              2) generate point of spline based on tangent mode,i.e the tagent has been set directly
Pnt3 CSpline::GetPoint(int i, Float t, Float tension)
{
    Pnt3 v;
    int size = P.size();
    if (m_mode == SPLMODE_CLOSED_SPLINE)
    {
        m_vTangent.resize(size + 1);
        m_vTangentMagnitude0.resize(size + 1);
        m_vTangentMagnitude1.resize(size + 1);
    }
    else
    {
        m_vTangent.resize(size);
        m_vTangentMagnitude0.resize(size);
        m_vTangentMagnitude1.resize(size);
    }

    if ((m_mode != SPLMODE_SPLINE && m_mode != SPLMODE_CLOSED_SPLINE))
    {
        m_bSplineGenerateMode = 0; // normal mode
    }

    // cout << __FILE__ << " " << __LINE__ << " point" << i << " m_mode=" << m_mode << " m_bSplineGenerateMode=" << m_bSplineGenerateMode << endl;
    if (m_bSplineGenerateMode != 1) // normal mode
    {
        if (m_mode == SPLMODE_SPLINE)
        {
            v = GetPointA(i, t);
        }
        else if (m_mode == SPLMODE_CLOSED_SPLINE)
        {
            // cout << __FILE__ << " " << __LINE__ << " m_mode == SPLMODE_CLOSED_SPLINE" << endl;
            v = GetPointA(i, t);
        }
    }
    else
    { // Tagent mode
        assert(false);
    }
    return v;
}

void CSpline::Generate()
{
    int i;

    int num = P.size();

    m_vA.resize(num);
    m_vB.resize(num);
    m_vC.resize(num);

    m_vK.resize(num);
    m_Mat[0].resize(num);
    m_Mat[1].resize(num);
    m_Mat[2].resize(num);

    double AMag, AMagOld;

    // vector A
    for (i = 0; i <= (num - 2); i++)
    {
        m_vA[i] = P[i + 1] - P[i];
    }

    AMagOld = m_vA[0].norm();

    if (AMagOld < 0.0001)
        AMagOld = 0.0001;

    for (i = 0; i <= (num - 3); i++)
    {
        AMag = m_vA[i + 1].norm();

        if (AMag < 0.0001)
            AMag = 0.0001;

        m_vK[i] = AMagOld / AMag;
        AMagOld = AMag;
    }
    m_vK[num - 2] = 1.0f;

    // Matrix
    for (i = 1; i <= num - 2; i++)
    {
        m_Mat[0][i] = 1.0f;
        m_Mat[1][i] = 2.0f * m_vK[i - 1] * (1.0f + m_vK[i - 1]);
        m_Mat[2][i] = m_vK[i - 1] * m_vK[i - 1] * m_vK[i];
    }

    m_Mat[1][0] = 2.0f;
    m_Mat[2][0] = m_vK[0];
    m_Mat[0][num - 1] = 1.0f;
    m_Mat[1][num - 1] = 2.0f * m_vK[num - 2];

    for (i = 1; i <= num - 2; i++)
    {
        m_vB[i] = 3.0f * (m_vA[i - 1] + m_vK[i - 1] * m_vK[i - 1] * m_vA[i]);
    }

    m_vB[0] = 3.0f * m_vA[0];
    m_vB[num - 1] = 3.0f * m_vA[num - 2];

    MatrixSolve(m_vB);

    for (i = 0; i <= num - 2; i++)
    {
        m_vC[i] = m_vK[i] * m_vB[i + 1];
    }

    m_bSplInitiated = true;

    return;
}

void CSpline::MatrixSolve(std::vector<Pnt3> &vB)
{
    int i, j;

    int num = P.size();

    std::vector<Pnt3> Work, WorkB;
    Work.resize(num);
    WorkB.resize(num);

    for (i = 0; i < num; i++)
    {
        if (fabs(m_Mat[1][i]) < 0.0001)
        {
            if (m_Mat[1][i] < 0)
                m_Mat[1][i] = -0.0001;
            else
                m_Mat[1][i] = 0.0001;
        }
    }

    for (i = 0; i <= num - 1; i++)
    {
        Work[i] = vB[i] / m_Mat[1][i];
        WorkB[i] = Work[i];
    }

    for (j = 0; j < 10; j++)
    { ///  need convergence judge
        Work[0] = (vB[0] - m_Mat[2][0] * WorkB[1]) / m_Mat[1][0];
        for (int i = 1; i < num - 1; i++)
        {
            Work[i] = (vB[i] - m_Mat[0][i] * WorkB[i - 1] - m_Mat[2][i] * WorkB[i + 1]) / m_Mat[1][i];
        }
        Work[num - 1] = (vB[num - 1] - m_Mat[0][num - 1] * WorkB[num - 2]) / m_Mat[1][num - 1];

        for (i = 0; i <= num - 1; i++)
        {
            WorkB[i] = Work[i];
        }
    }

    for (i = 0; i <= num - 1; i++)
    {
        vB[i] = Work[i];
    }

    return;
}

Pnt3 CSpline::GetPointA(int i, Float t)
{
    if (m_bSplInitiated == false)
    {
        if (m_mode == SPLMODE_SPLINE)
        {
            //ToDO
            Generate();
        }
        else if (m_mode == SPLMODE_CLOSED_SPLINE)
        {
            GenerateClosedSpline();
        }
    }
    Pnt3 pt;
    float f, g, h;

    f = t * t * (3.0f - 2.0f * t);
    g = t * (t - 1.0f) * (t - 1.0f);
    h = t * t * (t - 1.0f);

    pt = P[i] + m_vA[i] * f + m_vB[i] * g + m_vC[i] * h;

    return pt;
}

// closed cubic spline
void CSpline::GenerateClosedSpline()
{
    int i;
    float AMag, AMagOld, AMag0;

    int num = P.size();

    m_vA.resize(num);
    m_vB.resize(num);
    m_vC.resize(num);

    m_vK.resize(num);
    m_Mat[0].resize(num);
    m_Mat[1].resize(num);
    m_Mat[2].resize(num);

    // vector A
    for (int i = 0; i <= num - 2; i++)
    {
        m_vA[i] = P[i + 1] - P[i];
    }

    m_vA[num - 1] = P[0] - P[num - 1];

    // k
    AMag0 = AMagOld = m_vA[0].norm();

    if (AMag0 < 0.0001)
        AMag0 = 0.0001;

    for (i = 0; i <= num - 2; i++)
    {
        AMag = m_vA[i + 1].norm();

        if (AMag < 0.0001)
            AMag = 0.0001;

        m_vK[i] = AMagOld / AMag;
        AMagOld = AMag;
    }
    m_vK[num - 1] = AMagOld / AMag0;

    // Matrix
    for (i = 1; i <= num - 1; i++)
    {
        m_Mat[0][i] = 1.0f;
        m_Mat[1][i] = 2.0f * m_vK[i - 1] * (1.0f + m_vK[i - 1]);
        m_Mat[2][i] = m_vK[i - 1] * m_vK[i - 1] * m_vK[i];
    }
    m_Mat[0][0] = 1.0f;
    m_Mat[1][0] = 2.0f * m_vK[num - 1] * (1.0f + m_vK[num - 1]);
    m_Mat[2][0] = m_vK[num - 1] * m_vK[num - 1] * m_vK[0];

    for (i = 1; i <= num - 1; i++)
    {
        m_vB[i] = 3.0f * (m_vA[i - 1] + m_vK[i - 1] * m_vK[i - 1] * m_vA[i]);
    }

    m_vB[0] = 3.0f * (m_vA[num - 1] + m_vK[num - 1] * m_vK[num - 1] * m_vA[0]);

    MatrixSolveEX(m_vB);

    for (i = 0; i <= num - 2; i++)
    {
        m_vC[i] = m_vK[i] * m_vB[i + 1];
    }

    m_vC[num - 1] = m_vK[num - 1] * m_vB[0];

    m_bSplInitiated = true;

    return;
}

void CSpline::MatrixSolveEX(vector<Vec3> &vB)
{
    int i, j;

    vector<Vec3> Work, WorkB;

    int num = P.size();

    Work.resize(num);
    WorkB.resize(num);

    for (i = 0; i < num; i++)
    {
        if (fabs(m_Mat[1][i]) < 0.0001)
        {
            if (m_Mat[1][i] < 0)
                m_Mat[1][i] = -0.0001;
            else
                m_Mat[1][i] = 0.0001;
        }
    }

    for (i = 0; i <= num - 1; i++)
    {
        Work[i] = vB[i] / m_Mat[1][i];
        WorkB[i] = Work[i];
    }

    for (j = 0; j < 10; j++)
    { // need judge of convergence
        Work[0] = (vB[0] - m_Mat[0][0] * WorkB[num - 1] - m_Mat[2][0] * WorkB[1]) / m_Mat[1][0];

        for (int i = 1; i < num - 1; i++)
        {
            Work[i] = (vB[i] - m_Mat[0][i] * WorkB[i - 1] - m_Mat[2][i] * WorkB[i + 1]) / m_Mat[1][i];
        }

        Work[num - 1] = (vB[num - 1] - m_Mat[0][num - 1] * WorkB[num - 2] - m_Mat[2][num - 1] * WorkB[0]) / m_Mat[1][num - 1];

        for (i = 0; i <= num - 1; i++)
        {
            WorkB[i] = Work[i];
        }
    }

    for (i = 0; i <= num - 1; i++)
    {
        vB[i] = Work[i];
    }

    return;
}
