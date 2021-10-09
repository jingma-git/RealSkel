#pragma once

#include "common.h"

class CSpline
{
public:
    enum SPLINE_MODE
    {
        SPLMODE_LINER = 0,
        SPLMODE_SPLINE = 1,
        SPLMODE_BSPLINE = 2,
        SPLMODE_BEZIER = 3,
        SPLMODE_CLOSED_SPLINE = 4,
        SPLMODE_CLOSED_BSPLINE = 5,
        SPLMODE_CLOSED_BEZIER = 6,
    };

    CSpline()
    {
        Clear();
    }

    CSpline(const CSpline &other)
    {
        P = other.P;
        m_mode = other.m_mode;
        m_bSplineGenerateMode = other.m_bSplineGenerateMode;
        m_vTangent = other.m_vTangent;
        m_vTangentMagnitude0 = other.m_vTangentMagnitude0;
        m_vTangentMagnitude1 = other.m_vTangentMagnitude1;

        m_bSplInitiated = false;
        int num = P.size();
        m_vA.resize(num);
        m_vB.resize(num);
        m_vC.resize(num);

        m_vK.resize(num);
        m_Mat[0].resize(num);
        m_Mat[1].resize(num);
        m_Mat[2].resize(num);
    }

    void Clear()
    {
        m_bSplineGenerateMode = 0;
        m_bSplInitiated = false;

        P.clear();
        m_vTangent.clear();
        m_vTangentMagnitude0.clear();
        m_vTangentMagnitude1.clear();

        m_vA.clear();
        m_vB.clear();
        m_vC.clear();
        m_vK.clear();
        m_Mat[0].clear();
        m_Mat[1].clear();
        m_Mat[2].clear();
    }

    void ChangeMode(int mode);
    void AddCtrlPoint(const Pnt3 &p);
    void SetZDepth(double depth);
    int GetCtrlPointCount() const { return P.size(); }
    Pnt3 &GetCtrlPoint(int i) { return P[i]; }
    const Pnt3 &GetCtrlPoint(int i) const { return P[i]; }
    int GetMode() const { return m_mode; }
    Pnt3 GetPoint(int i, Float t, Float tension);
    Pnt3 GetPointA(int i, Float t);
    void Generate();
    void MatrixSolve(std::vector<Pnt3> &vB);
    void GenerateClosedSpline();
    void MatrixSolveEX(vector<Vec3> &vB);

public:
    bool m_bSplInitiated;
    // for generating a good shape of a b-spline
    vector<Vec3> m_vA;
    vector<Vec3> m_vB;
    vector<Vec3> m_vC;
    vector<float> m_vK;
    vector<float> m_Mat[3];
    vector<Pnt3> P; // control point

protected:
    int m_mode;

    int m_bSplineGenerateMode;
    // the tagent of each control point i have two magnitude
    vector<Pnt3> m_vTangent;
    vector<Float> m_vTangentMagnitude0; // magnitude0 influence segment (i-1, i)
    vector<Float> m_vTangentMagnitude1; // magnitude1 influence segment (i, i+1)
};
