#pragma once

#include "common.h"
#include "TMesh.h"
#include "Plane.h"
#include "Line.h"
#include "Ray.h"
#include <Eigen/Eigen>

//----------------------------------------Line, Segment and Point--------------------------------------------------------------------
Float CalLineLength(const vector<Pnt2> &linePts);
Float CalLineLength(const vector<Pnt3> &linePts);
Float CalPolygonLength(const vector<Pnt3> &linePts);
void SetZVal(vector<Pnt3> &line, double z);
void TranslateLine(vector<Pnt3> &line, const Vec3 &offset);
void RotateLine(vector<Pnt3> &line, const Eigen::Matrix3d &R);
void ScaleLine(vector<Pnt3> &line, double scl);
void SimplifyPolyLineDP(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float tol);
void SimplifyPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float tol);
void SimplifyDP(const vector<Pnt3> &PArr, int stIdx, int endIdx, vector<bool> &PArrMark, Float tol);
void MakePolyline(const vector<Pnt3> &lines, vector<Pnt3> &pline, int nseg);
void UniformPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, float avgLength);
void SmoothLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, int iter = 5);
void SmoothPolyLine(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, double d = 0.2, int iter = 5);
void SmoothPolyLineBiLaplician(const vector<Pnt3> &oriPArr, vector<Pnt3> &resPArr, double d = 0.2, int iter = 5);
void DiscreteLineByLen(const vector<Pnt2> &linePts, const double &lineLen, const double &divLen, vector<Pnt2> &pts);
void DiscreteLineByLen(const vector<Pnt3> &linePts, const double &lineLen, const double &divLen, vector<Pnt3> &pts);
bool IsTwoLineIntersectIn2D(const Pnt3 &ln1StPt, const Pnt3 &ln1EndPt, const Pnt3 &ln2StPt, const Pnt3 &ln2EndPt);
bool IsLineSelfIntersect(const vector<Pnt2> &linePts);
bool IsLineSelfIntersect(const vector<Pnt3> &linePts); // all z must be zero
bool IsClockWise(const vector<Pnt3> &linePts);
bool GetTwoLineIntersectRatio(const Pnt3 &v1, const Pnt3 &v2, const Pnt3 &v3, const Pnt3 &v4, Float &r, Float &s);
Pnt3 GetTwoLineInterPt(Pnt3 ln1StPt, Pnt3 ln1EndPt, Pnt3 ln2StPt, Pnt3 ln2EndPt);
void FindIntersectionPts(const vector<Pnt3> &line1, const vector<Pnt3> &line2, vector<LineIntersection> &isects);
bool IsPointInTriangle(const Pnt3 &p, const Pnt3 &p0, const Pnt3 &p1, const Pnt3 &p2);
bool IsPointInPolygon(const Pnt3 &p, const std::vector<Pnt3> &poly);
IntersectionType IsPointIntersectTriangle(const Pnt3 &testP, const Pnt3 &p0, const Pnt3 &p1, const Pnt3 &p2);
std::pair<IntersectionType, int> IsPointIntersectTriangle(const Pnt3 &testP, const Fd &fd, const TMesh &ms);
Float GetAngleOf2Vector(const Vec3 &v1, const Vec3 &v2);
void DistancePointToLine(const Pnt3 &src, const Pnt3 &dst, const Pnt3 &p, double &dis, double &t);
//---------------------------------------- Plane --------------------------------------------------------------------
inline void GetRotMatToXYPlane(const Vec3 &src_n, Eigen::Matrix3d &rot)
{
    // https://math.stackexchange.com/questions/1167717/transform-a-plane-to-the-xy-plane
    using namespace Eigen;
    Vec3 unitX(0, 0, 1);
    Vec3 plane_n = src_n.normalized();
    Vec3 axis = plane_n.cross(unitX).normalized();
    double angle = std::acos(plane_n.dot(unitX));
    rot = AngleAxisd(angle, axis);
}

void GetRotMatX(double angle, Eigen::Matrix3d &R);
void GetRotMatY(double angle, Eigen::Matrix3d &R);
void GetRotMatZ(double angle, Eigen::Matrix3d &R);
void SymmetrizePtsXY(const std::vector<Pnt3> &pts,
                     std::vector<Pnt3> &sym_pts,
                     double plane_pos);

void SymmetrizePtsYZ(const std::vector<Pnt3> &pts,
                     std::vector<Pnt3> &sym_pts,
                     double plane_pos);

bool IntersectPlane(const Plane &plane, const Ray &ray, Pnt3 &isect);
//---------------------------------------- BBOX --------------------------------------------------------------------
bool IsInsideBBox(const BBox3 &bbox, const Pnt3 &p);
//---------------------------------------- Mesh --------------------------------------------------------------------
enum TriangulationMode
{
    Constrained = 0,
    Constrained_Delaunay_Split,
    Constrained_Delaunay

};

bool Triangulation(const vector<Pnt2> &loop, TMesh &mesh, int iMode, float refArea = -1.0, bool debug = false); //ToDO: make as template
bool Triangulation(const vector<Pnt3> &loop, TMesh &mesh, int iMode, float refArea = -1.0, bool debug = false);
void SplitFace_2DMesh(TMesh &ms);
void SewMeshes(
    const Eigen::MatrixXd &V1,
    const Eigen::MatrixXi &F1,
    const std::vector<int> &b1,
    const Eigen::MatrixXd &V2,
    const Eigen::MatrixXi &F2,
    const std::vector<int> &b2,
    Eigen::MatrixXd &V,
    Eigen::MatrixXi &F);
double Volume(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
Pnt3 Centroid(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

void WritePly(TMesh &mesh, string name);
void WriteOBJ(TMesh &mesh, string fname);
void WriteOFF(TMesh &mesh, string fname);
void ReadOBJ(TMesh &mesh, string fname);

template <class Graph>
void print_graph(const Graph &g, std::string name)
{
    using namespace std;
    cout << "============" << name << endl;
    cout << "-------------vertices:" << endl;
    for (Vd vd : g.vertices())
    {
        cout << vd << ":" << g.point(vd).transpose() << endl;
    }

    cout << "-------------halfedges:" << endl;
    for (Hd hd : g.halfedges())
    {
        Fd f = g.face(hd);
        Vd vd0 = g.source(hd);
        Vd vd1 = g.target(hd);

        cout << hd << "|" << vd0 << "->" << vd1 << "|" << g.edge(hd) << "|" << f
             << " halfedge(" << vd0 << ")=" << g.halfedge(vd0) << " halfedge(" << vd1 << ")=" << g.halfedge(vd1) << endl;
    }
}

inline bool readPoly(std::string path, vector<Pnt2> &pts)
{
    std::fstream file(path);
    if (!file.is_open())
    {
        return false;
    }

    std::string line;
    std::stringstream ss;

    std::getline(file, line);
    int v_num, dim, attr_num, marker_num;
    ss << line;
    ss >> v_num >> dim >> attr_num >> marker_num;
    for (int i = 0; i < v_num; i++)
    {
        int vidx;
        float x, y;

        std::string line;
        std::stringstream ss;

        std::getline(file, line);
        ss << line;
        ss >> vidx >> x >> y;
        pts.push_back(Pnt2(x, y));
    }
    return true;
}

template <class Point>
inline bool writePolyline(std::string path, const vector<Point> &pts)
{
    FILE *fptr;
    fptr = fopen(path.c_str(), "w");

    if (fptr == NULL)
    {
        printf("Error!\n");
        return false;
    }
    fprintf(fptr, "%d\n", static_cast<int>(pts.size()));
    for (int i = 0; i < pts.size(); i++)
    {
        fprintf(fptr, "%g %g %g\n", pts[i][0], pts[i][1], pts[i][2]);
    }
    fclose(fptr);
    return true;
}

template <class Point>
inline bool readPolyline(std::string path, vector<Point> &pts)
{
    FILE *fptr;
    fptr = fopen(path.c_str(), "r");

    if (fptr == NULL)
    {
        printf("Error!\n");
        return false;
    }
    int n;
    fscanf(fptr, "%d\n", &n);
    pts.resize(n);
    for (int i = 0; i < pts.size(); i++)
    {
        fscanf(fptr, "%lf %lf %lf\n", &pts[i][0], &pts[i][1], &pts[i][2]);
    }
    fclose(fptr);
    return true;
}

//---------------------------------------- Painting --------------------------------------------------------------------
inline Pnt2 GetPtstOfTriangle(const Pnt3 &p, const Pnt3 &p1, const Pnt3 &p2, const Pnt3 &p3)
{
    double s, t;
    Vec3 u = p3 - p2;
    Vec3 v = p1 - p2;
    Vec3 w = p - p2;

    double uuDot = u.dot(u);
    double vvDot = v.dot(v);
    double uvDot = u.dot(v);
    double uwDot = u.dot(w);
    double wvDot = w.dot(v);

    double det = uvDot * uvDot - uuDot * vvDot;
    if (abs(det) < 1e-6)
        return Pnt2(0, 0);
    s = (uvDot * wvDot - vvDot * uwDot) / det;
    t = (uvDot * uwDot - uuDot * wvDot) / det;
    return Pnt2(s, t);
}