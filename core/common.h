#pragma once
// -------------------------macros-------------------------------------------------
#ifndef M_DEBUG
#define M_DEBUG std::cout << __FILE__ << " " << __LINE__ << " "
#endif

//-------------------------Customized Data Structure--------------------------------
#include "SMesh.h"

//---------------------------------------STL Library--------------------------------
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <unordered_map>
#include <list>
#include <cmath>
#include <boost/optional/optional.hpp>

#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif

//---------------------------------------Eigen--------------------------------------
#include <Eigen/Eigen>
#include <Eigen/Sparse>

//-------------Basic DataType Similar to WinDef.h on Windows------------------------
typedef unsigned char BYTE;  // 8 bits/one byte
typedef unsigned short WORD; // two bytes
typedef unsigned long DWORD; // four bytes
typedef unsigned int UINT;

using namespace std;

//---------------------------------------Surface Mesh--------------------------------

// kernel
typedef double Float;
typedef Eigen::Vector2d Pnt2;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Pnt3;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2i CPoint;

// surface mesh
typedef Surface_mesh<Pnt3> SMesh;
typedef SMesh::Vertex_index Vd;
typedef SMesh::Vertex_iterator Vertex_iterator;
// typedef SMesh::In_edge_iterator In_edge_iterator;
typedef SMesh::Face_index Fd;
typedef SMesh::Edge_index Ed;
typedef SMesh::Halfedge_index Hd;

enum ViewMode
{
    VIEW_FRONT,
    VIEW_BACK,
    VIEW_RIGHT,
    VIEW_LEFT,
    VIEW_TOP,
    VIEW_BOTTOM,
    VIEW_PERSP
};

enum CoordAxis
{
    COORD_AXIS_X,
    COORD_AXIS_Y,
    COORD_AXIS_Z
};

enum RotAxis
{
    ROT_AXIS_X,
    ROT_AXIS_Y,
    ROT_AXIS_Z
};

//------------------
enum SymetryPlane
{
    SYM_XY_PLANE = 0,
    SYM_YZ_PLANE
};

enum SkeletonOption
{
    SKEL_OFFSET_POLYGON,
    SKEL_SYM
};

//---------------------------------------Customized Datastructure --------------------------------
struct BBox2
{
    Float xmin, ymin, xmax, ymax;
    BBox2()
    {
        xmin = std::numeric_limits<double>::max();
        ymin = std::numeric_limits<double>::max();
        xmax = std::numeric_limits<double>::min();
        ymax = std::numeric_limits<double>::min();
    }

    BBox2(Float xmin_, Float ymin_, Float xmax_, Float ymax_) : xmin(xmin_), ymin(ymin_), xmax(xmax_), ymax(ymax_) {}
    double diaglen() { return sqrt((xmax - xmin) * (xmax - xmin) + (ymax - ymin) * (ymax - ymin)); }
};

inline ostream &operator<<(ostream &out, const BBox2 &b)
{
    out << b.xmin << ", " << b.ymin << ", " << b.xmax << ", " << b.ymax;
    return out;
}

struct BBox3
{
    double xmin, ymin, zmin, xmax, ymax, zmax;
    BBox3() {}
    BBox3(double xmin_, double ymin_, double zmin_, double xmax_, double ymax_, double zmax_)
        : xmin(xmin_), ymin(ymin_), zmin(zmin_), xmax(xmax_), ymax(ymax_), zmax(zmax_) {}

    double volume()
    {
        return (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
    }

    double max_axis_len()
    {
        double x_len = extent_x();
        double y_len = extent_y();
        double z_len = extent_z();
        if (x_len > y_len)
        {
            if (x_len > z_len)
            {
                return x_len;
            }
            else // x_len < z_len
            {
                return z_len;
            }
        }
        else // x_len < y_len
        {
            if (y_len > z_len)
            {
                return y_len;
            }
            else // y_len < z_len
            {
                return z_len;
            }
        }
    }

    double extent_x()
    {
        return xmax - xmin;
    }

    double extent_y()
    {
        return ymax - ymin;
    }

    double extent_z()
    {
        return zmax - zmin;
    }
};

inline ostream &operator<<(ostream &out, const BBox3 &b)
{
    out << b.xmin << ", " << b.ymin << ", " << b.zmin << "|" << b.xmax << ", " << b.ymax << ", " << b.zmax;
    return out;
}

enum IntersectionType
{
    ON_FACE,
    ON_EDGE,
    ON_VERTEX,
    EMPTY
};

struct LineIntersection
{
    Pnt3 pos;          // intersect point
    int l1_st, l1_end; // segment on polyline 1
    int l2_st, l2_end; // segment on polyline 2
    LineIntersection(Pnt3 pos_, int l1_st_, int l1_end_, int l2_st_, int l2_end_)
        : pos(pos_), l1_st(l1_st_), l1_end(l1_end_), l2_st(l2_st_), l2_end(l2_end_) {}
};

const std::vector<std::string> inter_names = {"ON_FACE",
                                              "ON_EDGE",
                                              "ON_VERTEX",
                                              "EMPTY"};

template <class Point>
inline BBox2 GetBBox(const vector<Point> &pts)
{
    Float xmin = 1e6, xmax = 1e-6, ymin = 1e6, ymax = 1e-6;
    for (const Point &p : pts)
    {
        if (p.x() < xmin)
        {
            xmin = p.x();
        }
        if (p.y() < ymin)
        {
            ymin = p.y();
        }
        if (p.x() > xmax)
        {
            xmax = p.x();
        }
        if (p.y() > ymax)
        {
            ymax = p.y();
        }
    }
    return {xmin, ymin, xmax, ymax};
}

inline bool IsIntersect(const BBox2 &b1, const BBox2 &b2)
{

    if (b2.xmax < b1.xmin || b1.xmax < b2.xmin)
    {
        return false;
    }
    else if (b2.ymax < b1.ymin || b1.ymax < b2.ymin)
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <class Point>
bool IsIntersect(const vector<Point> &ln1, const vector<Point> &ln2) //ToDO: change, is not corret
{
    BBox2 b1 = GetBBox(ln1);
    BBox2 b2 = GetBBox(ln2);
    if (b2.xmax < b1.xmin || b1.xmax < b2.xmin)
    {
        return false;
    }
    else if (b2.ymax < b1.ymin || b1.ymax < b2.ymin)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief calcuate triangle_face normal, notice the triangle must be arranged strictly in p0->p1->p2
 * 
 * @param p0 
 * @param p1 
 * @param p2 
 * @return Vec3 
 */
inline Vec3 FaceNorm(const Pnt3 &p0, const Pnt3 &p1, const Pnt3 &p2)
{
    return (p1 - p0).cross(p2 - p0);
}

//-----------------ToDO: remove Rotate and ComputeGLbAng------------------end

inline Float ComputeArea(const Pnt3 &p1, const Pnt3 &p2, const Pnt3 &p3)
{
    return (p1 - p2).cross(p3 - p2).norm() * 0.5;
}

inline bool IsZero(double x)
{
    return std::fabs(x - 0) < 1e-14;
}