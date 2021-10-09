// egl
#include "../egl/readLine.h"
#include "../egl/writeLine.h"
#include "../egl/writeStr.h"
#include "../egl/string_util.h"

// basic function
#include "XSubToy.h"
#include "XFunctions.h"
#include "auto_skel.h"
#include "CSketchManager.h"
#include "EGL.h"

// laplace-inflation
#include <igl/writePLY.h>
#include <igl/boundary_facets.h>
#include <igl/unique.h>
#include <igl/colon.h>
#include <igl/setdiff.h>
#include <igl/slice.h>
#include <igl/lbs_matrix.h>
#include <igl/bbw.h>
#include "cgal_adapter.h"

#include "../egl/writeLine.h"
// arap_layer deform
#include "ArapDeform.h"

#include "boundary_conditions.h"
// skeletonization
#include "straight_skeletor.h"
#include "sskel_to_bone.h"
#include "quicksort.h"
#include "sskel_io.h"

// for rotation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>

static bool debug = false;
const float DIS_INTER = 1500;
const float CTRL_PT_SIZE = 20.0f;
static int BBW_ITER = 30;

using namespace Eigen;
// #define ROT_DEBUG

std::ostream &operator<<(std::ostream &out, const XSubToy &subToy)
{
    out << "id:" << subToy.m_toyID << "\n"
        << "isSelected:" << subToy.m_isSelected << "\n"
        << "parentID:" << subToy.m_parentID << "\n"
        << "symToyID:" << subToy.m_symToyID << "\n"
        << "layerID:" << subToy.m_layerID << "\n"
        << "layerThick:" << subToy.m_layerThick << "\n"
        << "layerDepth:" << subToy.m_layerDepth << "\n"
        << "layerAxis:" << subToy.m_layerAxis << "\n"
        << "iniViewDir:" << subToy.m_iniViewDir[0] << " " << subToy.m_iniViewDir[1] << " " << subToy.m_iniViewDir[2] << "\n"
        << "autoSkel:" << subToy.m_autoSkel << "\n"
        << "skelOpt:" << subToy.m_skelOpt << "\n"
        << "centroid:" << subToy.m_centroid[0] << " " << subToy.m_centroid[1] << " " << subToy.m_centroid[2] << "\n";
    const Pnt3 &p0 = subToy.m_symLine.p0;
    const Pnt3 &p1 = subToy.m_symLine.p1;
    out << "symLine:" << p0[0] << " " << p0[1] << " " << p0[2] << " " << p1[0] << " " << p1[1] << " " << p1[2] << endl;
    return out;
}

std::istream &operator>>(std::istream &in, XSubToy &subToy)
{
    std::string line, key, value;
    int position;
    while (std::getline(in, line))
    {
        if (line[0] == '#')
            continue;
        position = line.find(":");
        if (position == line.npos)
            continue;
        key = line.substr(0, position);
        value = line.substr(position + 1, line.length());
        egl::trim(value);
        if (key == "id")
        {
            subToy.m_toyID = std::stoi(value);
        }
        else if (key == "isSelected")
        {
            subToy.m_isSelected = std::stoi(value);
        }
        else if (key == "parentID")
        {
            subToy.m_parentID = std::stoi(value);
        }
        else if (key == "symToyID")
        {
            subToy.m_symToyID = std::stoi(value);
        }
        else if (key == "layerID")
        {
            subToy.m_layerID = std::stoi(value);
        }
        else if (key == "layerThick")
        {
            subToy.m_layerThick = std::stod(value);
        }
        else if (key == "layerDepth")
        {
            subToy.m_layerDepth = std::stod(value);
        }
        else if (key == "layerAxis")
        {
            subToy.m_layerAxis = std::stoi(value);
        }
        else if (key == "iniViewDir")
        {
            Vec3 &dir = subToy.m_iniViewDir;
            sscanf(value.c_str(), "%lf %lf %lf", &dir[0], &dir[1], &dir[2]);
        }
        else if (key == "autoSkel")
        {
            subToy.m_autoSkel = std::stoi(value);
        }
        else if (key == "skelOpt")
        {
            subToy.m_skelOpt = std::stoi(value);
        }
        else if (key == "centroid")
        {
            Vec3 &cent = subToy.m_centroid;
            sscanf(value.c_str(), "%lf %lf %lf", &cent[0], &cent[1], &cent[2]);
        }
        else if (key == "symLine")
        {
            Pnt3 &p0 = subToy.m_symLine.p0;
            Pnt3 &p1 = subToy.m_symLine.p1;
            sscanf(value.c_str(), "%lf %lf %lf %lf %lf %lf", &p0[0], &p0[1], &p0[2], &p1[0], &p1[1], &p1[2]);
        }
    }
	return in;
}

XSubToy::XSubToy()
    : m_centroid(0, 0, 0)
{
    m_sketchSpline.ChangeMode(CSpline::SPLINE_MODE::SPLMODE_CLOSED_SPLINE);
}

XSubToy::XSubToy(const XSubToy *other, const SymetryPlane sym_plane, double plane_pos)
{
    // parent-child relationship
    SetParent(other->m_parent);
    SetSymToy(const_cast<XSubToy *>(other));

    // contour line 3D, centroid, m_iniViewDir
    if (sym_plane == SYM_XY_PLANE)
    {
        M_DEBUG << " SYM_XY_PLANE plane_pos=" << plane_pos << endl;
        if (other->InitViewMode() == VIEW_FRONT ||
            other->InitViewMode() == VIEW_BACK)
        {
            SetInitViewDir(-other->m_iniViewDir);
        }
        else if (other->InitViewMode() == VIEW_RIGHT ||
                 other->InitViewMode() == VIEW_LEFT)
        {
            SetInitViewDir(other->m_iniViewDir);
        }
        else if (other->InitViewMode() == VIEW_TOP ||
                 other->InitViewMode() == VIEW_BOTTOM)
        {
            SetInitViewDir(other->m_iniViewDir);
        }

        m_layerID = -m_layerID;
        m_layerThick = other->m_layerThick;
        m_layerDepth = -other->m_layerDepth;

        SymmetrizePtsXY(other->m_sketchSpline.P, m_sketchSpline.P, plane_pos);
        SymmetrizePtsXY(other->m_sketchPolyLine, m_sketchPolyLine, plane_pos);
    }
    else if (sym_plane == SYM_YZ_PLANE)
    {
        M_DEBUG << " SYM_YZ_PLANE plane_pos=" << plane_pos << endl;
        if (other->InitViewMode() == VIEW_FRONT ||
            other->InitViewMode() == VIEW_BACK)
        {
            SetInitViewDir(other->m_iniViewDir);
        }
        else if (other->InitViewMode() == VIEW_RIGHT ||
                 other->InitViewMode() == VIEW_LEFT)
        {
            SetInitViewDir(-other->m_iniViewDir);
        }
        else if (other->InitViewMode() == VIEW_TOP ||
                 other->InitViewMode() == VIEW_BOTTOM)
        {
            SetInitViewDir(other->m_iniViewDir);
        }

        m_layerID = other->m_layerID;
        m_layerThick = other->m_layerThick;
        m_layerDepth = other->m_layerDepth;

        SymmetrizePtsYZ(other->m_sketchSpline.P, m_sketchSpline.P, plane_pos);
        SymmetrizePtsYZ(other->m_sketchPolyLine, m_sketchPolyLine, plane_pos);
    }

    SetCentroid();
    M_DEBUG << "symmetry centroid: " << m_centroid.transpose() << endl;
    SetRotToXY();
    SetSketchPolyLineOnXY();

    // meshes
    GenSymMesh(sym_plane, plane_pos, other->m_curMesh2DTemp, m_curMesh2DTemp);
    GenSymMesh(sym_plane, plane_pos, other->m_curMesh2D, m_curMesh2D);
    GenSymMesh(sym_plane, plane_pos, other->m_defMesh2D, m_defMesh2D);
    GenSymMesh(sym_plane, plane_pos, other->m_curMesh3D, m_curMesh3D);

    // skeleton
    m_autoSkel = other->m_autoSkel;
    m_skelOpt = other->m_skelOpt;
    GenSymSSkel(sym_plane, plane_pos, other->m_rawSSkel, m_rawSSkel);
    m_rawSSkel.collect_nodes(junc_nodes, term_nodes);
    collect_branches(junc_nodes, term_nodes, branches);
    if (m_autoSkel)
    {
        GenSymSSkel(sym_plane, plane_pos, other->m_sskel, m_sskel);
    }

    M_DEBUG << "the old subtoy: " << endl;
    cout << *other << endl;

    M_DEBUG << "the newed symetric subtoy: " << endl;
    cout << *this << endl;
}

void XSubToy::Save(std::string out_dir)
{
    std::string istr = std::to_string(GetID());

    std::ofstream out;
    out.open(out_dir + "/BasicInfo" + istr + ".txt");
    if (out.is_open())
    { // basic info
        out << *this;

        // lines
        egl::writeLine(out_dir + "/SketchSpline" + istr + ".txt", m_sketchSpline.P);
        egl::writeLine(out_dir + "/SketchPolyline" + istr + ".txt", m_sketchPolyLine);
        // meshes
        WriteOBJ(m_curMesh2DTemp, out_dir + "/SubMesh2DTemp" + istr + ".obj");
        WriteOBJ(m_curMesh2D, out_dir + "/SubMesh2D" + istr + ".obj");
        WriteOBJ(m_curMesh3D, out_dir + "/SubMesh3D" + istr + ".obj");
        // skeleton
        writeSSkel(out_dir + "/rawSSkel" + istr + ".txt", m_rawSSkel);
        writeSSkel(out_dir + "/m_sskel" + istr + ".txt", m_sskel);
    }

    out.close();
}

void XSubToy::Load(std::string in_dir)
{
    std::string istr = std::to_string(GetID());
    std::ifstream in(in_dir + "/BasicInfo" + istr + ".txt");
    if (in.is_open())
    {
        in >> *this;
        M_DEBUG << "Open XSubToy BasicInfo Successfully" << endl;
        cout << *this;

        // lines
        egl::readLine(in_dir + "/SketchSpline" + istr + ".txt", m_sketchSpline.P);
        egl::readLine(in_dir + "/SketchPolyline" + istr + ".txt", m_sketchPolyLine);
        // M_DEBUG << "m_sketchSpline=" << m_sketchSpline.GetCtrlPointCount() << endl;
        // M_DEBUG << "m_sketchPolyline=" << m_sketchPolyLine.size() << endl;

        SetRotToXY();
        SetSketchPolyLineOnXY();

        // meshes
        ReadOBJ(m_curMesh2DTemp, in_dir + "/SubMesh2DTemp" + istr + ".obj");
        ReadOBJ(m_curMesh2D, in_dir + "/SubMesh2D" + istr + ".obj");
        // WriteOBJ(m_defMesh2D, in_dir + "/SubDefMesh2D" + istr + ".obj");
        ReadOBJ(m_curMesh3D, in_dir + "/SubMesh3D" + istr + ".obj");

        // skeleton
        readSSkel(in_dir + "/rawSSkel" + istr + ".txt", m_rawSSkel);
        m_rawSSkel.collect_nodes(junc_nodes, term_nodes);
        m_rawSSkel.set_root(m_rawSSkel.find_node_with_max_branches());
        collect_branches(junc_nodes, term_nodes, branches);
        readSSkel(in_dir + "/m_sskel" + istr + ".txt", m_sskel);
        m_sskel.set_root(m_sskel.find_node_with_max_branches());
        // M_DEBUG << name() << " read rawSSkel" << endl;
        // m_rawSSkel.print();
        // M_DEBUG << name() << " read mSSkel" << endl;
        // m_sskel.print();

        M_DEBUG << "XSubToy::Load " << name() << " successfully!\n";
    }
    else
    {
        M_DEBUG << "Cannot open " << in_dir << endl;
    }

    in.close();
}

double XSubToy::CalSymPlanePos(const SymetryPlane sym_plane)
{
    if (m_parent == nullptr)
        return 0;

    double plane_pos;
    BBox3 b = m_parent->m_curMesh3D.bbox();
    if (sym_plane == SYM_XY_PLANE)
    {
        plane_pos = (b.zmin + b.zmax) / 2.0;
        M_DEBUG << "CalSymPlanePos  SYM_XY_PLANE bbox=" << b << " plane_pos=" << plane_pos << endl;
    }
    else if (sym_plane == SYM_YZ_PLANE)
    {
        plane_pos = (b.xmin + b.xmax) / 2.0;
        M_DEBUG << "CalSymPlanePos  SYM_YZ_PLANE bbox=" << b << " plane_pos=" << plane_pos << endl;
    }
    return plane_pos;
}

void XSubToy::AddChild(XSubToy *child)
{
    if (std::find(m_children.begin(), m_children.end(), child) == m_children.end())
    {
        m_children.push_back(child);
    }
}

void XSubToy::SetParent(XSubToy *parent)
{
    if (parent)
    {
        this->m_parent = parent;
        // Tell parent she has just given birth
        this->m_parent->AddChild(this);
        this->m_parentID = m_parent->GetID();
        // M_DEBUG << parent->name() << " add me " << this->name() << endl;
    }
}

void XSubToy::LeaveParent()
{
    if (m_parent)
    {
        m_parent->EraseChild(this);
        m_parent = nullptr;
        m_parentID = -1;
    }
}

void XSubToy::DeadToChildren()
{
    for (XSubToy *child : m_children)
    {
        child->m_parent = nullptr;
    }
}

void XSubToy::EraseChild(XSubToy *child)
{
    m_children.erase(std::remove(m_children.begin(), m_children.end(), child), m_children.end());
    // M_DEBUG << name() << " erase child " << child->name() << " " << m_children.size() << endl;
}

void XSubToy::SetSymToy(XSubToy *symToy)
{
    m_symToy = symToy;
    if (symToy)
    {
        m_symToyID = symToy->GetID();
    }
    else
    {
        m_symToyID = -1;
    }
}

Vec3 XSubToy::GetAxisDepthVec()
{
    if (m_layerID != 0 && fabs(m_layerDepth - 0) < 1e-14)
    {
        m_layerDepth = CalDepth();
    }

    if (m_layerAxis == COORD_AXIS_X)
    {
        return Vec3(1, 0, 0) * m_layerDepth;
    }
    else if (m_layerAxis == COORD_AXIS_Y)
    {
        return Vec3(0, 1, 0) * m_layerDepth;
    }
    else
    {
        return Vec3(0, 0, 1) * m_layerDepth;
    }
}

Plane XSubToy::GetLayerPlane()
{
    M_DEBUG << "m_layerDepth=" << m_layerDepth << endl;
    if (m_layerAxis == COORD_AXIS_X)
    {
        return Plane(Vec3(1, 0, 0), Pnt3(1, 0, 0) * m_layerDepth);
    }
    else if (m_layerAxis == COORD_AXIS_Y)
    {
        return Plane(Vec3(0, 1, 0), Pnt3(0, 1, 0) * m_layerDepth);
    }
    else
    {
        return Plane(Vec3(0, 0, 1), Pnt3(0, 0, 1) * m_layerDepth);
    }
}

double XSubToy::CalDepth()
{
    if (m_layerID == 0)
    {
        m_layerDepth = 0;
        return m_layerDepth;
    }

    // ToDO: add the function to mannuly set subToy's depth
    if (m_parent && m_layerID > 0) // put this subToy in front of its parent
    {
        BBox3 bbox = m_parent->m_curMesh3D.bbox();
        m_layerDepth = 0.5 * (bbox.zmax + bbox.zmin) + 0.333 * (bbox.zmax - bbox.zmin); //ToDO: make 0.333 as a parameter that can be adjusted by users
        // M_DEBUG << "CalDepth() bbox=" << bbox
        //         << "  0.5 * (bbox.zmax + bbox.zmin)=" << 0.5 * (bbox.zmax + bbox.zmin)
        //         << " bbox.zmax - bbox.zmin=" << (bbox.zmax - bbox.zmin) << endl;
        // M_DEBUG << " parent=" << m_parent->name() << " this=" << this->name()
        //         << " m_layerDepth=" << m_layerDepth << endl;
    }
    return m_layerDepth;
}

void XSubToy::SetZDepth(double depth)
{
    if (m_layerID == 0)
        return;
    SetLayerDepth(depth);
    // contour
    m_sketchSpline.SetZDepth(depth);
    SetZVal(m_sketchPolyLine, depth);
    // meshes
    m_curMesh2DTemp.SetZDepth(depth);
    m_curMesh2D.SetZDepth(depth);
    m_defMesh2D.SetZDepth(depth);
    m_curMesh3D.SetZDepth(depth);

    m_attachPoint(2) += depth;
}

double XSubToy::ContourArea()
{
    assert(m_curMesh2DTemp.number_of_faces() > 0);
    return m_curMesh2DTemp.Get3DArea();
}

void XSubToy::Scale(double scl)
{
    // M_DEBUG << "XSubToy::Scale " << scl << endl;
    // lines
    TranslateLine(m_sketchSpline.P, -m_centroid);
    ScaleLine(m_sketchSpline.P, scl);
    TranslateLine(m_sketchSpline.P, m_centroid);

    TranslateLine(m_sketchPolyLine, -m_centroid);
    ScaleLine(m_sketchPolyLine, scl);
    TranslateLine(m_sketchPolyLine, m_centroid);

    // meshes
    m_curMesh3D.Translate(-m_centroid);
    m_curMesh3D.Scale(scl);
    m_curMesh3D.Translate(m_centroid);
    // WriteOBJ(m_curMesh3D, "output/m_curMesh3D_scl.obj");

    m_curMesh2DTemp.Translate(-m_centroid);
    m_curMesh2DTemp.Scale(scl);
    m_curMesh2DTemp.Translate(m_centroid);

    m_curMesh2D.Translate(-m_centroid);
    m_curMesh2D.Scale(scl);
    m_curMesh2D.Translate(m_centroid);
    m_defMesh2D = m_curMesh2D;

    // skeleton
    TranslateSSkel(-m_centroid);
    ScaleSSkel(scl);
    TranslateSSkel(m_centroid);

    // update m_rotToXY
    SetRotToXY();
    SetSketchPolyLineOnXY();
}

void XSubToy::Translate(const Vec3 &trans_update)
{
    TranslateLine(m_sketchSpline.P, trans_update);
    TranslateLine(m_sketchPolyLine, trans_update);

    // meshes
    m_curMesh3D.Translate(trans_update);
    m_curMesh2DTemp.Translate(trans_update);
    m_curMesh2D.Translate(trans_update);
    m_defMesh2D = m_curMesh2D;

    // skeleton
    TranslateSSkel(trans_update);

    // WriteOBJ(m_curMesh3D, "output/trans_3d_" + name() + ".obj");
    // WriteOBJ(m_curMesh2D, "output/trans_2d_" + name() + ".obj");
    // WriteOBJ(m_defMesh2D, "output/trans_2d_def_" + name() + ".obj");
    // centroid
    m_centroid += trans_update;
}

void XSubToy::Rotate(int axis, double angle)
{
    if (axis == ROT_AXIS_X)
    {
        RotateX(angle);
    }
    else if (axis == ROT_AXIS_Y)
    {
        RotateY(angle);
    }
    else if (axis = ROT_AXIS_Z)
    {
        RotateZ(angle);
    }
}

void XSubToy::Rotate(const Eigen::Matrix3d &R)
{

    // lines
    TranslateLine(m_sketchSpline.P, -m_centroid);
    RotateLine(m_sketchSpline.P, R);
    TranslateLine(m_sketchSpline.P, m_centroid);

    TranslateLine(m_sketchPolyLine, -m_centroid);
    RotateLine(m_sketchPolyLine, R);
    TranslateLine(m_sketchPolyLine, m_centroid);

    // meshes
    m_curMesh3D.Translate(-m_centroid);
    m_curMesh3D.Rotate(R);
    m_curMesh3D.Translate(m_centroid);

    m_curMesh2DTemp.Translate(-m_centroid);
    m_curMesh2DTemp.Rotate(R);
    m_curMesh2DTemp.Translate(m_centroid);

    m_curMesh2D.Translate(-m_centroid);
    m_curMesh2D.Rotate(R);
    m_curMesh2D.Translate(m_centroid);
    m_defMesh2D = m_curMesh2D;

    // skeleton
    TranslateSSkel(-m_centroid);
    RotateSSkel(R);
    TranslateSSkel(m_centroid);

    // update m_rotToXY
    SetRotToXY();
    SetSketchPolyLineOnXY();

    // WriteOBJ(m_curMesh3D, "output/rotate_3d" + name() + ".obj");
#ifdef ROT_DEBUG
    WriteOBJ(m_curMesh3D, "output/rotate_3d_centroid_" + name() + ".obj");
    WriteOBJ(m_curMesh2D, "output/rotate_2d_centroid_" + name() + ".obj");
    WriteOBJ(m_defMesh2D, "output/rotate_2d_centroid_def_" + name() + ".obj");

    TMesh tmp = m_curMesh2D;
    tmp.Translate(-m_centroid);
    tmp.Rotate(m_rotToXY);
    WriteOBJ(tmp, "output/reverse_rotate_2d0_" + name() + ".obj");
    M_DEBUG << "m_rotToXY=\n"
            << m_rotToXY << endl;
    tmp.Rotate(m_rotToXY.transpose());
    WriteOBJ(tmp, "output/rotate_2d0_" + name() + ".obj");

    Eigen::Matrix3d rotToXY;
    GetRotToXYFromPlane(rotToXY);
    TMesh tmp1 = m_curMesh2DTemp;
    tmp1.Translate(-m_centroid);
    tmp1.Rotate(rotToXY);
    WriteOBJ(tmp1, "output/reverse_rotate_2d1_" + name() + ".obj");
    M_DEBUG << "rotToXT=\n"
            << rotToXY << endl;
    tmp1.Rotate(rotToXY.transpose());
    WriteOBJ(tmp1, "output/rotate_2d1_" + name() + ".obj");

    TMesh sketchPolyLineMesh;
    vector<Pnt3> contPts;
    Get2DDiscreteLine(m_sketchPolyLineOnXY, contPts, 3, 1);
    bool flag = Triangulation(contPts, sketchPolyLineMesh, Constrained);
    WriteOBJ(sketchPolyLineMesh, "output/rotate_polyline_" + name() + ".obj");
#endif
}

void XSubToy::InitContour(const std::vector<Pnt3> &ct_uniPts3)
{
    m_sketchSpline.Clear();
    m_sketchPolyLines.clear();
    m_sketchPolyLine.clear();

    m_sketchSpline.ChangeMode(CSpline::SPLMODE_CLOSED_SPLINE);
    for (const Pnt3 &p : ct_uniPts3)
    {
        m_sketchSpline.AddCtrlPoint(p);
    }
    CSketchManager::ConvertSplineToPolyLines(m_sketchSpline, m_sketchPolyLines);
    CSketchManager::GetPolyLine(m_sketchPolyLines, m_sketchPolyLine);
}

void XSubToy::SetCentroid()
{
    m_centroid.setZero();
    for (const Pnt3 &p : m_sketchPolyLine)
    {
        m_centroid += p;
    }
    m_centroid /= static_cast<double>(m_sketchPolyLine.size());
    // M_DEBUG << name() << " SetCentroid " << m_centroid.transpose() << endl;
}

void XSubToy::GetPlaneFromTriag(const std::vector<Pnt3> &pts /*closed polygon*/,
                                Plane &plane)
{
    double tot_len = CalPolygonLength(pts);
    double tri_len = tot_len / 3.0;
    int i1, i2;
    int n = static_cast<int>(pts.size());
    double len = 0;
    for (i1 = 1; i1 < n; ++i1)
    {
        len += (pts[i1] - pts[i1 - 1]).norm();
        if (len > tri_len)
            break;
    }
    len = 0;
    for (i2 = i1 + 1; i2 < n; ++i2)
    {
        len += (pts[i2] - pts[i2 - 1]).norm();
        if (len > tri_len)
            break;
    }
    const Pnt3 &p0 = pts[0];
    const Pnt3 &p1 = pts[i1];
    const Pnt3 &p2 = pts[i2];
    plane = triag_to_plane(p0, p1, p2);
}

void XSubToy::GetPlaneFromLSQ(const std::vector<Pnt3> &pts, Plane &res)
{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_3 Point_3;
    typedef K::Vector_3 Vector_3;
    typedef K::Plane_3 Plane_3;

    std::vector<Point_3> points_set;
    points_set.reserve(pts.size());
    for (const Pnt3 &p : pts)
    {
        points_set.push_back(Point_3(p[0], p[1], p[2]));
    }

    Plane_3 plane;
    Point_3 center_of_mass;
    CGAL::linear_least_squares_fitting_3(points_set.begin(), points_set.end(), plane, center_of_mass,
                                         CGAL::Dimension_tag<0>());
    Vector_3 nor = plane.orthogonal_vector();
    res = Plane(nor[0], nor[1], nor[2], plane.d());
}

void XSubToy::GetRotToXYFromPlane(Eigen::Matrix3d &R)
{
    // M_DEBUG << "GetRotToXYFromPlane" << endl;
    Plane plane, plane1;
    Eigen::Matrix3d tmpR;
    GetPlaneFromTriag(m_sketchPolyLine, plane);
    GetRotMatToXYPlane(plane.normal(), R);

    // GetPlaneFromLSQ(m_sketchPolyLine, plane1);
    // GetRotMatToXYPlane(plane1.normal(), R);
    // M_DEBUG << "plane=" << plane << endl;
    // M_DEBUG << "plane1=" << plane << endl;
}

void XSubToy::SetRotToXY()
{
    GetRotToXYFromPlane(m_rotToXY);
    // M_DEBUG << name() << " SetRotToXY\n"
    //         << m_rotToXY.transpose() << endl;
}

void XSubToy::SetSketchPolyLineOnXY()
{
    // m_sketchPolyLineOnXY.clear();
    // m_sketchPolyLineOnXY.reserve(m_sketchPolyLine.size());
    // for (const Pnt3 &p : m_sketchPolyLine)
    // {
    //     m_sketchPolyLineOnXY.push_back(m_rotToXY * (p - m_centroid));
    //     // M_DEBUG << "m_sketchPolyLineOnXY " << m_sketchPolyLineOnXY.back().transpose() << endl;
    // }
    MapToXY(m_sketchPolyLine, m_sketchPolyLineOnXY);
}

void XSubToy::MapToXY(const std::vector<Pnt3> &pts3d, std::vector<Pnt3> &ptsXY)
{
    ptsXY.clear();
    ptsXY.reserve(pts3d.size());
    for (const Pnt3 &p : pts3d)
    {
        ptsXY.push_back(m_rotToXY * (p - m_centroid));
        // M_DEBUG << "m_sketchPolyLineOnXY " << m_sketchPolyLineOnXY.back().transpose() << endl;
    }
}

void XSubToy::ProcessSelfIsectSketchLine()
{
    TMesh meshTmp;
    vector<Pnt3> contPts;
    Get2DDiscreteLine(m_sketchPolyLineOnXY, contPts, 3, 1);

    bool flag = Triangulation(contPts, meshTmp, Constrained);
    meshTmp.Rotate(m_rotToXY.transpose());
    meshTmp.Translate(m_centroid);

    vector<Pnt3> bnd_pts, ct_simPts3;
    meshTmp.FindBoundary(bnd_pts);
    SimplifyPolyLine(bnd_pts, ct_simPts3, 3);

    InitContour(ct_simPts3);
    SetCentroid();
    SetRotToXY();
    SetSketchPolyLineOnXY();
}

void XSubToy::TranslateSSkel(const Vec3 &trans_update)
{
    m_rawSSkel.translate(trans_update);
    m_sskel.translate(trans_update);
    TranslateBranches(trans_update);
}

void XSubToy::RotateSSkel(const Eigen::Matrix3d &R)
{
    m_rawSSkel.rotate(R);
    m_sskel.rotate(R);
    RotateBranches(R);
}

void XSubToy::ScaleSSkel(double scl)
{
    m_rawSSkel.scale(scl);
    m_sskel.scale(scl);
    ScaleBranches(scl);
}

void XSubToy::TranslateBranches(const Vec3 &trans_update)
{
    for (auto bit = branches.begin(); bit != branches.end(); ++bit)
    {
        bit->translate(trans_update);
    }
}

void XSubToy::RotateBranches(const Eigen::Matrix3d &R)
{
    for (auto bit = branches.begin(); bit != branches.end(); ++bit)
    {
        bit->rotate(R);
    }
}

void XSubToy::ScaleBranches(double scl)
{
    for (auto bit = branches.begin(); bit != branches.end(); ++bit)
    {
        bit->scale(scl);
    }
}

void XSubToy::RotateX(double angle)
{
    Eigen::Matrix3d R;
    GetRotMatX(angle, R);
    Rotate(R);
}

void XSubToy::RotateY(double angle)
{
    Eigen::Matrix3d R;
    GetRotMatY(angle, R);
    Rotate(R);
}

void XSubToy::RotateZ(double angle)
{
    Eigen::Matrix3d R;
    GetRotMatZ(angle, R);
    Rotate(R);
}

bool XSubToy::Get2DDiscreteLine(const vector<Pnt2> &vec2Pts, vector<Pnt2> &vec2Pts_new, int iPixel, double pixel_dis)
{
    if (vec2Pts.size() == 0)
        return false;
    float fLenSum = CalLineLength(vec2Pts);
    float fDisUnit = pixel_dis * iPixel;
    int iPolyNum = int(fLenSum / fDisUnit + 0.5);
    if (iPolyNum < 10)
    {
        fDisUnit = fLenSum / 10;
    }

    // M_DEBUG << " iPolyNum=" << iPolyNum << " fLenSum=" << fLenSum << " fDisUnit=" << fDisUnit << endl;
    DiscreteLineByLen(vec2Pts, fLenSum, fDisUnit, vec2Pts_new);
    if ((vec2Pts_new.back() - vec2Pts_new.front()).norm() < 1e-3)
        vec2Pts_new.pop_back();
    return true;
}

bool XSubToy::Get2DDiscreteLine(const vector<Pnt3> &vec2Pts, vector<Pnt3> &vec2Pts_new, int iPixel, double pixel_dis)
{
    if (vec2Pts.size() == 0)
        return false;
    float fLenSum = CalLineLength(vec2Pts);
    float fDisUnit = pixel_dis * iPixel;
    int iPolyNum = int(fLenSum / fDisUnit + 0.5);
    if (iPolyNum < 10)
    {
        fDisUnit = fLenSum / 10;
    }

    // M_DEBUG << " iPolyNum=" << iPolyNum << " fLenSum=" << fLenSum << " fDisUnit=" << fDisUnit << endl;
    DiscreteLineByLen(vec2Pts, fLenSum, fDisUnit, vec2Pts_new);
    if ((vec2Pts_new.back() - vec2Pts_new.front()).norm() < 1e-3)
        vec2Pts_new.pop_back();
    return true;
}

bool XSubToy::TriangulateCoarse(const vector<Pnt3> &contPts)
{
    if (IsLineSelfIntersect(contPts))
    {
        cout << "Line self-intersect!" << endl;
        return false;
    }
    if (m_curMesh2DTemp.num_vertices() > 0)
        m_curMesh2DTemp.clear();

    bool flag = Triangulation(contPts, m_curMesh2DTemp, Constrained);
    WritePly(m_curMesh2DTemp, "output/m_curMesh2DTemp" + to_string(m_toyID) + ".ply");
    return flag;
}

bool XSubToy::TriangulateFine(const vector<Pnt3> &contPts, float max_triArea)
{
    bool flag = Triangulation(contPts, m_curMesh2D, Constrained_Delaunay, max_triArea);
    WritePly(m_curMesh2D, "output/m_curMesh2D" + to_string(m_toyID) + ".ply");
    return flag;
}

bool XSubToy::CalChordalAxisLines()
{
    if (m_curMesh2DTemp.num_vertices() <= 0)
        return false;
    m_curMesh2DTemp.GetChordalAxisLines(m_vAxisLines);
    return true;
}

bool XSubToy::CalChordalAxisLineCylinder()
{
    if (m_curMesh2DTemp.num_vertices() <= 0)
        return false;
    m_curMesh2DTemp.GetChordalAxisLinesCylinder(m_vAxisLines);
    return true;
}

bool XSubToy::CalChordalAxisLineThinning()
{
    std::vector<std::vector<Pnt3>> lines;
    cal_medial_axis_from_polygon(m_sketchPolyLine, lines);
    M_DEBUG << " CalChordalAxisLineThinning";

    for (int i = 0; i < lines.size(); i++)
    {
        std::vector<Pnt3> ln0, ln;
        DiscreteLineByLen(lines[i], CalLineLength(lines[i]), 3, ln0);
        SmoothLine(ln0, ln); //ToDO: move to 'construct bones'
        int n = ln.size();
        C3DLine cln;
        cln.Setcpt(ln[0], 0);
        cln.Setcpt(ln[n - 1], 1);
        for (int i = 1; i <= n - 2; i++)
        {
            cln.Setipt(ln[i]);
        }
        m_vAxisLines.push_back(cln);
    }
    return true;
}

void XSubToy::DeformLBS()
{
    using namespace Eigen;
    MatrixXi F = m_defMesh2D.GetFMat();
    // M_DEBUG << " DeformLBS " << name() << endl;
    // M_DEBUG << " m_skinT " << name() << endl;
    // cout << m_skinT.transpose() << endl;
    MatrixXd V = m_skinM * m_skinT.transpose();
    // igl::writePLY("output/" + name() + "_deformed2D.ply", V, F);
    m_defMesh2D.CreateHalfedgeMesh(V, F);
}

bool XSubToy::CalSkelJoints(float tol, double z_depth)
{
    if (!m_autoSkel)
        return false;

    using namespace Eigen;
    std::vector<std::vector<Pnt3>> lines;
    cal_skel_from_polygon(m_sketchPolyLine, lines, tol);
    // lines.resize(6);
    // lines[0].push_back(Pnt3(0, 200, 0));
    // lines[0].push_back(Pnt3(0, 100, 0));
    // lines[0].push_back(Pnt3(0, 0, 0));
    // lines[1].push_back(Pnt3(200, 0, 0));
    // lines[1].push_back(Pnt3(100, 0, 0));
    // lines[1].push_back(Pnt3(0, 0, 0));
    // lines[2].push_back(Pnt3(0, 0, 0));
    // lines[2].push_back(Pnt3(0, -100, 0));
    // lines[2].push_back(Pnt3(0, -200, 0));
    // lines[3].push_back(Pnt3(200, 0, 0));
    // lines[3].push_back(Pnt3(200, -100, 0));
    // lines[3].push_back(Pnt3(200, -200, 0));
    // lines[4].push_back(Pnt3(300, 0, 0));
    // lines[4].push_back(Pnt3(200, 0, 0));
    // lines[5].push_back(Pnt3(0, 0, 0));
    // lines[5].push_back(Pnt3(-200, 0, 0));

    // if there are multiple medial axis, delete overlapping (very close) points
    // 1. construct Adjacent matrix for EndPoint
    // 2. add HubPoint to m_joints and record their indices in m_joints
    // 3. add other points in simplified medial axis (lines) to m_joints and construct corresponding m_bones
    if (lines.size() > 1)
    {
        Float len = 0;
        int num_bones = 0;
        for (int i = 0; i < lines.size(); i++)
        {
            len += CalLineLength(lines[i]);
            num_bones += (lines.size() - 1);
            cout << __FILE__ << " " << __LINE__ << " line" << i << " length=" << CalLineLength(lines[i]) << endl;
        }
        Float avg_len = num_bones > 0 ? len / num_bones : len;
        Float thresh = avg_len / 3.0;
        if (debug)
            cout << __FILE__ << " " << __LINE__ << " len_sum=" << len << " thresh=" << thresh << endl;

        // 1. construct Adjacent matrix for EndPoint
        int num_endPts = lines.size() * 2;
        Eigen::MatrixXi adjMat = MatrixXi::Zero(num_endPts, num_endPts);
        if (debug)
            cout << __FILE__ << " " << __LINE__ << " construct Adjacent matrix........................." << endl;
        for (int i = 0; i < lines.size(); i++)
        {
            for (int j = 0; j < lines.size(); j++)
            {
                if (i == j)
                    continue;
                double dis_ist_jst = (lines[i][0] - lines[j][0]).norm();
                double dis_ist_jend = (lines[i][0] - lines[j][lines[j].size() - 1]).norm();
                double dis_iend_jst = (lines[i][lines[i].size() - 1] - lines[j][0]).norm();
                double dis_iend_jend = (lines[i][lines[i].size() - 1] - lines[j][lines[j].size() - 1]).norm();

                if (dis_ist_jst < thresh)
                {
                    adjMat(2 * i, 2 * j) = 1;
                    adjMat(2 * j, 2 * i) = 1;
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " st_st line" << i << " line" << j << endl;
                }
                else if ((lines[i][0] - lines[j][lines[j].size() - 1]).norm() < thresh)
                {
                    adjMat(2 * i, 2 * j + 1) = 1;
                    adjMat(2 * j + 1, 2 * i) = 1;
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " st_end line" << i << " line" << j << endl;
                }
                else if ((lines[i][lines[i].size() - 1] - lines[j][0]).norm() < thresh)
                {
                    adjMat(2 * i + 1, 2 * j) = 1;
                    adjMat(2 * j, 2 * i + 1) = 1;
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " end_st line" << i << " line" << j << endl;
                }
                else if ((lines[i][lines[i].size() - 1] - lines[j][lines[j].size() - 1]).norm() < thresh)
                {
                    adjMat(2 * i + 1, 2 * j + 1) = 1;
                    adjMat(2 * j + 1, 2 * i + 1) = 1;
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " end_end line" << i << " line" << j << endl;
                }
            }
            if (debug)
                cout << __FILE__ << " " << __LINE__ << " line" << i << " st adj to=" << adjMat.row(2 * i).sum()
                     << " end adj to=" << adjMat.row(2 * i + 1).sum() << endl;
        }
        // Assume line i is adjcent to at least one line
        for (int i = 0; i < lines.size(); i++)
        {
            if (adjMat.row(2 * i).sum() == 0 && adjMat.row(2 * i + 1).sum() == 0)
            {
                // find nearest end point
                double min_dis = std::numeric_limits<double>::max();
                bool isSt = false; // whether the closest pair is i's start point or end point
                LinePoint clo;     // closest j end point
                for (int j = 0; j < lines.size(); j++)
                {
                    if (i == j)
                        continue;
                    double dis_ist_jst = (lines[i][0] - lines[j][0]).norm();
                    double dis_ist_jend = (lines[i][0] - lines[j][lines[j].size() - 1]).norm();
                    double dis_iend_jst = (lines[i][lines[i].size() - 1] - lines[j][0]).norm();
                    double dis_iend_jend = (lines[i][lines[i].size() - 1] - lines[j][lines[j].size() - 1]).norm();
                    if (dis_ist_jst < min_dis)
                    {
                        min_dis = dis_ist_jst;
                        clo.line_idx = j;
                        clo.pt_idx = 0;
                        isSt = true;
                    }
                    if (dis_ist_jend < min_dis)
                    {
                        min_dis = dis_ist_jend;
                        clo.line_idx = j;
                        clo.pt_idx = lines[j].size() - 1;
                        isSt = true;
                    }
                    if (dis_iend_jst < min_dis)
                    {
                        min_dis = dis_iend_jst;
                        clo.line_idx = j;
                        clo.pt_idx = 0;
                        isSt = false;
                    }
                    if (dis_iend_jend < min_dis)
                    {
                        min_dis = dis_iend_jend;
                        clo.line_idx = j;
                        clo.pt_idx = lines[j].size() - 1;
                        isSt = false;
                    }
                }

                if (isSt)
                {
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " st-line" << i << " end-line" << clo.line_idx << "," << clo.pt_idx << endl;
                    adjMat(2 * i, 2 * clo.line_idx + (clo.pt_idx != 0)) = 1;
                    adjMat(2 * clo.line_idx + (clo.pt_idx != 0), 2 * i) = 1;
                }
                else
                {
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " st-line" << i << " end-line" << clo.line_idx << "," << clo.pt_idx << endl;
                    adjMat(2 * i + 1, 2 * clo.line_idx + (clo.pt_idx != 0)) = 1;
                    adjMat(2 * clo.line_idx + (clo.pt_idx != 0), 2 * i + 1) = 1;
                }
            }
        }
        if (debug)
        {
            cout << __FILE__ << " " << __LINE__ << " construct Adjacent matrix.........................End" << endl;
            cout << adjMat << endl;
            cout << "..............................." << endl;
        }

        std::map<LinePoint, bool> jointAdded;
        std::map<LinePoint, int> jointMap; // value: indices in m_joints
        // for HubPoint, if any point in the hub is added to m_joints, reuse the old jointMap[LinePoint in cluster]
        auto findJointIdx = [&](const LinePoint &query)
        {
            // front end point has offset 0, back end point has offset 1 in adjMat
            int r = (2 * query.line_idx) + (query.pt_idx != 0);
            // cout << __FILE__ << " " << __LINE__ << " findJointIdx for line" << query.line_idx << " point" << query.pt_idx << endl;
            for (int c = 0; c < adjMat.cols(); c++)
            {
                LinePoint adjP(c / 2, (c % 2 == 0) ? 0 : (lines[c / 2].size() - 1));
                if (adjMat(r, c) && jointAdded[adjP])
                {
                    // cout << " found because line" << adjP.line_idx << " pt" << adjP.pt_idx << " is already added" << endl;
                    return jointMap[adjP];
                }
            }
            return -1;
        };
        auto calCenter = [&](const LinePoint &query) -> Pnt3
        {
            int lidx = query.line_idx;
            int pidx = query.pt_idx;
            int r = (2 * query.line_idx) + (query.pt_idx != 0);
            assert(adjMat.row(r).sum() >= 1 && "Must have at least one adjacent endpoints");
            Pnt3 cent = lines[lidx][pidx];
            int num_adj = 0;
            for (int c = 0; c < adjMat.cols(); c++)
            {
                if (adjMat(r, c) == 1)
                {
                    cent += lines[c / 2][(c % 2 == 0) ? 0 : (lines[c / 2].size() - 1)];
                    num_adj++;
                }
            }
            cent /= (double)(num_adj + 1);
            // cout << __FILE__ << " " << __LINE__ << " line" << lidx << " p" << pidx << " cent=" << cent.transpose();
            return cent;
        };

        // 2. add HubPoint to m_joints and record their indices in m_joints
        for (int i = 0; i < lines.size(); i++)
        {
            LinePoint stP(i, 0), endP(i, lines[i].size() - 1);
            if (debug)
                cout << __FILE__ << " " << __LINE__ << " line" << i << " st=" << adjMat.row(2 * i).sum() << " end=" << adjMat.row(2 * i + 1).sum() << endl;
            if (adjMat.row(2 * i).sum() >= 1 && adjMat.row(2 * i + 1).sum() == 0)
            {
                // start point
                int jointIdx = findJointIdx(stP);
                if (jointIdx == -1)
                {
                    Pnt3 cent = calCenter(stP);
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " add new Joint " << cent.transpose() << endl;
                    m_joints.emplace_back(m_joints.size(), cent);
                    jointMap[stP] = m_joints.size() - 1;
                    jointAdded[stP] = true;
                }
                else
                {
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " found old Joint " << jointIdx << endl;
                    jointMap[stP] = jointIdx;
                    jointAdded[stP] = true;
                }
            }
            else if (adjMat.row(2 * i + 1).sum() >= 1 && adjMat.row(2 * i).sum() == 0)
            {
                // end point
                int jointIdx = findJointIdx(endP);
                if (jointIdx == -1)
                {
                    Pnt3 cent = calCenter(endP);
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " add new Joint " << cent.transpose() << endl;
                    m_joints.emplace_back(m_joints.size(), cent);
                    jointMap[endP] = m_joints.size() - 1;
                    jointAdded[endP] = true;
                }
                else
                {
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " found old Joint " << jointIdx << endl;
                    jointMap[endP] = jointIdx;
                    jointAdded[endP] = true;
                }
            }
            else if (adjMat.row(2 * i + 1).sum() >= 1 && adjMat.row(2 * i).sum() >= 1)
            {
                // start point
                int stJointIdx = findJointIdx(stP);
                if (stJointIdx == -1)
                {
                    Pnt3 cent = calCenter(stP);
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " add new Joint " << cent.transpose() << endl;
                    m_joints.emplace_back(m_joints.size(), cent);
                    jointMap[stP] = m_joints.size() - 1;
                    jointAdded[stP] = true;
                }
                else
                {
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " found old Joint " << stJointIdx << endl;
                    jointMap[stP] = stJointIdx;
                    jointAdded[stP] = true;
                }

                int endJointIdx = findJointIdx(endP);
                if (endJointIdx == -1)
                {
                    Pnt3 cent = calCenter(endP);
                    if (debug)
                        cout << __FILE__ << " " << __LINE__ << " add new Joint " << cent.transpose() << endl;
                    m_joints.emplace_back(m_joints.size(), cent);
                    jointMap[endP] = m_joints.size() - 1;
                    jointAdded[endP] = true;
                }
                else
                {
                    jointMap[endP] = endJointIdx;
                    jointAdded[endP] = true;
                }
            }
            else
            {
                assert(false);
            }
        }
        if (debug)
            cout << __FILE__ << " " << __LINE__ << " now joint becomes " << m_joints.size() << endl;
        // 3. add other points in simplified medial axis (lines) to m_joints and construct corresponding m_bones
        if (debug)
            cout << __FILE__ << " " << __LINE__ << " build connection...." << endl;
        std::vector<Vector2i> bones;
        for (int i = 0; i < lines.size(); i++)
        {
            if (debug)
                cout << "...line" << i << endl;
            LinePoint stP(i, 0), endP(i, lines[i].size() - 1);
            if (adjMat.row(2 * i).sum() >= 1 && adjMat.row(2 * i + 1).sum() == 0) // start point is already added to m_joints
            {
                int j = 1;
                m_joints.emplace_back(m_joints.size(), lines[i][j]);

                int prevPIdx = jointMap[stP];
                int curPIdx = m_joints.size() - 1;
                bones.push_back(Vector2i(prevPIdx, curPIdx));
                if (debug)
                    cout << "startPoint is hub, add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                m_joints[prevPIdx].add_adj(curPIdx);
                m_joints[curPIdx].add_adj(prevPIdx);
                j++;
                for (; j < lines[i].size(); j++)
                {
                    m_joints.emplace_back(m_joints.size(), lines[i][j]);

                    prevPIdx = curPIdx;
                    curPIdx = m_joints.size() - 1;
                    bones.push_back(Vector2i(prevPIdx, curPIdx));
                    if (debug)
                        cout << "j=" << j << " add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                    m_joints[prevPIdx].add_adj(curPIdx);
                    m_joints[curPIdx].add_adj(prevPIdx);
                }
            }
            else if (adjMat.row(2 * i + 1).sum() >= 1 && adjMat.row(2 * i).sum() == 0) // start point is already added to m_joints
            {
                int j = lines[i].size() - 2;
                m_joints.emplace_back(m_joints.size(), lines[i][j]);

                int prevPIdx = jointMap[endP];
                int curPIdx = m_joints.size() - 1;
                bones.push_back(Vector2i(prevPIdx, curPIdx));
                if (debug)
                    cout << "endPoint is hub, add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                m_joints[prevPIdx].add_adj(curPIdx);
                m_joints[curPIdx].add_adj(prevPIdx);
                j--;
                for (; j >= 0; j--)
                {
                    m_joints.emplace_back(m_joints.size(), lines[i][j]);

                    prevPIdx = curPIdx;
                    curPIdx = m_joints.size() - 1;
                    bones.push_back(Vector2i(prevPIdx, curPIdx));
                    if (debug)
                        cout << "j=" << j << " add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                    m_joints[prevPIdx].add_adj(curPIdx);
                    m_joints[curPIdx].add_adj(prevPIdx);
                }
            }
            else if (adjMat.row(2 * i).sum() >= 1 && (adjMat.row(2 * i + 1).sum() >= 1)) // both start and end point is already added to m_joints
            {
                int j = 1;
                m_joints.emplace_back(m_joints.size(), lines[i][j]);
                if (debug)
                    cout << "select startPoint is hub, add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;

                int prevPIdx = jointMap[stP];
                int curPIdx = m_joints.size() - 1;
                bones.push_back(Vector2i(prevPIdx, curPIdx));
                m_joints[prevPIdx].add_adj(curPIdx);
                m_joints[curPIdx].add_adj(prevPIdx);
                j++;
                for (; j < lines[i].size() - 1; j++)
                {
                    m_joints.emplace_back(m_joints.size(), lines[i][j]);

                    prevPIdx = curPIdx;
                    curPIdx = m_joints.size() - 1;
                    bones.push_back(Vector2i(prevPIdx, curPIdx));
                    if (debug)
                        cout << "j=" << j << " add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                    m_joints[prevPIdx].add_adj(curPIdx);
                    m_joints[curPIdx].add_adj(prevPIdx);
                }

                prevPIdx = m_joints.size() - 1;
                curPIdx = jointMap[endP];
                bones.push_back(Vector2i(prevPIdx, curPIdx));
                if (debug)
                    cout << "j=" << j << " add bone " << bones.back().transpose() << " bones=" << bones.size() << endl;
                m_joints[prevPIdx].add_adj(curPIdx);
                m_joints[curPIdx].add_adj(prevPIdx);
            }
            else
            {
                assert(false);
            }
            if (debug)
                cout << "-----------------------\n";
        }

        m_bones.resize(bones.size(), 2);
        for (int i = 0; i < bones.size(); i++)
        {
            m_bones(i, 0) = bones[i](0);
            m_bones(i, 1) = bones[i](1);
        }
        cout << __FILE__ << " " << __LINE__ << " joints=" << m_joints.size() << " bones=" << m_bones.rows() << endl;
    }
    else
    {
        m_joints.emplace_back(m_joints.size(), lines[0][0]);
        m_bones.resize(lines[0].size() - 1, 2);
        cout << __FILE__ << " " << __LINE__ << " m_bones=" << m_bones.rows() << ", " << m_bones.cols() << endl;
        for (int j = 1; j < lines[0].size(); j++)
        {
            m_joints.emplace_back(m_joints.size(), lines[0][j]);
            m_bones(j - 1, 0) = j - 1;
            m_bones(j - 1, 1) = j;
            m_joints[j - 1].add_adj(j);
            m_joints[j].add_adj(j - 1);
        }
    }
    return true;
}

void XSubToy::ConstructSkeleton(Skeleton<Bone> *skel, double z_depth)
{
    if (!m_autoSkel)
        return;

    if (isSkeletonCreated)
        return;

    // set m_boneRoot as the one with most adjacent point
    size_t maxNumAdj = std::numeric_limits<size_t>::min();
    int rootJIdx = -1;
    for (int i = 0; i < m_joints.size(); i++)
    {
        size_t numAdj = m_joints[i].adj.size();
        cout << __FILE__ << " " << __LINE__ << " joint " << i << " adj=" << m_joints[i].adj.size() << endl;
        if (numAdj > maxNumAdj)
        {
            maxNumAdj = numAdj;
            rootJIdx = i;
        }
    }
    cout << __FILE__ << " " << __LINE__ << " maxNumAdj=" << maxNumAdj << endl;
    // set joints' z-depth
    for (int i = 0; i < m_joints.size(); i++)
    {
        Pnt3 &pos = m_joints[i].pos;
        pos.z() = z_depth;
        cout << __FILE__ << " " << __LINE__ << " " << name() << ": " << pos.transpose() << endl;
    }
    if (maxNumAdj <= 2) // A straight line
    {
        // if subToy's bbox's W > H, select right-most as root
        // else select top-most as root
        auto &sJoints = m_joints;
        auto &sBones = m_bones;
        Bone *root = new Bone(skel, NULL, sJoints[sBones(0, 0)].pos);
        skel->roots.insert(skel->roots.end(), root);
        m_boneRoot = root;
        Bone *parent = root;
        for (int i = 0; i < sBones.rows(); i++)
        {
            Bone *child = new Bone(skel, parent, sJoints[sBones(i, 1)].pos - sJoints[sBones(i, 0)].pos);
            parent = child;
        }
    }
    else
    {
        assert(maxNumAdj > 2);
        cout << __FILE__ << " " << __LINE__ << " maxNumAdj=" << maxNumAdj << endl;
        std::list<Joint *> Q;
        std::list<Bone *> QB;
        std::map<Joint *, bool> isVisit;
        Joint *rootJoint = &m_joints[rootJIdx];
        Q.push_back(rootJoint);
        Bone *root = new Bone(skel, NULL, rootJoint->pos);
        skel->roots.insert(skel->roots.end(), root);
        m_boneRoot = root;
        cout << __FILE__ << " " << __LINE__ << ".......................................m_boneRoot: " << name() << " " << m_boneRoot->offset.transpose() << endl;
        QB.push_back(root);

        while (!Q.empty())
        {
            Joint *parentJ = Q.front();
            Q.pop_front();
            Bone *parent = QB.front();
            QB.pop_front();

            for (std::vector<int>::iterator it = parentJ->adj.begin();
                 it != parentJ->adj.end(); it++)
            {
                Joint *adjJ = &m_joints[(*it)];
                if (!isVisit[adjJ])
                {
                    Bone *child = new Bone(skel, parent, adjJ->pos - parentJ->pos);
                    Q.push_back(adjJ);
                    QB.push_back(child);
                }
            }
            isVisit[parentJ] = true;
        }
    }

    isSkeletonCreated = true;
}

void XSubToy::ConstructSkeleton(Skeleton<Bone> *skel, const Pnt3 &attachPoint, double z_depth)
{
    if (!m_autoSkel)
        return;

    if (isSkeletonCreated)
        return;

    // set m_boneRoot as the one nearest to attachPoint
    // change root's position as attachPoint
    int rootJIdx = -1;
    double minDis = std::numeric_limits<double>::max();
    for (int i = 0; i < m_joints.size(); i++)
    {
        double dis = (m_joints[i].pos - attachPoint).norm();
        if (dis < minDis)
        {
            minDis = dis;
            rootJIdx = i;
        }
    }
    // cout << __FILE__ << " " << __LINE__ << " minDis=" << minDis << " rootJIdx=" << rootJIdx << endl;
    // cout << __FILE__ << " " << __LINE__ << " m_joints=" << m_joints.size() << endl;
    // for (int i = 0; i < m_joints.size(); i++)
    // {
    //     cout << m_joints[i];
    // }
    // set joints' z-depth
    for (int i = 0; i < m_joints.size(); i++)
    {
        Pnt3 &pos = m_joints[i].pos;
        pos.z() = z_depth;
    }

    assert(rootJIdx >= 0);
    m_joints[rootJIdx].pos = attachPoint;
    m_joints[rootJIdx].pos.z() = z_depth;

    // starting from root, adding children in breath-first manner
    std::list<Joint *> Q;
    std::list<Bone *> QB;
    std::map<Joint *, bool> isVisit;
    Joint *rootJoint = &m_joints[rootJIdx];
    Q.push_back(rootJoint);
    Bone *root = new Bone(skel, NULL, rootJoint->pos);

    m_boneRoot = root;
    m_attachBone = new Bone(skel, NULL, rootJoint->pos);
    skel->roots.insert(skel->roots.end(), m_attachBone);
    m_attachBone1 = new Bone(skel, m_attachBone, Vec3(0, 0, 0), m_toyID);
    cout << __FILE__ << " " << __LINE__ << ".......................................m_boneRoot: " << name() << " " << m_boneRoot->offset.transpose() << endl;
    QB.push_back(root);

    while (!Q.empty())
    {
        Joint *parentJ = Q.front();
        Q.pop_front();
        Bone *parent = QB.front();
        QB.pop_front();
        for (std::vector<int>::iterator it = parentJ->adj.begin();
             it != parentJ->adj.end(); it++)
        {
            Joint *adjJ = &m_joints[(*it)];
            if (!isVisit[adjJ])
            {
                Eigen::Vector3d off = adjJ->pos - parentJ->pos;
                Bone *child = new Bone(skel, parent, off);

                // cout << __FILE__ << " " << __LINE__ << " child=\n";
                // child->print();

                Q.push_back(adjJ);
                QB.push_back(child);
            }
        }
        isVisit[parentJ] = true;
    }
    isSkeletonCreated = true;
}

void XSubToy::InflateTeddy(Float height_ratio)
{

    assert(m_curMesh2D.num_vertices() > 0);
    //ToDO: SplitFace_2DMesh(m_curMesh2D);
    m_curMesh2D.RefineLocalMesh();
    WritePly(m_curMesh2D, "output/m_curMesh2D_refineLocalMesh.ply");
    //ToDO: AdjustMeshToBound
    Refine2DMesh(m_curMesh2D, height_ratio); //ToDO: save original 2D edge lengths
    WritePly(m_curMesh2D, "output/m_curMesh2D_refine2DMesh.ply");

    // 3. Get chordal axis line by coarse mesh
    m_curMesh2DTemp.GetChordalAxisLines(m_vAxisLines);
    if (m_vAxisLines.size() == 0)
    {
        return;
    }
    GetElevateValueInToAxisLines(m_curMesh2DTemp, m_vAxisLines);

    // 4. Elevate
    TMesh pMesh3DTemp[2];
    ElevateMesh(m_curMesh2D, pMesh3DTemp[0], m_curMesh2DTemp, m_vAxisLines, height_ratio, true);
    WritePly(pMesh3DTemp[0], "output/pMesh3DTemp_0.ply");
    ElevateMesh(m_curMesh2D, pMesh3DTemp[1], m_curMesh2DTemp, m_vAxisLines, -height_ratio, false);
    WritePly(pMesh3DTemp[1], "output/pMesh3DTemp_1.ply");

    for (Vd vi : pMesh3DTemp[0].vertices())
    {
        if (pMesh3DTemp[0].is_border(vi))
            continue;

        Hd hi = pMesh3DTemp[0].halfedge(vi);
        Pnt3 &p = pMesh3DTemp[0].point(vi);
        if (p.z() < 0.1)
        {
            float zTmp = 0;
            int count = 0;
            for (Vd vj : pMesh3DTemp[0].vertices_around_target(hi))
            {
                zTmp += pMesh3DTemp[0].point(vj).z();
                count++;
            }
            zTmp /= count;
            p = Pnt3(p.x(), p.y(), zTmp);
        }
    }

    for (Vd vi : pMesh3DTemp[1].vertices())
    {
        if (pMesh3DTemp[1].is_border(vi))
            continue;

        Hd hi = pMesh3DTemp[1].halfedge(vi);
        Pnt3 &p = pMesh3DTemp[1].point(vi);
        if (p.z() < 0.1)
        {
            float zTmp = 0;
            int count = 0;
            for (Vd vj : pMesh3DTemp[1].vertices_around_target(hi))
            {
                zTmp += pMesh3DTemp[1].point(vj).z();
                count++;
            }
            zTmp /= count;
            p = Pnt3(p.x(), p.y(), zTmp);
        }
    }

    float fMaxEdgeLen = m_curMesh2DTemp.max_edge_len();

    //------------------------------ Merge & Smooth -----------------------------------------------------
    MergeTMeshes(m_curMesh3D, pMesh3DTemp);
    WritePly(m_curMesh3D, "output/merged.ply");

    float zRef = 0;
    Float len_avg = m_curMesh3D.avg_edge_len();
    vector<Vd> vSmthVtIdx, vSmthVtIdx1;
    for (Vd vd : m_curMesh3D.vertices())
    {
        Pnt3 &p = m_curMesh3D.point(vd);
        if (fabs(p.z() - zRef) > 0.2)
        {
            vSmthVtIdx.push_back(vd);
        }
        vSmthVtIdx1.push_back(vd);
    }
    for (int i = 0; i < 3; i++)
    {
        m_curMesh3D.SmoothenMeshNaive(vSmthVtIdx, 1, len_avg);
        m_curMesh3D.SmoothenMeshEvenFilter(vSmthVtIdx, 1, len_avg);
    }
    WritePly(m_curMesh3D, "output/smooth1.ply");

    m_curMesh3D.RefineLocalMesh();
    WritePly(m_curMesh3D, "output/merged_smth_refineLocalMesh.ply");
    vSmthVtIdx.clear(), vSmthVtIdx1.clear();
    for (Vd vd : m_curMesh3D.vertices())
    {
        Pnt3 &p = m_curMesh3D.point(vd);
        if (fabs(p.z() - zRef) > 0.2)
        {
            vSmthVtIdx.push_back(vd);
        }
        vSmthVtIdx1.push_back(vd);
    }
    for (int i = 0; i < 8; i++)
    {
        m_curMesh3D.SmoothenMeshNaive(vSmthVtIdx, 1, len_avg);
        m_curMesh3D.SmoothenMeshEvenFilter(vSmthVtIdx, 1, len_avg);
    }
    WritePly(m_curMesh3D, "output/merged_smth2.ply");
}

void XSubToy::MergeTMeshes(TMesh &mesh3D, TMesh meshes[])
{
    mesh3D = meshes[0];

    vector<vector<Vd>> faces; // num_face x 3
    map<Vd, Vd> join_v;       // key: border Vd in mesh[0], closest border Vd in mesh[1]
    vector<Vd> v_border0, v_border1;
    for (Vd v : meshes[0].vertices())
    {
        if (meshes[0].is_border(v))
        {
            v_border0.push_back(v);
        }
    }

    for (Vd v : meshes[1].vertices())
    {
        if (meshes[1].is_border(v))
        {
            v_border1.push_back(v);
        }
    }

    for (Vd vi : v_border0)
    {
        for (Vd vj : v_border1)
        {
            if ((meshes[0].point(vi) - meshes[1].point(vj)).norm() < 0.1)
            {
                join_v[vj] = vi;
            }
        }
    }

    std::map<Vd, Vd> added; // key is meshes[1] vd, value is vd in mesh3D
    // std::map<Vd, int> status; // 0 represent from
    for (Fd fd : meshes[1].faces())
    {
        Hd hd = meshes[1].halfedge(fd);
        Hd nxt_hd = meshes[1].next(hd);
        Vd v0 = meshes[1].source(hd);
        Vd v1 = meshes[1].target(hd);
        Vd v2 = meshes[1].target(nxt_hd);
        if (join_v[v0].is_valid())
        {
            v0 = join_v[v0];
        }
        else
        {
            if (added[v0].is_valid())
            {
                v0 = added[v0];
            }
            else
            {
                Vd v0_new = mesh3D.add_vertex(meshes[1].point(v0));
                added[v0] = v0_new;
                v0 = v0_new;
            }
        }

        if (join_v[v1].is_valid())
        {
            v1 = join_v[v1];
        }
        else
        {
            if (added[v1].is_valid())
            {
                v1 = added[v1];
            }
            else
            {
                Vd v1_new = mesh3D.add_vertex(meshes[1].point(v1));
                added[v1] = v1_new;
                v1 = v1_new;
            }
        }

        if (join_v[v2].is_valid())
        {
            v2 = join_v[v2];
        }
        else
        {
            if (added[v2].is_valid())
            {
                v2 = added[v2];
            }
            else
            {
                Vd v2_new = mesh3D.add_vertex(meshes[1].point(v2));
                added[v2] = v2_new;
                v2 = v2_new;
            }
        }
        mesh3D.add_face(v0, v1, v2);
    }
    mesh3D.resize(mesh3D.num_vertices(), mesh3D.num_edges(), mesh3D.num_faces());
}

bool XSubToy::GetElevateValueInToAxisLines(const TMesh &mesh, vector<C3DLine> &lines)
{
    // large triangle has large elevation, small triangle has small elevation
    if (mesh.num_vertices() == 0 || lines.size() == 0)
    {
        return false;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        C3DLine &line = lines[i];
        int iNumSize = line.GetiptCount() + 2;
        float fLenSum = 0;
        line.m_vOff.resize(iNumSize, 0);

        for (int j = 0; j < iNumSize; j++)
        {
            if (j == 0)
            {
                Fd fd = line.Getcf(0);
                Hd hed = mesh.halfedge(fd);
                Hd next_hed = next(hed, mesh);
                Pnt3 vt = line.Getcpt(0);
                Pnt3 vt1 = mesh.point(source(hed, mesh));
                Pnt3 vt2 = mesh.point(target(hed, mesh));
                Pnt3 vt3 = mesh.point(target(next_hed, mesh));
                fLenSum = (vt - vt1).norm() + (vt - vt2).norm() + (vt - vt3).norm();
                line.m_vOff[0] = fLenSum / 3.0;
            }
            else if (j == iNumSize - 1)
            {
                Fd fd = line.Getcf(1);
                if (mesh.GetFType(fd) == JUNCTION)
                {
                    Hd hed = mesh.halfedge(fd);
                    Hd next_hed = next(hed, mesh);
                    Pnt3 vt = line.Getcpt(0);
                    Pnt3 vt1 = mesh.point(source(hed, mesh));
                    Pnt3 vt2 = mesh.point(target(hed, mesh));
                    Pnt3 vt3 = mesh.point(target(next_hed, mesh));
                    float fLenSum = (vt - vt1).norm() + (vt - vt2).norm() + (vt - vt3).norm();
                    line.m_vOff.back() = fLenSum / 3.0;
                }
                else
                {
                    line.m_vOff.back() = 0.0;
                }
            }
            else
            {
                Ed ed = line.Getedge(j - 1);
                Hd hed = halfedge(ed, mesh);
                Pnt3 vt1 = mesh.point(source(hed, mesh));
                Pnt3 vt2 = mesh.point(target(hed, mesh));
                float fLen = (vt1 - vt2).norm() / 2.0;
                line.m_vOff[j] = fLen;
            }
        }
    }
    return true;
}

void XSubToy::Refine2DMesh(TMesh &ms, Float fRatio)
{
    if (fRatio < 0.2) //if the height ratio is so small, do not need to ajdust
    {
        return;
    }

    map<int, Float> oriELen; // original edge length before ajustment
    for (Ed e : ms.edges())
    {
        oriELen[e.idx()] = ms.edge_len(e);
    }

    vector<int> vFlag;
    vFlag.resize(ms.num_vertices(), -1);

    for (Vd vd : ms.vertices())
    {
        if (ms.is_border(vd))
        {
            vFlag[vd.idx()] = 0;
        }
    }

    float len, scl;
    int NUM = 2;
    bool flag;
    int ne;
    for (int k = 0; k < NUM; k++)
    {
        if (k == 0)    // scl parameter can be adjusted
            scl = 0.4; // first/border cirlce
        else if (k == 1)
            scl = 0.8; // second circle
        else if (k == 2)
            scl = 0.6;
        else
            scl = 0.8;
        for (Vd vi : ms.vertices())
        {
            int i = vi.idx();
            Hd hi = ms.halfedge(vi);
            if (vFlag[i] != -1)
                continue;
            flag = false;
            ne = 0;
            Vec3 dis(0., 0., 0.);
            for (Hd hj : ms.halfedges_around_target(hi))
            {
                Vd vj = source(hj, ms);
                int j = vj.idx();
                if (vFlag[j] == k)
                {
                    dis += (ms.point(vj) - ms.point(vi)) * (1. - scl);
                    flag = true;
                    ne++;
                }
            }

            if (flag)
            {
                vFlag[i] = k + 1;

                ms.point(vi) += dis / (Float)ne;
            }
        }

        for (int i = 0; i < 5; i++)
        {
            for (Vd vi : ms.vertices())
            {
                if (vFlag[vi.idx()] != -1)
                    continue;
                Hd hi = ms.halfedge(vi);
                Vec3 dis(0., 0., 0.);
                for (Hd hj : ms.halfedges_around_target(hi))
                {
                    Vd vj = source(hj, ms);
                    Vec3 vij = ms.point(vj) - ms.point(vi);
                    dis += (vij.normalized()) * (ms.edge_len(hj) - oriELen[ms.edge(hj).idx()]);
                }
                dis = dis / 8.;
                ms.point(vi) += dis;
            }
        }
    }
}

void XSubToy::ElevateMesh(const TMesh &meshRef, TMesh &meshOut, TMesh &meshCoarse, vector<C3DLine> &lines, float fScl, bool bIsUpart)
{
    if (lines.size() == 0 || meshRef.num_vertices() == 0)
        return;

    meshOut = meshRef;
    if (!bIsUpart)
    {
        meshOut.ReverseFaceOrientation();
    }

    vector<int> vFaceTp; // store three types of each triangle
    vFaceTp.resize(meshOut.num_vertices(), TERMINAL);
    int iJuncT = -1;
    vector<int> vJuncT;
    float fAreaMax = -1.0e6;

    for (Fd fd : meshCoarse.faces())
    {
        vJuncT.push_back(fd.idx());
    }
    for (int i = 0; i < vJuncT.size(); i++)
    {
        float fArea = meshCoarse.Get3DFArea(vJuncT[i]);
        if (fArea > fAreaMax)
        {
            fAreaMax = fArea;
            iJuncT = vJuncT[i];
        }
    }

    for (Vd vi : meshOut.vertices())
    {
        if (meshOut.is_border(vi))
        {
            continue;
        }

        Fd fd = meshOut.null_face();
        float fOff = 0;
        Pnt3 &pi = meshOut.point(vi);
        fd = meshCoarse.FindLocatedFaceIn2D(pi);
        if (fd.is_valid())
        {
            // interpolate fOff_pi by the intersection point's off on the triangle
            if (meshCoarse.GetFType(fd) == JUNCTION)
            {
                fOff = GetElevateValueInTriangle(pi, meshCoarse, fd, lines);
            }
            else
            {
                fOff = GetElevateValueInSleeveOrTerminalT(pi, meshCoarse, fd, lines);
            }
        }
        else
        {
            // ToDO
            assert(false);
        }
        pi = Pnt3(pi.x(), pi.y(), pi.z() + fOff * fScl);
    }
}

float XSubToy::GetElevateValueInTriangle(const Pnt3 &vt, TMesh &mesh, Fd iFaceID, vector<C3DLine> &lines)
{
    int i = 0, j = 0;
    float fRet = 0;
    //	int iFaceType = face.GetType();
    float fMaxOff12 = 0, fMaxOff23 = 0, fMaxOff31 = 0;

    vector<int> vCurLinesId; //lines pass through current junction face,should be 3 lines
    for (i = 0; i < static_cast<int>(lines.size()); i++)
    {
        if (lines[i].Getcf(0) == iFaceID || lines[i].Getcf(1) == iFaceID)
        {
            vCurLinesId.push_back(i);
        }
    }
    if (int(vCurLinesId.size()) != 3)
    {
        assert(vCurLinesId.size() == 3);
        return fRet;
    }
    C3DLine *pLine12, *pLine23, *pLine31;
    pLine12 = pLine23 = pLine31 = NULL;
    Hd f_hd = mesh.halfedge(iFaceID);
    Vd vd1 = mesh.source(f_hd);
    Vd vd2 = mesh.target(f_hd);
    Vd vd3 = mesh.target(next(f_hd, mesh));
    for (int i = 0; i < vCurLinesId.size(); i++)
    {
        C3DLine &line = lines[vCurLinesId[i]];
        Ed ed = (line.Getcf(0) == iFaceID) ? line.Getedge(0) : line.m_edges.back();
        Vd ev0 = mesh.vertex(ed, 0);
        Vd ev1 = mesh.vertex(ed, 1);
        if ((ev0 == vd1 && ev1 == vd2) || (ev1 == vd1 && ev0 == vd2))
            pLine12 = &line;
        if ((ev0 == vd2 && ev1 == vd3) || (ev1 == vd2 && ev0 == vd3))
            pLine23 = &line;
        if ((ev0 == vd3 && ev1 == vd1) || (ev1 == vd3 && ev0 == vd1))
            pLine31 = &line;
    }

    if (pLine12 != NULL)
    {
        fMaxOff12 = (pLine12->Getcf(0) == iFaceID) ? pLine12->m_vOff[1] : pLine12->m_vOff[pLine12->m_vOff.size() - 2];
    }
    if (pLine23 != NULL)
    {
        fMaxOff23 = (pLine23->Getcf(0) == iFaceID) ? pLine23->m_vOff[1] : pLine23->m_vOff[pLine23->m_vOff.size() - 2];
    }
    if (pLine31 != NULL)
    {
        fMaxOff31 = (pLine31->Getcf(0) == iFaceID) ? pLine31->m_vOff[1] : pLine31->m_vOff[pLine31->m_vOff.size() - 2];
    }

    Pnt3 v1, v2, v3;
    v1 = mesh.point(vd1);
    v2 = mesh.point(vd2);
    v3 = mesh.point(vd3);

    float fGap = float(0.001);
    if ((vt - v1).norm() < fGap || (vt - v2).norm() < fGap || (vt - v3).norm() < fGap)
    {
        return 0;
    }

    Hd h12 = f_hd;
    Hd h23 = next(h12, mesh);
    Hd h31 = next(h23, mesh);
    Ed iEdge;
    Ed iEdge12 = mesh.edge(h12);
    Ed iEdge23 = mesh.edge(h23);
    Ed iEdge31 = mesh.edge(h31);

    vector<Ed> vEdgeIdx;
    vector<float> vOff;
    vector<C3DLine *> vLine;
    C3DLine *pLine = NULL;
    vEdgeIdx.push_back(iEdge12);
    vEdgeIdx.push_back(iEdge23);
    vEdgeIdx.push_back(iEdge31);
    vOff.push_back(fMaxOff12);
    vOff.push_back(fMaxOff23);
    vOff.push_back(fMaxOff31);
    vLine.push_back(pLine12);
    vLine.push_back(pLine23);
    vLine.push_back(pLine31);
    float fOffTmp = 0;
    for (i = 0; i < 2; i++)
    {
        for (j = i + 1; j < 3; j++)
        {
            if (vOff[i] < vOff[j])
            {
                fOffTmp = vOff[i];
                vOff[i] = vOff[j];
                vOff[j] = fOffTmp;

                iEdge = vEdgeIdx[i];
                vEdgeIdx[i] = vEdgeIdx[j];
                vEdgeIdx[j] = iEdge;

                pLine = vLine[i];
                vLine[i] = vLine[j];
                vLine[j] = pLine;
            }
        }
    }

    Ed &minEdge = vEdgeIdx.back();
    v2 = mesh.point(mesh.vertex(minEdge, 0));
    v3 = mesh.point(mesh.vertex(minEdge, 1));
    vector<Vd> vPtIdx;
    vPtIdx.push_back(mesh.vertex(minEdge, 0));
    vPtIdx.push_back(mesh.vertex(minEdge, 1));
    for (Vd v_f : mesh.vertices_around_face(h12))
    {
        if (v_f != vPtIdx[0] && v_f != vPtIdx[1])
        {
            v1 = mesh.point(v_f);
            break;
        }
    }
    float fLenSum12, fLenSum23, fLenSum31;
    fLenSum12 = (v1 - v2).norm() / 2;
    fLenSum23 = (v2 - v3).norm() / 2;
    fLenSum31 = (v3 - v1).norm() / 2;
    fMaxOff12 = (fLenSum12 > fLenSum31) ? vOff[0] : vOff[1];
    fMaxOff31 = (fLenSum31 > fLenSum12) ? vOff[0] : vOff[1];
    fMaxOff23 = vOff.back();

    pLine12 = (fLenSum12 > fLenSum31) ? vLine[0] : vLine[1];
    pLine31 = (fLenSum31 > fLenSum12) ? vLine[0] : vLine[1];
    pLine23 = vLine.back();

    Pnt3 vtMaxOff;
    Pnt3 vtMid12, vtMid31, vtNextMid12, vtNextMid31;
    float fNextMaxOff12, fNextMaxOff31;
    vtMid12 = (v1 + v2) / 2;
    vtMid31 = (v1 + v3) / 2;
    vtNextMid12 = (pLine12->GetiptCount() > 1) ? pLine12->Getipt(1) : pLine12->Getcpt(1);
    vtNextMid31 = (pLine31->GetiptCount() > 1) ? pLine31->Getipt(1) : pLine31->Getcpt(1);
    fNextMaxOff12 = pLine12->m_vOff[2];
    fNextMaxOff31 = pLine31->m_vOff[2];

    bool bFlag1 = false, bFlag2 = false, bFlag3 = false;
    Vec3 vtDir;
    Pnt3 vtInter12, vtInter13, vtInter23, pt1, pt2, pt3;
    vtDir = (v2 - v3).normalized();
    pt1 = vt + vtDir * DIS_INTER;
    pt2 = vt - vtDir * DIS_INTER;
    pt3 = v1 + (vt - v1).normalized() * DIS_INTER;
    bFlag1 = IsTwoLineIntersectIn2D(pt1, pt2, v1, v2);
    bFlag2 = IsTwoLineIntersectIn2D(pt1, pt2, v1, v3);
    bFlag3 = IsTwoLineIntersectIn2D(v1, pt3, v2, v3);
    if (bFlag1 && bFlag2)
    {
        vtInter12 = GetTwoLineInterPt(pt1, pt2, v1, v2);
        vtInter13 = GetTwoLineInterPt(pt1, pt2, v1, v3);
        vtInter23 = GetTwoLineInterPt(v1, pt3, v2, v3);
    }
    else
    {
        //judge whether vt is on line v2v3;
        Vec3 vect1 = (vt - v2).normalized();
        Vec3 vect2 = (v3 - v2).normalized();
        float fDot;
        fDot = fabs(vect1.dot(vect2));
        if (fabs(fDot - 1) < 0.01)
        {
            bFlag1 = bFlag2 = true;
            vtInter12 = v2;
            vtInter13 = v3;
            vtInter23 = GetTwoLineInterPt(v1, pt3, v2, v3);
        }
    }
    if (bFlag1 && bFlag2)
    {
        float fScl1, fScl;
        float fLen12, fLen23, fLen31, fLen, fLenSum, fMaxOff;
        float fOff23;
        fLen12 = (vtInter12 - (v1 + v2) / 2).norm();
        fLen23 = (vtInter23 - (v2 + v3) / 2).norm();
        fLen31 = (vtInter13 - (v1 + v3) / 2).norm();

        fScl1 = (vt - vtInter12).norm() / (vtInter13 - vtInter12).norm();
        fMaxOff = fMaxOff12 * (1 - fScl1) + fMaxOff31 * fScl1;
        float fMaxOffTmp;
        fMaxOffTmp = sqrt(fMaxOff12 * fMaxOff12 * (1 - fScl1) + fMaxOff31 * fMaxOff31 * fScl1);
        fMaxOff = fMaxOffTmp;

        fOff23 = GetElevateValue(fLen23, fLenSum23, fMaxOff23);
        fLen = (vt - v1).norm();
        fLenSum = (vtInter23 - v1).norm() / 2;

        if (fLen < fLenSum)
        {
            fLen = (vt - (v1 + vtInter23) / 2).norm();
            fRet = GetElevateValue(fLen, fLenSum, fMaxOff);
        }
        else
        {
            fLen = (vt - (v1 + vtInter23) / 2).norm();
            if (fMaxOff - fOff23 < 5)
            {
                fScl = fLen / fLenSum;
                fRet = fMaxOff * (1 - fScl) + fOff23 * fScl;
            }
            else
            {
                fScl = (fOff23 / fMaxOff);
                float fDis = fLenSum / sqrt(1 - fScl * fScl);
                fRet = GetElevateValue(fLen, fDis, fMaxOff);
            }
        }
    }
    else
    {
        assert(false);
    }
    return fRet;
}

float XSubToy::GetElevateValue(float fLen, float fLenSum, float fMaxOff)
{
    float fRet = 0;
    float fScl = fLen / fLenSum;
    if (fScl > 1)
    {
        fScl = 1;
    }
    if (fScl < 0)
    {
        fScl = 0;
    }
    // ellipse function
    fRet = fMaxOff * sqrt(1 - fScl * fScl);
    return fRet;
}

float XSubToy::GetElevateValueInSleeveOrTerminalT(Pnt3 vt, TMesh &mesh, Fd iFaceID, vector<C3DLine> &lines)
{
    float fRet = 0;
    int i = 0, j = 0, iClineID = -1, iEdgeIdx = -1;
    for (i = 0; i < static_cast<int>(lines.size()); i++)
    {
        int iEdgeCount = lines[i].GetedgesCount();
        for (j = 0; j < iEdgeCount; j++)
        {
            Ed &edge = lines[i].Getedge(j);
            Hd h = mesh.halfedge(edge);

            Fd iFace1 = mesh.face(h);
            Fd iFace2 = mesh.face(opposite(h, mesh));
            if (iFace1 == iFaceID || iFace2 == iFaceID)
            {
                iClineID = i;
                iEdgeIdx = j;
                break;
            }
        }
        if (iClineID != -1)
        {
            break;
        }
    }
    if (iClineID == -1)
    {
        return fRet;
    }

    C3DLine &line = lines[iClineID];
    Pnt3 vt1, vt2, pt1, pt2, pt3;
    vt1 = line.Getipt(iEdgeIdx);
    if (iEdgeIdx < line.GetedgesCount() - 1)
    {
        vt2 = line.Getipt(iEdgeIdx + 1);
    }
    else
    {
        vt2 = line.Getcpt(1);
    }

    Pnt3 vtRef;
    Vd iPtRef;
    Hd f_hd = mesh.halfedge(iFaceID);
    Ed &edge = line.Getedge(iEdgeIdx);
    Vd ev0 = mesh.vertex(edge, 0);
    Vd ev1 = mesh.vertex(edge, 1);
    if (mesh.GetFType(iFaceID) == SLEEVE)
    {
        if (iEdgeIdx == line.GetedgesCount() - 1)
        {
            iPtRef = ev0;
            assert(false);
        }
        else
        {
            Ed &nextEdge = line.Getedge(iEdgeIdx + 1);
            Vd nxt_ev0 = mesh.vertex(nextEdge, 0);
            Vd nxt_ev1 = mesh.vertex(nextEdge, 1);
            if (ev0 == nxt_ev0 || ev1 == nxt_ev0)
            {
                iPtRef = nxt_ev0;
            }
            else if (ev0 == nxt_ev1 || ev1 == nxt_ev1)
            {
                iPtRef = nxt_ev1;
            }
        }
    }
    else if (mesh.GetFType(iFaceID) == TERMINAL)
    {
        iPtRef = ev0;
    }
    if (iPtRef != mesh.null_vertex())
    {
        Pnt3 v1, v2, v3, vtInter12, vtInter13, pt1, pt2;
        Vec3 vtDir;
        Vd idx = mesh.null_vertex();
        for (Vd vd : mesh.vertices_around_face(f_hd))
        {
            if (vd != ev0 && vd != ev1)
            {
                idx = vd;
            }
        }
        v1 = mesh.point(iPtRef);
        v2 = (ev0 == iPtRef) ? mesh.point(ev1) : mesh.point(ev0);
        v3 = mesh.point(idx);

        bool bFlag1, bFlag2;
        float fMaxOff31, fMaxOff12, fOff12, fOff31, fScl;
        float fLen12, fLen31, fLenSum12, fLenSum31;
        vtDir = (v2 - v3).normalized();
        pt1 = vt + vtDir * DIS_INTER;
        pt2 = vt - vtDir * DIS_INTER;

        bFlag1 = IsTwoLineIntersectIn2D(pt1, pt2, v1, v2);
        bFlag2 = IsTwoLineIntersectIn2D(pt1, pt2, v1, v3);
        if (bFlag1 && bFlag2)
        {
            vtInter12 = GetTwoLineInterPt(pt1, pt2, v1, v2);
            vtInter13 = GetTwoLineInterPt(pt1, pt2, v1, v3);
            fMaxOff12 = line.m_vOff[iEdgeIdx + 1];
            fMaxOff31 = line.m_vOff[iEdgeIdx + 2];
            fLen12 = (vtInter12 - (v1 + v2) / 2).norm();
            fLen31 = (vtInter13 - (v1 + v3) / 2).norm();

            fLenSum12 = (v1 - v2).norm() / 2;
            fLenSum31 = (v3 - v1).norm() / 2;

            fOff12 = GetElevateValue(fLen12, fLenSum12, fMaxOff12);
            fOff31 = GetElevateValue(fLen31, fLenSum31, fMaxOff31);
            fScl = (vt - vtInter12).norm() / (vtInter13 - vtInter12).norm();
            fRet = fOff12 * (1 - fScl) + fOff31 * fScl;
        }
        else
        {
            assert(false);
        }
    }
    return fRet;
}

void XSubToy::InflateLaplace(Float height_ratio)
{
    m_layerThick = height_ratio;
    pMesh3DTemp[0].clear();
    pMesh3DTemp[1].clear();
    m_curMesh3D.clear();
    ElevateMeshLaplace(m_curMesh2D, pMesh3DTemp[0], height_ratio, true);
    // WritePly(pMesh3DTemp[0], "output/pMesh3DTemp_0.ply");
    ElevateMeshLaplace(m_curMesh2D, pMesh3DTemp[1], -height_ratio, false);
    // WritePly(pMesh3DTemp[1], "output/pMesh3DTemp_1.ply");

    MergeTMeshes(m_curMesh3D, pMesh3DTemp);
    // WritePly(m_curMesh3D, "output/merged.ply");
    m_curMesh3D.ComputeVNormals();
}

bool XSubToy::ElevateMeshLaplace(const TMesh &meshRef, TMesh &meshOut, float height_ratio, bool bIsUpart)
{
    using namespace Eigen;

    if (meshRef.num_vertices() == 0)
        return false;

    meshOut = meshRef;
    if (!bIsUpart)
    {
        meshOut.ReverseFaceOrientation();
    }

    Eigen::MatrixXd &V = meshOut.GetVMat();
    Eigen::MatrixXi &F = meshOut.GetFMat();
    Eigen::SparseMatrix<double> &L = meshOut.GetLaplaceMat();

    //find boundary edges E, vertices b
    MatrixXi E;
    VectorXi b, IA, IC;
    igl::boundary_facets(F, E);
    igl::unique(E, b, IA, IC);
    // cout << "boundary vertices: " << b.rows() << endl;

    // List of all vertex indices
    VectorXi all, in;
    igl::colon<int>(0, V.rows() - 1, all);
    // List of interior indices
    igl::setdiff(all, b, in, IA);

    // Construct and slice up Laplacian
    SparseMatrix<double> L_in_in, L_in_b;
    igl::slice(L, in, in, L_in_in);
    igl::slice(L, in, b, L_in_b);
    // cout << "in: " << in.rows() << " b:" << b.rows() << endl;

    // Dirichlet boundary conditions from height-field
    SparseMatrix<double> &M = meshOut.GetMassMat();
    VectorXd bc = VectorXd::Zero(b.rows());
    VectorXd h, h_in; //height field
    h = M.diagonal() * height_ratio;
    for (int i = 0; i < b.rows(); i++)
    {
        h(b(i)) = 0.0;
    }
    igl::slice(h, in, 1, h_in);

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt(-L_in_in);
    VectorXd rhs = L_in_b * bc + h_in;
    VectorXd h_sol = llt.solve(rhs);
    if (llt.info() == Eigen::Success)
    {
        // M_DEBUG << " Laplace Mesh Inflate Solve Success" << endl;
        for (int i = 0; i < in.rows(); i++)
        {
            if (bIsUpart)
                V(in(i), 2) = sqrt(h_sol(i));
            else
                V(in(i), 2) = -sqrt(abs(h_sol(i)));
        }
        meshOut.SetPoints(V);
    }
    else
    {
        return false;
    }
    return true;
}

bool XSubToy::ElevateMeshLaplace(const TMesh &meshRef, TMesh &meshOut, Eigen::VectorXi &b, float height_ratio, bool bIsUpart)
{
    using namespace Eigen;

    if (meshRef.num_vertices() == 0)
        return false;

    meshOut = meshRef;
    if (!bIsUpart)
    {
        meshOut.ReverseFaceOrientation();
        // cout << "-------------------------------------back face" << endl;
    }

    Eigen::MatrixXd &V = meshOut.GetVMat();
    Eigen::MatrixXi &F = meshOut.GetFMat();
    Eigen::SparseMatrix<double> &L = meshOut.GetLaplaceMat();

    // List of all vertex indices
    VectorXi all, in, IA;
    igl::colon<int>(0, V.rows() - 1, all);
    // List of interior indices
    igl::setdiff(all, b, in, IA);

    // Construct and slice up Laplacian
    SparseMatrix<double> L_in_in, L_in_b;
    igl::slice(L, in, in, L_in_in);
    igl::slice(L, in, b, L_in_b);
    cout << "in: " << in.rows() << " b:" << b.rows() << endl;

    // Dirichlet boundary conditions from height-field
    SparseMatrix<double> &M = meshOut.GetMassMat();
    VectorXd bc = VectorXd::Zero(b.rows());
    VectorXd h, h_in; //height field
    h = M.diagonal() * height_ratio;
    for (int i = 0; i < b.rows(); i++)
    {
        h(b(i)) = 0.0;
    }
    igl::slice(h, in, 1, h_in);

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt(-L_in_in);
    VectorXd rhs = L_in_b * bc + h_in;
    VectorXd h_sol = llt.solve(rhs);
    if (llt.info() == Eigen::Success)
    {
        cout << "solve success" << endl;
        for (int i = 0; i < in.rows(); i++)
        {
            if (bIsUpart)
                V(in(i), 2) = sqrt(h_sol(i));
            else
                V(in(i), 2) = -sqrt(abs(h_sol(i)));
        }
        meshOut.SetPoints(V);
    }
    else
    {
        return false;
    }
    return true;
}

void XSubToy::GetRotateAngleAxis(const Vec3 &viewDir, Vec3 &axis, float &angle)
{
    Vec3 vect1(0, 0, 1);
    angle = GetAngleOf2Vector(vect1, viewDir);
    axis = vect1.cross(viewDir);
}

void XSubToy::GetMeshPts2D(const TMesh &mesh, vector<Pnt3> &mesh_pts2D)
{
    int vNum = mesh.num_vertices();
    mesh_pts2D.resize(vNum);
    for (Vd vd : mesh.vertices())
    {
        Pnt3 p = mesh.point(vd);
        assert(p.z() < 1.0e-6);
        mesh_pts2D[vd.idx()] = Pnt3(p.x(), p.y(), p.z());
    }
}

void XSubToy::GetContourPts2D(const vector<Pnt3> &sketchPolyLine, vector<Pnt3> &contour2D)
{
    contour2D.resize(sketchPolyLine.size());
    for (int i = 0; i < sketchPolyLine.size(); i++)
    {
        const Pnt3 &p = sketchPolyLine[i];
        assert(p.z() < 1.0e-6);
        contour2D[i] = Pnt3(p.x(), p.y(), p.z());
    }
}

void XSubToy::InitializeSimplifiedSketchPolyLines(const vector<vector<Pnt3>> &sketchPolyLines, vector<vector<Pnt3>> &simpPolyLines)
{
    simpPolyLines.clear();
    for (int i = 0; i < sketchPolyLines.size(); i++)
    {
        const vector<Pnt3> &polyline = sketchPolyLines.at(i);
        vector<Pnt3> simplified_polyline;

        for (int j = 0; j < polyline.size() - 1; j += 5)
        {
            simplified_polyline.push_back(polyline.at(j));
        }
        simpPolyLines.push_back(simplified_polyline);
    }
}

void XSubToy::ReComputeSimplifiedSketchPolyLines(CSpline &spline, vector<vector<Pnt3>> &simpPolyLines, int index)
{
    if (simpPolyLines.empty())
        return;

    assert(index >= 0 && index < simpPolyLines.size() && "Spline control point is wrong!");

    int SplineCtrlPtsSize = spline.GetCtrlPointCount();
    vector<Pnt3> &index_polyline = simpPolyLines.at(index);
    int index_polyline_size = index_polyline.size();
    bool IsIndexPolyDiscretized = CSketchManager::DiscretizeAppointedPolyLine(spline, index_polyline, index, index_polyline_size);
    int pre_index = (index - 1 + SplineCtrlPtsSize) % SplineCtrlPtsSize;
    if (pre_index >= simpPolyLines.size())
        return;
    vector<Pnt3> &preindex_polyline = simpPolyLines.at(pre_index);
    int preindex_polyline_size = preindex_polyline.size();

    bool IsPreIndexPolyDiscretized = CSketchManager::DiscretizeAppointedPolyLine(spline, preindex_polyline, pre_index, preindex_polyline_size);
}

void XSubToy::InitContourDeform(const Vec3 &viewDir)
{
    // ToDO: change this to contour captured by stencil test
    if (m_curMesh3D.number_of_vertices() == 0)
        return;
    m_defContour.clear();

    M_DEBUG << "InitContourDeform viewDir=" << viewDir.transpose() << endl;
    InitializeSimplifiedSketchPolyLines(m_defPolyLines, m_SimplifiedDefPolyLines);
    for (int i = 0; i < m_SimplifiedDefPolyLines.size(); i++)
    {
        vector<Pnt3> &polyline = m_SimplifiedDefPolyLines[i];
        for (int j = 0; j < polyline.size(); j++)
        {
            m_defContour.push_back(polyline[j]);
        }
    }

    ComputeAffineTransformationCoefficient3D(m_curMesh3D, m_defContour, viewDir, m_affineCoeff);
    M_DEBUG << " InitContourDeform successfully! viewDir=" << viewDir.transpose()
            << " m_defContour=" << m_defContour.size() << " m_affineCoeff=" << m_affineCoeff.size() << endl;
}

void XSubToy::DeformContour(const Vec3 &viewDir)
{
    if (m_curMesh3D.number_of_vertices() == 0)
        return;

    m_defContour.clear();

    ReComputeSimplifiedSketchPolyLines(m_defSpline, m_SimplifiedDefPolyLines, m_vDefCptIdx);
    for (int i = 0; i < m_SimplifiedDefPolyLines.size(); i++)
    {
        vector<Pnt3> &polyline = m_SimplifiedDefPolyLines[i];
        for (int j = 0; j < polyline.size(); j++)
        {
            m_defContour.push_back(polyline[j]);
        }
    }

    // M_DEBUG << " DeformContour viewDir=" << viewDir.transpose()
    //      << " m_defContour=" << m_defContour.size() << " m_affineCoeff=" << m_affineCoeff.size() << endl;
    RefereshModifiedMeshVerticesByAffineTransformation3D(m_defContour, m_affineCoeff, viewDir, m_curMesh3D);
    // WriteOBJ(m_curMesh3D, "output/contourDefMesh3d.obj");
}

void XSubToy::ComputeAffineTransformationCoefficient3D(const TMesh &mesh, const vector<Pnt3> &contour,
                                                       const Vec3 &viewDir, vector<vector<double>> &coeff)
{

    Matrix3d rot;
    GetRotMatToXYPlane(viewDir, rot);
    vector<Pnt3> vTmpPolyLinePts, vTmpPts;
    for (int i = 0; i < contour.size(); i++)
    {
        Pnt3 p = rot * contour[i];
        vTmpPolyLinePts.emplace_back(p.x(), p.y(), 0);
    }
    for (const Vd &vd : mesh.vertices())
    {
        Pnt3 p = rot * mesh.point(vd);
        vTmpPts.emplace_back(p.x(), p.y(), 0);
    }
    ComputeAffineTransformationCoefficient(vTmpPolyLinePts, vTmpPts, coeff);
}

void XSubToy::ComputeAffineTransformationCoefficient2D(const TMesh &mesh, const vector<Pnt3> &sketchPolyLine,
                                                       vector<vector<double>> &coeff)
{
    vector<Pnt3> vTmpPolyLinePts, vTmpPts;
    GetContourPts2D(sketchPolyLine, vTmpPolyLinePts);
    GetMeshPts2D(mesh, vTmpPts);
    ComputeAffineTransformationCoefficient(vTmpPolyLinePts, vTmpPts, coeff);
}

// Coeff: vNum by bNum matrix
void XSubToy::ComputeAffineTransformationCoefficient(const vector<Pnt3> &vTmpPolyLinePts, const vector<Pnt3> &vTmpPts, vector<vector<double>> &coeff)
{
    int vNum = vTmpPts.size();
    int bNum = vTmpPolyLinePts.size(); // contour points
    coeff.clear();
    for (int i = 0; i < vNum; i++)
    {
        const Pnt3 &pt3d = vTmpPts[i];
        vector<double> OnePtCoeff(bNum, 0);

        bool IsOnSplineCtrlPt = false;
        int OnSplineCtrlPtIndex = -1;
        bool IsOnSplineBoundary = false;
        int OnSplineBoundaryIndex = -1;

        // on boundary vertex
        for (int j = 0; j < bNum; j++)
        {
            float distance = (pt3d - vTmpPolyLinePts[j]).norm();
            if (distance < 1e-3)
            {
                IsOnSplineCtrlPt = true;
                OnSplineCtrlPtIndex = j;
                break;
            }
        }
        // on boundary edge
        if (!IsOnSplineCtrlPt && bNum >= 1)
        {
            for (int j = 0; j < bNum; j++)
            {
                float dot_vert = ((pt3d - vTmpPolyLinePts[j]).normalized()).dot((pt3d - vTmpPolyLinePts[(j + 1) % bNum]).normalized());
                if (fabs(dot_vert + 1) < 1e-3)
                {
                    IsOnSplineBoundary = true;
                    OnSplineBoundaryIndex = j;
                    break;
                }
            }
        }

        if (IsOnSplineCtrlPt && !IsOnSplineBoundary)
        {
            OnePtCoeff[OnSplineCtrlPtIndex] = 1.0;
        }
        else if (!IsOnSplineCtrlPt && IsOnSplineBoundary)
        {
            const Pnt3 &st = vTmpPolyLinePts[OnSplineBoundaryIndex];
            const Pnt3 &end = vTmpPolyLinePts[(OnSplineBoundaryIndex + 1) % bNum];
            float ratio = (pt3d - st).norm() / (end - st).norm();
            OnePtCoeff[OnSplineBoundaryIndex] = 1 - ratio;
            OnePtCoeff[(OnSplineBoundaryIndex + 1) % bNum] = ratio;
        }
        else // inside the mesh
        {
            vector<float> A, r, D;
            vector<double> tan_half_ang, Weight;
            vector<Vec3> S;
            // area, radius, direction
            for (int j = 0; j < bNum; j++)
            {
                A.push_back(ComputeArea(pt3d, vTmpPolyLinePts[j], vTmpPolyLinePts[(j + 1) % bNum]));
                r.push_back((pt3d - vTmpPolyLinePts[j]).norm());
                S.push_back(vTmpPolyLinePts[j] - pt3d);
            }
            // dot
            for (int j = 0; j < bNum; j++)
            {
                D.push_back(S[j].dot(S[(j + 1) % bNum]));
            }
            // tan(aj/2)
            for (int j = 0; j < bNum; j++)
            {
                tan_half_ang.push_back((r[j] * r[(j + 1) % bNum] - D[j]) / A[j] * 0.5);
            }
            Weight.push_back((tan_half_ang[bNum - 1] + tan_half_ang[0]) / r[0] * 0.5);
            for (int j = 1; j < bNum; j++)
            {
                Weight.push_back((tan_half_ang[j - 1] + tan_half_ang[j]) / r[j] * 0.5);
            }
            float WeightSum = 0;
            for (int j = 0; j < bNum; j++)
            {
                WeightSum += Weight[j];
            }
            for (int j = 0; j < bNum; j++)
            {
                OnePtCoeff[j] = Weight[j] / WeightSum;
            }
        }
        coeff.push_back(OnePtCoeff);
    }
}

void XSubToy::RefereshModifiedMeshVerticesByAffineTransformation3D(const vector<Pnt3> &contour,
                                                                   const vector<vector<double>> &coeff,
                                                                   const Vec3 &viewDir,
                                                                   TMesh &mesh)
{
    // Rotate to X-Y plane
    Matrix3d rot, rot_inv;
    GetRotMatToXYPlane(viewDir, rot);
    rot_inv = rot.inverse();
    vector<Pnt3> vTmpPolyLinePts, vTmpPts;
    for (int i = 0; i < contour.size(); i++)
    {
        Pnt3 p = rot * contour[i];
        vTmpPolyLinePts.emplace_back(p.x(), p.y(), p.z());
    }
    for (const Vd &vd : mesh.vertices())
    {
        Pnt3 p = rot * mesh.point(vd);
        vTmpPts.emplace_back(p.x(), p.y(), p.z());
    }
    RefereshModifiedMeshVerticesByAffineTransformation(vTmpPolyLinePts, coeff, vTmpPts);
    for (int i = 0; i < vTmpPts.size(); i++)
    {
        vTmpPts[i] = rot_inv * vTmpPts[i];
    }
    mesh.UpdatePos(vTmpPts);
}

void XSubToy::RefereshModifiedMeshVerticesByAffineTransformation2D(const vector<Pnt3> &sketchPolyLine,
                                                                   const vector<vector<double>> &coeff,
                                                                   TMesh &mesh)
{
    vector<Pnt3> vTmpPolyLinePts, vTmpPts;
    GetContourPts2D(sketchPolyLine, vTmpPolyLinePts);
    GetMeshPts2D(mesh, vTmpPts);
    RefereshModifiedMeshVerticesByAffineTransformation(vTmpPolyLinePts, coeff, vTmpPts);
    // ToDO: update mesh
    mesh.UpdatePos(vTmpPts);
}

void XSubToy::RefereshModifiedMeshVerticesByAffineTransformation(const vector<Pnt3> &vTmpPolyLinePts,
                                                                 const vector<vector<double>> &coeff,
                                                                 vector<Pnt3> &vTmpPts)
{
    assert(coeff.size() == vTmpPts.size());
    assert(coeff[0].size() == vTmpPolyLinePts.size());

    int vNum = vTmpPts.size();
    int bNum = vTmpPolyLinePts.size();

    for (int i = 0; i < vNum; i++)
    {
        float x = 0, y = 0;
        for (int j = 0; j < bNum; j++)
        {
            x += coeff[i][j] * vTmpPolyLinePts[j].x();
            y += coeff[i][j] * vTmpPolyLinePts[j].y();
        }
        vTmpPts[i] = Pnt3(x, y, vTmpPts[i].z());
    }
}

void XSubToy::SetInitViewDir(const Vec3 &iniViewDir)
{
    m_iniViewDir = iniViewDir;
    m_iniViewDir.normalize();
}

int XSubToy::InitViewMode() const
{
    const Vec3 frontViewDir(0, 0, -1);
    const Vec3 backViewDir(0, 0, 1);
    const Vec3 rightViewDir(-1, 0, 0);
    const Vec3 leftViewDir(1, 0, 0);
    const Vec3 topViewDir(0, -1, 0);
    const Vec3 bottomViewDir(0, 1, 0);

    if ((m_iniViewDir - frontViewDir).norm() < 1e-8)
    {
        return VIEW_FRONT;
    }
    else if ((m_iniViewDir - backViewDir).norm() < 1e-8)
    {
        return VIEW_BACK;
    }
    else if ((m_iniViewDir - rightViewDir).norm() < 1e-8)
    {
        return VIEW_RIGHT;
    }
    else if ((m_iniViewDir - leftViewDir).norm() < 1e-8)
    {
        return VIEW_LEFT;
    }
    else if ((m_iniViewDir - topViewDir).norm() < 1e-8)
    {
        return VIEW_TOP;
    }
    else if ((m_iniViewDir - bottomViewDir).norm() < 1e-8)
    {
        return VIEW_BOTTOM;
    }
    else
    {
        return VIEW_PERSP;
    }
}

void XSubToy::TriangulateCoarseFrom3DContour()
{

    vector<Pnt3> contPts;
    Get2DDiscreteLine(m_sketchPolyLineOnXY, contPts, 3, 1);

    if (m_curMesh2DTemp.num_vertices() > 0)
        m_curMesh2DTemp.clear();

    bool flag = Triangulation(contPts, m_curMesh2DTemp, Constrained);
    m_curMesh2DTemp.Rotate(m_rotToXY.transpose());
    m_curMesh2DTemp.Translate(m_centroid);
    WriteOBJ(m_curMesh2DTemp, "output/m_curMesh2DTemp" + to_string(m_toyID) + ".obj");
}

void XSubToy::TriangulateFineFrom3DContour()
{
    // Rotate 3DContour to XY Plane
    vector<Pnt3> contPts;
    Get2DDiscreteLine(m_sketchPolyLineOnXY, contPts, 10, 1);

    if (m_curMesh2D.num_vertices() > 0)
        m_curMesh2D.clear();

    float max_triArea = 0.333 * (m_curMesh2DTemp.Get3DArea() / m_curMesh2DTemp.num_faces());
    bool flag = Triangulation(contPts, m_curMesh2D, Constrained_Delaunay, max_triArea);

    // Rotate Mesh Verts back
    m_curMesh2D.Rotate(m_rotToXY.transpose());
    m_curMesh2D.Translate(m_centroid);
    WriteOBJ(m_curMesh2D, "output/m_curMesh2D" + to_string(m_toyID) + ".obj");
}

void XSubToy::InflateLaplaceFrom3DPlane(Float height_ratio)
{
    // Transform m_curMesh2D to XY Plane
    TMesh m_curMesh2DXYPlane = m_curMesh2D;
    m_curMesh2DXYPlane.Translate(-m_centroid);
    // WriteOBJ(m_curMesh2D, "output/m_curMesh2D" + to_string(m_toyID) + ".obj");
    // WriteOBJ(m_curMesh2DXYPlane, "output/m_curMesh2DXYPlane_trans" + to_string(m_toyID) + ".obj");
    m_curMesh2DXYPlane.Rotate(m_rotToXY);
    // WriteOBJ(m_curMesh2DXYPlane, "output/m_curMesh2DXYPlane_rotate" + to_string(m_toyID) + ".obj");

    // Inflate
    m_layerThick = height_ratio;
    pMesh3DTemp[0].clear();
    pMesh3DTemp[1].clear();
    m_curMesh3D.clear();
    ElevateMeshLaplace(m_curMesh2DXYPlane, pMesh3DTemp[0], height_ratio, true);
    // WriteOBJ(pMesh3DTemp[0], "output/pMesh3DTemp_0.obj");
    ElevateMeshLaplace(m_curMesh2DXYPlane, pMesh3DTemp[1], -height_ratio, false);
    // WriteOBJ(pMesh3DTemp[1], "output/pMesh3DTemp_1.obj");

    MergeTMeshes(m_curMesh3D, pMesh3DTemp);

    m_curMesh3D.Rotate(m_rotToXY.transpose()); // from 2D XY plane to 6 canonical 3D space
    m_curMesh3D.Translate(m_centroid);
    m_curMesh3D.ComputeVNormals();
    WriteOBJ(m_curMesh3D, "output/m_curMesh3D" + to_string(m_toyID) + ".obj");
}

#include "slice_polygon.h"
#include "sskel_draw.h"

typedef typename SSkeletor::SsPtr SsPtr;
typedef SSNode NodeT;
typedef SSEdge<NodeT> EdgeT;
typedef SSBranch<NodeT> SSBranchT;

void XSubToy::Skeletonize(int skel_opt)
{
    if (skel_opt == SKEL_SYM)
    {
        skeletonize_from_sym_line();
    }
    else
    {
        Skeletonize();
    }
}

void XSubToy::Skeletonize()
{
#ifdef SSKEL_DEBUG
    M_DEBUG << "offseting polygon" << endl;
#endif

    std::vector<Pnt3> simp_polyline;
    SimplifyPolyLine(m_sketchPolyLineOnXY, simp_polyline, 3); // ToDO: tol=3: smaller tol give more ssnodes
    if ((simp_polyline.back() - simp_polyline.front()).norm() < 0.1)
    {
        simp_polyline.pop_back();
    }
    if (IsClockWise(simp_polyline))
    {
        std::reverse(simp_polyline.begin(), simp_polyline.end());
    }

#ifdef SSKEL_DEBUG
    egl::writeLine("output/poly_" + name() + ".txt", simp_polyline);
    skelDrawerPtr = std::make_shared<SkelDrawer>();
    skelDrawerPtr->set_sil(simp_polyline);
    cv::Mat sil_img = skelDrawerPtr->draw_sil();
    cv::imwrite("output/sil.png", sil_img);
    cv::Mat sketch_img = skelDrawerPtr->draw_sketch_line(m_sketchPolyLineOnXY);
    cv::imwrite("output/processed_sketch.png", sketch_img);
#endif
    // straight skeleton extraction & collapsing
    SsPtr ss_ptr = SSkeletor::skeletonize(simp_polyline, m_rawSSkel, m_sskel);

#ifdef SSKEL_DEBUG
    cv::Mat ini_sskel_img = skelDrawerPtr->draw_straight_skeleton(ss_ptr, false);
    cv::imwrite("output/straight_skeleton.png", ini_sskel_img);
    skelDrawerPtr->draw_defining_contour(ss_ptr, &ini_sskel_img);
    cv::Mat pruned_ini_sskel_img = skelDrawerPtr->draw_straight_skeleton(ss_ptr, true);
    cv::imwrite("output/pruned.png", pruned_ini_sskel_img);
    cv::Mat collapse_sskel_img = skelDrawerPtr->draw_sskel(m_rawSSkel, &sil_img);
    cv::imwrite("output/collapse.png", collapse_sskel_img);
#else
    // inverse operation to SetSketchPolyLineOnXY() And MapToXY()
    m_rawSSkel.rotate(m_rotToXY.transpose());
    m_rawSSkel.translate(m_centroid);
#endif

    if (!m_autoSkel)
        return;

    // DP simplification
    m_rawSSkel.collect_nodes(junc_nodes, term_nodes);
    collect_branches(junc_nodes, term_nodes, branches);

    PolyPartition poly_partition(simp_polyline, branches);
#ifdef SSKEL_DEBUG
    poly_partition.partition();
    skelDrawerPtr->set_bcolors(branches);
    skelDrawerPtr->set_branches(poly_partition.branch_smthpolys(),
                                poly_partition.branch_unisamples(),
                                poly_partition.branch_slices());
    cv::Mat spline_img = skelDrawerPtr->draw_splines(poly_partition.branch_densamples(), sil_img);
    cv::imwrite("output/spline.png", spline_img);
    cv::Mat unisample_img = skelDrawerPtr->draw_uniform_samples(poly_partition.branch_unisamples(), sil_img);
    cv::imwrite("output/unisample.png", unisample_img);
    cv::Mat decompose_smth_img = skelDrawerPtr->draw_branch_smthpolys(poly_partition.branch_smthpolys(),
                                                                      &sil_img);
    cv::imwrite("output/decompose_smth.png", decompose_smth_img);
    cv::Mat slice_img = skelDrawerPtr->draw_branch_slices(poly_partition.branch_slices(), decompose_smth_img);
    cv::imwrite("output/slice.png", slice_img);
    auto &bvslice_imgs = skelDrawerPtr->draw_branch_vslices(poly_partition.branch_vslices());
    for (auto it = bvslice_imgs.begin(); it != bvslice_imgs.end(); ++it)
    {
        SSBranchT *b = it->first;
        cv::imwrite("output/vslice_" + b->id_str() + ".png", it->second);
        cv::imwrite("output/bimg_" + b->id_str() + ".png", skelDrawerPtr->bimgs[b].slice_img);
    }
    cv::Mat decompose_img = skelDrawerPtr->draw_branch_polygons(poly_partition.branch_polygons(),
                                                                poly_partition.branch_silidxs());
    cv::imwrite("output/decompose.png", decompose_img);
#endif
    simplify_branches(branches, poly_partition);

    // create skeleton
    create_skeleton();

#ifdef SSKEL_DEBUG
    M_DEBUG << "skeleton after dp simplification" << endl;
    m_sskel.print();

    cv::Mat dpsimp_sskel_img = skelDrawerPtr->draw_sskel(m_sskel, &sil_img);
    cv::imwrite("output/dpsimp_sskel0.png", dpsimp_sskel_img);
#endif

    // // from XY Plane to 3D Space
    // if (m_symToy == nullptr)
    // {
    //     RotateSSkel(m_rotToXY.transpose());
    //     TranslateSSkel(m_centroid);
    // }

#ifdef SSKEL_DEBUG
    M_DEBUG << name() << " sub skeletonize successfully!\n";
    for (int dp_thresh = 1; dp_thresh <= 20; ++dp_thresh)
    {
        interactive_simplify_skeleton(dp_thresh);
        cv::Mat simp_sskel = skelDrawerPtr->draw_sskel(m_sskel, &sil_img);
        cv::imwrite("output/dpsimp_" + std::to_string(int(dp_thresh)) + ".png", simp_sskel);
    }
#endif
}

void XSubToy::skeletonize_from_sym_line()
{
    M_DEBUG << "skeletonize_from_sym_line" << endl;
    Line l = m_symLine.trimmed_line(0.05);
    create_skeleton_from_line(l, m_rawSSkel);
    double radius = m_rawSSkel.nodes[0]->radius;
    SSBranch<NodeT> b(m_rawSSkel.nodes.front(), m_rawSSkel.nodes.back());
    b.pnts.push_back(l.p0);
    b.pnts.push_back(l.p1);
    b.radius.push_back(radius);
    b.radius.push_back(radius);
    b.simp_pnts.push_back(l.p0);
    b.simp_pnts.push_back(l.p1);
    b.simp_radius.push_back(radius);
    b.simp_radius.push_back(radius);
    branches.push_back(b);
    if (!m_autoSkel)
        return;
    create_skeleton_from_line(l, m_sskel);
}

void XSubToy::create_skeleton_from_line(const Line &l, SSkel<NodeT> &sskel)
{
    BBox3 bbox = m_curMesh3D.bbox();
    double len = bbox.max_axis_len();
    NodeT *n0 = sskel.add_node(l.p0);
    NodeT *n1 = sskel.add_node(l.p1);
    n0->set_radius(len * 0.5);
    n1->set_radius(len * 0.5);
    sskel.add_edge(n0, n1);
    sskel.set_root(n1);
}

void XSubToy::interactive_simplify_skeleton(double dp_thresh)
{
    if (!m_autoSkel)
        return;

    // cal bounded contour
    std::vector<Pnt3> contour;
    double polygon_len = CalPolygonLength(m_sketchPolyLineOnXY);
    DiscreteLineByLen(m_sketchPolyLineOnXY, polygon_len, 10, contour);
    if (IsClockWise(contour))
    {
        std::reverse(contour.begin(), contour.end());
    }

    for (typename std::list<SSBranchT>::iterator bit = branches.begin();
         bit != branches.end(); ++bit)
    {
        SSBranchT &b = *bit;
        // const std::vector<Pnt3> &contour = poly_partition.polygon(); // ToDO
        simplify_one_branch(b, dp_thresh, contour); // use the dp_thresh provided by user
    }

    create_skeleton();
}

void XSubToy::simplify_one_branch(SSBranchT &b, double init_thresh, const std::vector<Pnt3> &contour)
{
#ifdef BRANCH_DEBUG
    CvDrawer drawer;
    drawer.set_sil(contour);
    cv::Mat img = drawer.draw_sil();
#endif
    std::vector<bool> keepMark;
    std::vector<Pnt3> simp_branch;
    std::vector<double> simp_radius;
    int iter = 0;
    bool isBranchInsideShape = true; // check whether the simplified line inside the shape
    b.dp_thresh = init_thresh;

    do
    {
        simp_branch.clear();
        simp_radius.clear();
        SimplifyPolyLineDP(b.pnts, keepMark, b.dp_thresh); //ToDO: change this to control bone complexity

        for (size_t i = 0; i < b.pnts.size(); ++i)
        {
            if (keepMark[i])
            {
                simp_branch.push_back(b.pnts[i]);
                simp_radius.push_back(b.radius[i]);
            }
        }

        isBranchInsideShape = true;
        std::vector<Pnt3> simpXY;
        MapToXY(simp_branch, simpXY);
        if (!IsLineInsidePolygon(simpXY, contour))
        {
            M_DEBUG
                << "branch" << b.id << " "
                << "DP threshold is so large that make the simplified bone outside contour! dp_thresh="
                << b.dp_thresh
                << endl;
            isBranchInsideShape = false;
            b.dp_thresh *= 0.8;
        }

#ifdef BRANCH_DEBUG
        cv::Mat img_simp = img.clone();
        drawer.draw_line(img_simp, simpXY, cv::Scalar(0, 0, 0));
        drawer.draw_points(img_simp, simpXY, cv::Scalar(0, 0, 255), 3);
        cv::imwrite("output/b" + std::to_string(b.id) + " interactive_simp_" + std::to_string(int(b.dp_thresh)) + ".png", img_simp);
#endif
        if (b.dp_thresh < 1)
            break;

        if (iter > 10)
            break;
        ++iter;
    } while (!isBranchInsideShape);

    b.simp_pnts.clear();
    b.simp_pnts.reserve(simp_branch.size());
    for (const Pnt3 &p : simp_branch)
    {
        b.simp_pnts.push_back(p);
    }

    b.simp_radius.clear();
    b.simp_radius.reserve(simp_radius.size());
    for (double r : simp_radius)
    {
        b.simp_radius.push_back(r);
    }

    M_DEBUG << "branch after DP: " << b << std::endl;
}

bool XSubToy::IsLineInsidePolygon(const std::vector<Pnt3> &line, const std::vector<Pnt3> &polygon)
{
    for (size_t i = 0; i < line.size() - 1; ++i)
    {
        for (size_t j = 0; j < polygon.size() - 1; ++j)
        {
            if (IsTwoLineIntersectIn2D(line[i], line[i + 1], polygon[j], polygon[j + 1]))
            {
                return false;
            }
        }
    }
    return true;
}

void XSubToy::create_skeleton()
{
    m_sskel.clear();
    if (junc_nodes.size() > 0)
    {
        std::unordered_map<NodeT *, std::vector<SSBranchT *>> node_branches;
        collect_node_branches(branches, node_branches);
        NodeT *root = node_with_max_num_branches(node_branches);
        // M_DEBUG << "node_branches" << endl;
        // for (auto it = node_branches.begin(); it != node_branches.end(); ++it)
        // {
        //     M_DEBUG << *it->first << ": branches=" << it->second.size() << endl;
        // }
        // M_DEBUG << "root=" << *root << endl;
        create_tree_skel(junc_nodes, node_branches, root, m_sskel);
    }
    else
    {
        create_ribon_skel(term_nodes, branches.front(), m_sskel);
    }
}

void XSubToy::collect_branches(const std::vector<SSNode *> &junc_nodes,
                               const std::vector<SSNode *> &term_nodes,
                               std::list<SSBranchT> &branches)
{
    // Input: collapsed straight skeleton
    // Output: all branches
    std::unordered_map<NodeT *, std::vector<SSBranchT *>> node_branches;
    if (junc_nodes.size() > 0)
    {
        for (NodeT *jnode : junc_nodes)
        {
            find_branches_jnode(jnode, branches, node_branches);
        }
    }
    else
    {
        find_branches_jnode(term_nodes[0], branches, node_branches);
    }
}

#ifdef SSKEL_DEBUG
void XSubToy::simplify_branches(std::list<SSBranchT> &branches,
                                PolyPartition &poly_partition)
{
    double dp_thresh = DP_THRESH;
    std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_vslices = poly_partition.branch_vslices();
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_unisamples = poly_partition.branch_unisamples();
    for (typename std::list<SSBranchT>::iterator bit = branches.begin();
         bit != branches.end(); ++bit)
    {
        SSBranchT &b = *bit;
        const std::vector<Pnt3> &contour = poly_partition.polygon(); // ToDO: use partitoned polygon to restrict simplification
        std::vector<Pnt3> simp_branch;
        bool isBranchInsideShape = true;
        int iter = 0;
        do
        {
            simp_branch.clear();
            if (b.type() == JJ_BRANCH)
            {
                std::vector<bool> mark;
                SimplifyPolyLineDP(b.pnts, mark, dp_thresh); //ToDO: change this to control bone complexity
                for (size_t i = 0; i < mark.size(); ++i)
                {
                    if (mark[i])
                        simp_branch.push_back(b.pnts[i]);
                }
            }
            else
            {
                simplify_bounded_dp(&b, branch_unisamples[&b], branch_vslices[&b], simp_branch, dp_thresh);
            }

            // check whether the simplified line inside the shape
            isBranchInsideShape = true;
            for (size_t i = 0; i < simp_branch.size() - 1; ++i)
            {
                const Pnt3 &st_bone = simp_branch[i];
                const Pnt3 &end_bone = simp_branch[i + 1];
                for (size_t j = 0; j < contour.size() - 1; ++j)
                {
                    if (IsTwoLineIntersectIn2D(st_bone, end_bone, contour[j], contour[j + 1]))
                    {
                        printf("DP threshold is so large that make the simplified bone outside contour!\n");
                        isBranchInsideShape = false;
                        dp_thresh *= 0.8;
                        break;
                    }
                }
                if (!isBranchInsideShape)
                    break;
            }

            M_DEBUG << "simplify_branches iter=" << iter
                    << ", dp_thresh=" << dp_thresh << " b.pnts=" << b.psize()
                    << " simp_branch=" << simp_branch.size() << endl;

            if (dp_thresh < 1)
                break;
            if (iter > 10)
                break;
            ++iter;
        } while (!isBranchInsideShape);

        if (isBranchInsideShape)
        {
            b.simp_pnts.clear();
            b.simp_pnts.reserve(simp_branch.size());
            for (const Pnt3 &p : simp_branch)
            {
                b.simp_pnts.push_back(p);
            }
            M_DEBUG << b << " simp_bones=" << b.psize() << std::endl;
        }
    }

    M_DEBUG << "simplify branches success!" << endl;
}

void XSubToy::simplify_bounded_dp(SSBranchT *bPtr,
                                  const vector<Pnt3> &oriPArr,
                                  const vector<Slice> &slices,
                                  vector<Pnt3> &resPArr, float tol)
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
    vector<bool> PArrMark;

    // Bounded Douglas-Peucker polyline simplification
    int curSize = static_cast<int>(oriPArr.size());
    //std::cout << "PArr=" << PArr.size() << std::endl;
    PArrMark.resize(curSize, false);
    PArrMark.front() = true;
    PArrMark.back() = true; // mark the first and last vertices

    M_DEBUG << "--------bounded_dp " << *bPtr << endl;
    bounded_dp(bPtr, 0, oriPArr, slices, 0, curSize - 1, PArrMark, tol, 0.01);
    M_DEBUG << "--------bounded_dp end " << *bPtr << endl
            << endl;

    // copy PArr to resPArr.
    for (i = 0; i < curSize; ++i)
    {
        if (PArrMark.at(i))
            resPArr.push_back(oriPArr.at(i));
    }
}

void XSubToy::bounded_dp(SSBranchT *bPtr,
                         int iter,
                         const vector<Pnt3> &PArr,
                         const vector<Slice> &slices,
                         int stIdx, int endIdx,
                         vector<bool> &PArrMark, Float tol, double weight)
{
    if (endIdx <= stIdx + 1) // there is nothing to simplify
        return;

    // check for adequate approximation by segment S from PArr[stIdx] to PArr[endIdx]
    int maxi = stIdx;                          // index of vertex farthest from S
    Float maxd2 = 0;                           // max error
    Float tol2 = tol * tol;                    // tolerance squared
    Vec3 u = PArr.at(endIdx) - PArr.at(stIdx); // segment direction vector
    Float cu = u.squaredNorm();                // segment length squared

    // test each vertex PArr[i] for max distance from S
    Vec3 w;
    Pnt3 Pb;          // base of perpendicular from PArr[i] to S
    Float b, cw, dv2; // dv2 = distance PArr[i] to S squared
    double ds2;       // (segment S[i] - trapezoid Segment).squared()
    double err_i;
    double l_st = slices[stIdx].diameter();
    double l_end = slices[endIdx].diameter();
    double l_slices = (slices[endIdx].cent() - slices[stIdx].cent()).norm();
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
        // compute segment variation squared
        double t = (slices[i].cent() - slices[stIdx].cent()).norm();
        t /= l_slices;
        double l_ti = l_st + t * (l_end - l_st); // trapezoid segment length
        double l_i = slices[i].diameter();
        double ds2 = (l_i - l_ti) * (l_i - l_ti);
        err_i = dv2 + weight * ds2;
        M_DEBUG << i << " l_i=" << l_i << " dv2=" << dv2 << " ds2=" << ds2 << " err_i=" << err_i << endl;
        // test with current max distance squared
        if (err_i >= maxd2)
        {
            // PArr[i] is a new max vertex
            maxi = i;
            maxd2 = err_i;
        }
    }

    if (maxd2 > tol2)
    {
        cv::Mat dp_img = skelDrawerPtr->draw_one_branch_dp(bPtr, iter, maxi, stIdx, endIdx);
        cv::Mat slice_img = skelDrawerPtr->draw_one_branch_vslices_dp(bPtr, iter, slices, maxi, stIdx, endIdx);
        char fname[100], fname1[100];
        sprintf(fname, "output/vslice_%d_iter%d_st%d_en%d.png", bPtr->id, iter, stIdx, endIdx);
        sprintf(fname1, "output/dp_%d_iter%d_st%d_en%d.png", bPtr->id, iter, stIdx, endIdx);
        cv::imwrite(fname, slice_img);
        cv::imwrite(fname1, dp_img);
    }

    ++iter;

    if (maxd2 > tol2) // error is worse than the tolerance
    {
        // split the polyline at the farthest vertex from S
        PArrMark[maxi] = 1; // mark pArr[maxi] for the simplified polyline
        // recursively simplify the two subpolylines at pArr[maxi]
        bounded_dp(bPtr, iter, PArr, slices, stIdx, maxi, PArrMark, tol, weight);  // polyline PArr[stIdx] to PArr[maxi]
        bounded_dp(bPtr, iter, PArr, slices, maxi, endIdx, PArrMark, tol, weight); // polyline PArr[maxi] to PArr[endIdx]
    }

    // else the approximation is OK, so ignore intermediate vertices
    return;
}

#else

void XSubToy::simplify_branches(std::list<SSBranchT> &branches,
                                PolyPartition &poly_partition)
{

    // bounded contour
    std::vector<Pnt3> contour;
    double polygon_len = CalPolygonLength(m_sketchPolyLineOnXY);
    DiscreteLineByLen(m_sketchPolyLineOnXY, polygon_len, 10, contour); // ToDO: tol=3: smaller tol give more ssnodes
    if (IsClockWise(contour))
    {
        std::reverse(contour.begin(), contour.end());
    }

    for (typename std::list<SSBranchT>::iterator bit = branches.begin();
         bit != branches.end(); ++bit)
    {
        SSBranchT &b = *bit;
        simplify_one_branch(b, DP_THRESH, contour); // use the default dp thresh
    }

    M_DEBUG << "simplify branches success!" << endl;
}
#endif

void XSubToy::SimplifyPolyLineDP(const vector<Pnt3> &oriPArr, vector<bool> &keepMark, float tol)
{
    int i;
    int oriSize = static_cast<int>(oriPArr.size());
    if (oriSize <= 2) //no need simplify
    {
        keepMark.push_back(true);
        keepMark.push_back(true);
        return;
    }
    int curSize = static_cast<int>(oriPArr.size());
    keepMark.resize(curSize, false);
    keepMark.front() = true;
    keepMark.back() = true; // mark the first and last vertices
    SimplifyDP(oriPArr, 0, curSize - 1, keepMark, tol);
}

void XSubToy::create_tree_skel(const std::vector<NodeT *> &junc_nodes,
                               std::unordered_map<NodeT *, std::vector<SSBranchT *>> &node_branches,
                               NodeT *root,
                               SSkel<NodeT> &simp_sskel)
{

    std::list<NodeT *> Q;
    Q.push_back(root);
    std::unordered_map<SSBranchT *, bool> isAdded; // is branch added
    std::unordered_map<NodeT *, bool> isNodeVisit;
    std::unordered_map<NodeT *, NodeT *> old2New;
    // M_DEBUG << " add new junc node..." << std::endl;
    for (int i = 0; i < junc_nodes.size(); i++)
    {
        NodeT *jnode = junc_nodes[i];
        NodeT *newnode = simp_sskel.add_node(jnode->pos);
        newnode->set_radius(jnode->radius);
        old2New[jnode] = newnode;
        // M_DEBUG << "old2New old=" << jnode->idx << " new=" << old2New[jnode]->idx << std::endl;
    }
    while (!Q.empty())
    {
        NodeT *pnode = Q.front();
        Q.pop_front();
        isNodeVisit[pnode] = true;
        // std::cout << "visit parent node" << *pnode << " node_branches=" << node_branches[pnode].size() << std::endl;
        for (SSBranchT *bi : node_branches[pnode])
        {
            // std::cout << "visit branch " << *bi << std::endl;
            NodeT *cnode = bi->st == pnode ? bi->end : bi->st;
            if (cnode->type() == NODE_JUNCTION && !isNodeVisit[cnode])
            {
                Q.push_back(cnode);
            }
            if (!isAdded[bi])
            {
                isAdded[bi] = true;
                // M_DEBUG << " newsskel add branch " << *bi << std::endl;
                std::vector<NodeT *> nodes_tmp;
                nodes_tmp.reserve(bi->simp_pnts.size());
                NodeT *st = old2New[pnode];
                nodes_tmp.push_back(st);

                if (cnode->type() == NODE_JUNCTION)
                {
                    NodeT *end = old2New[cnode];
                    if (bi->end == pnode && bi->st == cnode)
                    {
                        std::reverse(bi->simp_pnts.begin(), bi->simp_pnts.end());
                    }

                    for (int i = 1; i < bi->simp_pnts.size() - 1; i++)
                    {
                        NodeT *ni = simp_sskel.add_node(bi->simp_pnts[i]);
                        ni->set_radius(bi->simp_radius[i]);
                        nodes_tmp.push_back(ni);
                    }
                    nodes_tmp.push_back(end);
                }
                else
                {
                    for (int i = 1; i < bi->simp_pnts.size(); i++)
                    {
                        NodeT *ni = simp_sskel.add_node(bi->simp_pnts[i]);
                        ni->set_radius(bi->simp_radius[i]);
                        nodes_tmp.push_back(ni);
                    }
                }

                // for (int i = 1; i < bi->simp_pnts.size() - 1; i++)
                // {
                //     NodeT *ni = simp_sskel.add_node(bi->simp_pnts[i]);
                //     nodes_tmp.push_back(ni);
                // }
                // NodeT *end = cnode->type() == NODE_JUNCTION ? old2New[cnode] : simp_sskel.add_node(bi->simp_pnts.back());
                // nodes_tmp.push_back(end);

                for (int i = 0; i < nodes_tmp.size() - 1; i++)
                {
                    NodeT *n0 = nodes_tmp[i];
                    NodeT *n1 = nodes_tmp[i + 1];
                    EdgeT *e01 = simp_sskel.add_edge(n0, n1);
                    // std::cout << "n0 idx=" << n0->idx << " adj_edges=" << n0->adj_edges.size() << " n1 idx=" << n1->idx << " adj_edges=" << n1->adj_edges.size() << std::endl;
                }
            }
        }
    }

    M_DEBUG << "create tree skeleton successfully!\n";
}

void XSubToy::create_ribon_skel(const std::vector<NodeT *> &term_nodes,
                                const SSBranchT &b,
                                SSkel<NodeT> &simp_sskel)
{
    int num_nodes = static_cast<int>(b.simp_pnts.size());
    for (int i = 0; i < num_nodes; i++)
    {
        const Pnt3 &p = b.simp_pnts[i];
        NodeT *newNode = simp_sskel.add_node(p);
        newNode->set_radius(b.simp_radius[i]);
    }
    for (int i = 0; i < num_nodes - 1; i++)
    {
        NodeT *n0 = simp_sskel.nodes[i];
        NodeT *n1 = simp_sskel.nodes[i + 1];
        EdgeT *e = simp_sskel.add_edge(n0, n1);
    }
}

NodeT *XSubToy::node_with_max_num_branches(
    const std::unordered_map<NodeT *, std::vector<SSBranchT *>> &node_branches)
{
    // make new sskel: start from old node with most branches, add its adjacent edges
    int max_branches = -1;
    NodeT *root = nullptr;
    for (auto it = node_branches.begin(); it != node_branches.end(); ++it)
    {
        int num_branches = it->second.size();
        // M_DEBUG << " node" << jnode->idx << " num_branches=" << num_branches << std::endl;
        if (num_branches > max_branches)
        {
            max_branches = num_branches;
            root = it->first;
        }
    }
    return root;
}