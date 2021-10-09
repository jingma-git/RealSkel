// Qt
#include <QVector4D>
#include <QMessageBox>
#include <QApplication>
#include <QPointF>
#include <QObject>
// Core
#include <core/XSubToy.h>
#include <core/XFunctions.h>
#include <core/CSketchManager.h>
#include <core/EGL.h>
#include <core/ContourExtractor.h>
#include <core/skeletor.h>
#include <igl/copyleft/cgal/partition.h>
#include <core/part_generator.h>
// egl
#include <egl/writeLine.h>
#include <egl/readLine.h>
// Viewer
#include "toyview.h"
#include "ColorManager.h"
#include "EasyGL.h"
#include "Three.h"
using namespace std;
using namespace Eigen;

//layer thickness, 0 for mesh, 0.1 for subContour spline, 0.2 for ctrlPts, ctrlRope, glueRope
#define POINT_SIZE 5.0f
#define LINE_WIDTH 3.0f

static void compute_stipple_line(const Eigen::Vector3d &s,
                                 const Eigen::Vector3d &d,
                                 const float len_step,
                                 std::vector<GLVec3> &data)
{
    Vector3d vec = d - s;
    float len = vec.norm();
    int num_steps = len / len_step;
    if (num_steps < 3)
    {
        data.emplace_back(s[0], s[1], s[2]);
        data.emplace_back(d[0], d[1], d[2]);
    }
    else
    {
        Vector3d unit_vec = vec / num_steps;
        for (int i = 0; i < num_steps; i++)
        {
            if (i % 2)
            {
                Vector3d p = s + i * unit_vec;
                data.emplace_back(p[0], p[1], p[2]);
            }
        }
    }
}

static GLVec3 heat_color(float s)
{
    using namespace Eigen;
    GLVec3 zero(1.0, 1.0, 1.0);
    GLVec3 one(1.0, 0.0, 0.0);
    GLVec3 plus(1.0, 1.0, 0.0);
    GLVec3 minus(0.0, 0.0, 1.0);

    if (s >= 2)
    {
        return plus; // Yellow
    }
    else if (s >= 1)
    {
        return (1 - (s - 1)) * one + (s - 1) * plus; // Red-yellow
    }
    else if (s >= 0)
    {
        return (1 - s) * zero + (s)*one; // White-red
    }
    else if (s >= -1)
    {
        return (1 - (s + 1)) * minus + (s + 1) * zero; // Blue-white
    }
    else
    {
        return minus; // Blue
    }
}

// QColor ToyView::CTRL_PT_COLOR(102, 51, 0, 255);
// QColor ToyView::SUBMESH_COLOR(222, 222, 222);
// QColor ToyView::PARENT_MESH_COLOR(204, 153, 255);

QVector<QColor> ToyView::rnd_colors = ColorManager::rndColors(200);

ToyView::ToyView()
{
    // set data
    m_toy = new XToy;
    m_subContourPool.SetBlockSize(XToy::NUM_INI_SUBTOYS);
    m_subCtrlPtsPool.SetBlockSize(XToy::NUM_INI_SUBTOYS);

    // sketch line
    sketchgl.setEdgePtr(&m_sketchline);
    sketchgl.setColor(QColor(Qt::red));
    sketchgl.setEdgeWidth(LINE_WIDTH);
    // outer contour
    contour_gl.setEdgePtr(&contourPts);
    contour_gl.setColor(QColor(Qt::red));
    contour_gl.setEdgeWidth(LINE_WIDTH);
    // def contour
    defContour_gl.setColor(QColor(Qt::red));
    defCtrlPts_gl.setColor(CTRL_PT_COLOR);
    // skeleton
    m_skelGL = new SkelGL(m_toy->skel);
    m_restSkelGL = new SkelGL(m_toy->restSkel);
    // arcball
    m_arcBallGL = new ArcBallGL();
    // plane
    m_planeGL = new PlaneGL();
}

ToyView::~ToyView()
{
    clearSubToyGL();
    delete m_toy;
    delete m_skelGL;
    delete m_restSkelGL;
    delete m_arcBallGL;
}

void ToyView::clearSubToyGL()
{
    for (EdgeContainer *subContGL : m_subContourGL)
        m_subContourPool.Recycle(subContGL);
    m_subContourGL.clear();
    subContours.clear();

    for (PointContainer *subCtrlPtsGL : m_subCtrlPtsGL)
        m_subCtrlPtsPool.Recycle(subCtrlPtsGL);
    m_subCtrlPtsGL.clear();
    subCtrlPts.clear();

    for (size_t i = 0; i < m_subMeshGLCoarse.size(); ++i) // 2D Coarse Mesh
    {
        delete m_subMeshGLCoarse[i];
    }
    m_subMeshGLCoarse.clear();

    for (size_t i = 0; i < m_subMeshGLFine.size(); ++i)
    {
        delete m_subMeshGLFine[i];
    }
    m_subMeshGLFine.clear();

    for (size_t i = 0; i < m_subMeshGLDeform.size(); ++i)
    {
        delete m_subMeshGLDeform[i];
    }
    m_subMeshGLDeform.clear();

    for (size_t i = 0; i < m_subMeshGL3D.size(); ++i)
    {
        delete m_subMeshGL3D[i];
    }
    m_subMeshGL3D.clear();
}

void ToyView::load(QString in_dir)
{
    clearSubToyGL();
    m_toy->load(in_dir.toStdString());

    // visualization
    int n = static_cast<int>(m_toy->m_subToys.size());
    for (int i = 0; i < n; ++i)
    {
        XSubToy *subToy = m_toy->m_subToys[i];
        // Contour Visulization
        getSubContourGL(subToy->GetID());
        getSubCtrlPtsGL(subToy->GetID());

        //2D Mesh Visulization
        MeshGL *meshglCoarse = getSubMeshGLCoarse(subToy->GetID(), &subToy->m_curMesh2DTemp);
        MeshGL *meshglFine = getSubMeshGLFine(subToy->GetID(), &subToy->m_curMesh2D);
        MeshGL *meshglDeform = getSubMeshGLDeform(subToy->GetID(), &subToy->m_defMesh2D);
        //3D Mesh Visulization
        MeshGL *meshgl = getSubMeshGL3D(subToy->GetID(), &subToy->m_curMesh3D);
    }

    updateSubToyGL();
}

void ToyView::save(std::string out_dir)
{
    m_toy->save(out_dir);
}

void ToyView::removeViewer(EasyGL *gl)
{
    M_DEBUG << " ToyView::removeViewer " << gl;
    // Components
    sketchgl.removeViewer(gl);
    defContour_gl.removeViewer(gl);
    defCtrlPts_gl.removeViewer(gl);
    for (int i = 0; i < m_subContourGL.size(); i++)
    {
        m_subContourGL[i]->removeViewer(gl);
        m_subCtrlPtsGL[i]->removeViewer(gl);
    }
    for (int i = 0; i < m_subMeshGLCoarse.size(); i++)
    {
        m_subMeshGLCoarse[i]->removeViewer(gl);
    }
    for (int i = 0; i < m_subMeshGLFine.size(); i++)
    {
        m_subMeshGLFine[i]->removeViewer(gl);
    }
    for (int i = 0; i < m_subMeshGLDeform.size(); i++)
    {
        m_subMeshGLDeform[i]->removeViewer(gl);
    }
    for (int i = 0; i < m_subMeshGL3D.size(); i++)
    {
        m_subMeshGL3D[i]->removeViewer(gl);
    }

    // Merged mesh
    if (m_meshGL2D)
        m_meshGL2D->removeViewer(gl);
    if (m_meshGL3D)
        m_meshGL3D->removeViewer(gl);
    if (m_skelGL)
    {
        m_skelGL->removeViewer(gl);
    }

    if (m_restSkelGL)
    {
        m_restSkelGL->removeViewer(gl);
    }

    if (m_arcBallGL)
    {
        m_arcBallGL->removeViewer(gl);
    }

    if (m_planeGL)
    {
        m_planeGL->removeViewer(gl);
    }
}

XSubToy *ToyView::getCurToy()
{
    return m_toy->GetCurToy();
}

void ToyView::draw_line(EasyGL *gl)
{
    if (m_sketchline.size() < 3)
        return;

    sketchgl.draw(gl);
}

void ToyView::update_line(EasyGL *gl)
{
    if (gl->m_opMode == OPMODE_DRAW_SYM)
    {
        m_sketchline.clear();
        std::vector<Pnt3> polygon;
        m_sym_sketch.to_polygon(polygon);
        for (const Pnt3 &p : polygon)
        {
            m_sketchline.push_back(GLVec3(p.x(), p.y(), p.z()));
        }
        sketchgl.setEdgePtr(&m_sketchline);
    }
    sketchgl.update(gl);
}

void ToyView::add_to_sym_line(const QPoint &point, EasyGL *gl)
{
    Pnt3 p = gl->Cvt2Dto3D(point.x(), point.y(), gl->GetWorldCenterZ());
    m_sym_sketch.set_view_dir(gl->GetViewDir());
    m_sym_sketch.add_pt(p);
}

void ToyView::set_plane(const Plane &plane)
{
    m_planeGL->set_plane(plane);
    m_planeGL->compute_element();

    for (auto v : EasyGL::EasyGLPool())
    {
        v->m_isDrawPlane = true;
        m_planeGL->update(v);
        v->repaint();
    }
}

void ToyView::draw_plane(EasyGL *gl)
{
    m_planeGL->draw(gl);
}

void ToyView::add_outer_contour(const std::vector<Pnt3> &outer_contour, EasyGL *gl)
{
    PartGenerator pg(outer_contour); //part_generator: ToDO: auto sketchline fitting
    M_DEBUG << "add_outer_contour" << endl;
    for (int i = 0; i < static_cast<int>(outer_contour.size()); ++i)
    {
        if (pg.part_idxs[i] == -1)
        {
            const Pnt3 &p = outer_contour[i];
            contourPts.emplace_back(p.x(), p.y(), p.z());
        }
    }
    M_DEBUG << "add_outer_contour successfully!" << endl;
}

void ToyView::draw_outer_contour(EasyGL *gl)
{
    contour_gl.draw(gl);
}

void ToyView::update_outer_contour(EasyGL *gl)
{
    contour_gl.update(gl);
}

void ToyView::draw_subContours(EasyGL *gl)
{
    if (subContours.size() <= 0)
        return;
    for (size_t i = 0; i < m_subContourGL.size(); ++i)
    {
        // !!! must add this line, because
        // subContours.push_back(gl_spline); //!!!this is where deallocation happens
        m_subContourGL[i]->setEdgePtr(&subContours[i]);
        m_subContourGL[i]->draw(gl);
    }
}

void ToyView::draw_subCtrlPts(EasyGL *gl)
{
    if (subCtrlPts.size() <= 0)
        return;
    int i = m_toy->m_curSubToyIdx;
    if (i < 0 || i > m_subCtrlPtsGL.size())
        return;

    float point_size = POINT_SIZE * gl->GetNDCDisOfOnePixelOn2D();
    m_subCtrlPtsGL[i]->setPointSize(point_size);
    m_subCtrlPtsGL[i]->setPointPtr(&subCtrlPts[i]);
    m_subCtrlPtsGL[i]->draw(gl);
}

void ToyView::sketchline2Spline(const vector<Pnt3> &ct_pts3, CSpline &spline, EasyGL *gl)
{
    // backend
    vector<Pnt3> ct_simPts3, ct_uniPts3;
    float fPixelDis = gl->Get3dDisOfOnePixelOn2D();
    // qDebug() << __FILE__ << " " << __LINE__ << " fPixelDis=" << fPixelDis;
    float tol = fPixelDis * 2;
    float avgLen = 40 * tol;
    SimplifyPolyLine(ct_pts3, ct_simPts3, tol);
    UniformPolyLine(ct_simPts3, ct_uniPts3, avgLen);
    spline.ChangeMode(CSpline::SPLMODE_CLOSED_SPLINE);
    while ((ct_uniPts3[0] - ct_uniPts3.back()).norm() < POINT_SIZE) //ToDO: improve
    {
        ct_uniPts3.pop_back();
    }
    for (const Pnt3 &p : ct_uniPts3)
    {
        spline.AddCtrlPoint(p);
    }
}

void ToyView::extract_part_contour(const std::vector<Pnt3> &outer_contour, EasyGL *gl)
{
    double line_len = CalLineLength(outer_contour);
    std::vector<Pnt3> simp_polyline;
    SimplifyPolyLine(outer_contour, simp_polyline, 10);
    if ((simp_polyline.back() - simp_polyline.front()).norm() < 0.1)
    {
        simp_polyline.pop_back();
    }

    std::vector<std::vector<Pnt3>> part_polys;
    igl::copyleft::cgal::partition(simp_polyline, part_polys);
    M_DEBUG << "part_polys=" << part_polys.size() << endl;

    cv::Mat img = cv::Mat::zeros(gl->height(), gl->width(), CV_8UC3);
    for (int i = 0; i < static_cast<int>(part_polys.size()); ++i)
    {
        std::vector<Pnt3> &part_poly = part_polys[i];
        std::vector<cv::Point> pts2d(part_poly.size());
        cv::Scalar color(rand() % 255, rand() % 255, rand() % 255);
        for (int j = 0; j < static_cast<int>(part_poly.size()); ++j)
        {
            const Pnt3 &p = part_poly[j];
            gl->Cvt3Dto2D(int(p[0]), int(p[1]), int(p[2]), pts2d[j].x, pts2d[j].y);
            cv::circle(img, pts2d[j], 3, color, -1);
            cv::putText(img, std::to_string(j), pts2d[j], cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
        }
        for (int j = 0; j < static_cast<int>(pts2d.size() - 1); ++j)
        {
            cv::line(img, pts2d[j], pts2d[j + 1], color, 1);
        }
    }
    cv::imwrite("output/part_poly.jpg", img);
}

EdgeContainer *ToyView::getSubContourGL(int id)
{
    int n = static_cast<int>(m_subContourGL.size());
    XSubToy *subToy = m_toy->m_subToys[id];

    if (id > n - 1) // newly added subtoy
    {
        // interpolated spline
        std::vector<GLVec3> gl_spline;
        for (const Pnt3 &p : subToy->m_sketchPolyLine)
        {
            gl_spline.push_back(GLVec3(p.x(), p.y(), p.z() + 0.1));
        }
        subContours.push_back(gl_spline); //!!!this is where deallocation happens

        EdgeContainer *edgeContainer = m_subContourPool.Request();
        new (edgeContainer) EdgeContainer();
        m_subContourGL.push_back(edgeContainer);
        m_subContourGL[id]->setColor(rnd_colors[id]);
        m_subContourGL[id]->setEdgePtr(&subContours[id]);
        return edgeContainer;
    }

    EdgeContainer *edgeContainer = m_subContourGL[id];
    subContours[id].clear();
    int sign = subToy->m_layerDepth >= 0 ? 1 : -1;
    for (const Pnt3 &p : subToy->m_sketchPolyLine)
    {
        subContours[id].push_back(GLVec3(p.x(), p.y(), p.z() + sign * 0.1));
    }
    edgeContainer->setEdgePtr(&subContours[id]);
    return edgeContainer;
}

PointContainer *ToyView::getSubCtrlPtsGL(int id)
{
    int n = static_cast<int>(m_subCtrlPtsGL.size());
    XSubToy *subToy = m_toy->m_subToys[id];
    CSpline &spline = subToy->m_sketchSpline;

    if (id > n - 1) // newly added subtoy
    {
        // spline control points
        vector<GLVec3> gl_ctrlPts;
        for (int i = 0; i < spline.GetCtrlPointCount(); i++)
        {
            const Pnt3 &p = spline.GetCtrlPoint(i);
            gl_ctrlPts.push_back(GLVec3(p.x(), p.y(), p.z() + 0.2));
        }
        subCtrlPts.push_back(gl_ctrlPts); //!!!this is where deallocation happens

        PointContainer *pointContainer = m_subCtrlPtsPool.Request();
        new (pointContainer) PointContainer();
        pointContainer->setColor(CTRL_PT_COLOR);
        pointContainer->setPointPtr(&subCtrlPts[id]);
        m_subCtrlPtsGL.push_back(pointContainer);
        return pointContainer;
    }

    PointContainer *pointContainer = m_subCtrlPtsGL[id];
    int sign = subToy->m_layerDepth >= 0 ? 1 : -1;
    subCtrlPts[id].clear();
    for (int i = 0; i < spline.GetCtrlPointCount(); i++)
    {
        const Pnt3 &p = spline.GetCtrlPoint(i);
        subCtrlPts[id].push_back(GLVec3(p.x(), p.y(), p.z() + sign * 0.2));
    }
    pointContainer->setPointPtr(&subCtrlPts[id]);
    return pointContainer;
}

void ToyView::computeSubContour(int id)
{
    getSubContourGL(id);
    getSubCtrlPtsGL(id);
}

#ifdef SSKEL_DEBUG
#include <core/sskel_draw.h>
#endif
bool ToyView::add_subContour(const std::vector<GLVec3> &sketchline, EasyGL *gl) // the first most important function!!!
{
    if (sketchline.size() < 3)
        return false;
    if (gl->m_viewMode == VIEW_PERSP)
    {
        QMessageBox::warning(QApplication::activeWindow(),
                             "ERROR",
                             "Cannot sketch under perspective view!\n");
        return false;
    }

    // backend
    // sketchline-->simplify-->uniform discretization-->cubic interpolation-->m_sketchSpline
    vector<Pnt3> ct_pts3, ct_simPts3, ct_uniPts3;
    for (const GLVec3 &p : sketchline)
    {
        // ct_pts3.push_back(Pnt3(p.x, p.y, 0.0));
        ct_pts3.push_back(Pnt3(p.x, p.y, p.z));
    }

#ifdef SSKEL_DEBUG
    std::vector<Pnt3> input_pts;
    egl::readLine("/home/server/MaJing/cpp_proj/shadow_play/input/poly_subToy0.txt", input_pts);

    SkelDrawer drawer;
    drawer.set_sil(input_pts);
    double input_len = CalPolygonLength(input_pts);
    DiscreteLineByLen(input_pts, input_len, 2, ct_pts3);
    cv::Mat img_raw, img_raw_noised, img_raw_smooth;
    img_raw = drawer.draw_sketch_line(ct_pts3);
    // check sensitivity to small shape changes
    int num_pts = static_cast<int>(ct_pts3.size());
    for (int i = 0; i < num_pts;)
    {
        int flag = rand() % 2;
        if (flag)
        {
            int num_disturb = rand() % 10;
            int xoff = rand() % 4;
            int sign = rand() % 2;
            int count = 0;
            while (count < num_disturb)
            {
                Pnt3 &p = ct_pts3[i];
                p[0] += (sign == 0 ? -1 : 1) * xoff;
                ++i;
                ++count;
            }
        }
        else
        {
            ++i;
        }
    }
    egl::writeLine("input/poly_noised.txt", ct_pts3);
    img_raw_noised = drawer.draw_sketch_line(ct_pts3);
    cv::imwrite("output/raw_sketch.png", img_raw);
    cv::imwrite("output/raw_sketch_noised.png", img_raw_noised);
    // SmoothPolyLine(ct_pts3, ct_pts3, 0.2, 20);
    // SmoothPolyLineBiLaplician(ct_pts3, ct_pts3, 0.2, 20);
    DiscreteLineByLen(input_pts, input_len, 10, ct_pts3);
    img_raw_smooth = drawer.draw_sketch_line(ct_pts3);
    cv::imwrite("output/raw_sketch_smooth.png", img_raw_smooth);
#endif

    float fPixelDis = gl->Get3dDisOfOnePixelOn2D();
    float tol = fPixelDis * 2;
    float avgLen = 40 * tol;
    double ct_pts3Len = CalLineLength(ct_pts3);
    M_DEBUG << "fPixelDis=" << fPixelDis << " ct_pts3Len=" << ct_pts3Len << endl;
    std::vector<Pnt3> ct_pts3Tmp(ct_pts3);
    DiscreteLineByLen(ct_pts3Tmp, ct_pts3Len, 10, ct_pts3);
    SimplifyPolyLine(ct_pts3, ct_simPts3, tol);
    UniformPolyLine(ct_simPts3, ct_uniPts3, avgLen);
    if (IsClockWise(ct_uniPts3))
    {
        std::reverse(ct_uniPts3.begin(), ct_uniPts3.end());
    }

    XSubToy *subToy = m_toy->AddSubToy();
    subToy->SetInitViewDir(gl->GetViewDir());
    subToy->SetLayerID(gl->m_curLayer);
    if (gl->m_opMode == OPMODE_DRAW_SYM)
    {
        subToy->set_skel_opt(SKEL_SYM);
        subToy->SetSymLine(m_sym_sketch.sym_line());
        M_DEBUG << "SetSymLine successfully!" << endl;
    }

    // Prepare discrete line for triangulation and skeletonization
    subToy->InitContour(ct_uniPts3);
    subToy->SetCentroid();
    subToy->SetRotToXY();
    subToy->SetSketchPolyLineOnXY();
    subToy->ProcessSelfIsectSketchLine();
    egl::writeLine("output/sketchPolyOnXY.txt", subToy->m_sketchPolyLineOnXY);
    double pixel_dis = gl->Get3dDisOfOnePixelOn2D();
    // test sketch-line self-intersections
    std::vector<Pnt3> contPtsOnXY;
    subToy->Get2DDiscreteLine(subToy->m_sketchPolyLineOnXY, contPtsOnXY, 10, pixel_dis);
    // if (IsLineSelfIntersect(contPtsOnXY)) // comment out according to Reviewer2's advice
    // {
    // QMessageBox::warning(QApplication::activeWindow(),
    //                      "ERROR",
    //                      "Sketch line self-intersect!");
    // m_toy->DelSubToy(subToy);
    // return false;

    // }

    // visualization
    getSubContourGL(subToy->GetID());
    getSubCtrlPtsGL(subToy->GetID());

    M_DEBUG << *subToy << endl;
    return true;
}

void ToyView::deleteSubContour(int id)
{
    if (id != static_cast<int>(subContours.size()) - 1)
    {
        M_DEBUG << "to be deleted subContour is not the last one " << id << endl;
        std::swap(subContours[id], subContours.back());
        std::swap(subCtrlPts[id], subCtrlPts.back());
        std::swap(m_subContourGL[id], m_subContourGL.back());
        std::swap(m_subCtrlPtsGL[id], m_subCtrlPtsGL.back());
    }

    subContours.pop_back();
    subCtrlPts.pop_back();
    m_subContourGL.pop_back();
    m_subCtrlPtsGL.pop_back();
}

bool ToyView::delete_subContour()
{
    XSubToy *subToy = m_toy->GetCurToy();
    int id = subToy->GetID();
    M_DEBUG << " delete_subContour " << id << endl;
    if (id != static_cast<int>(m_toy->m_subToys.size()) - 1)
    {
        M_DEBUG << "to be deleted subToy is not the last one " << id << endl;
        std::swap(subContours[id], subContours.back());
        std::swap(subCtrlPts[id], subCtrlPts.back());
        std::swap(m_subContourGL[id], m_subContourGL.back());
        std::swap(m_subCtrlPtsGL[id], m_subCtrlPtsGL.back());
    }

    subContours.pop_back();
    subCtrlPts.pop_back();
    m_subContourGL.pop_back();
    m_subCtrlPtsGL.pop_back();

    m_toy->DelSubToy(subToy);

    for (auto v : EasyGL::EasyGLPool())
    {
        update_subContours(v);
        v->repaint();
    }
    return true;
}

void ToyView::update_subContours(EasyGL *gl)
{
    for (int i = 0; i < static_cast<int>(m_subContourGL.size()); ++i)
    {
        m_subContourGL[i]->update(gl);
        m_subCtrlPtsGL[i]->update(gl);
    }
}

void ToyView::update_subContours(int id, EasyGL *gl)
{
    m_subContourGL[id]->update(gl);
    m_subCtrlPtsGL[id]->update(gl);
}

void ToyView::add_symetry_toy(EasyGL *gl, SymetryPlane sym_plane, bool wstParent)
{
    if (getGlobalMode(gl->m_opMode) == OPMODE_OBJECT_MODE)
    {
        XSubToy *oSubToy = m_toy->GetCurToy();                                 // original subToy
        double plane_pos = wstParent ? oSubToy->CalSymPlanePos(sym_plane) : 0; // ToDO: let the user specify symmetry position
        XSubToy *subToy = m_toy->AddSymSubToy(oSubToy, sym_plane, plane_pos);

        // Contour Visulization
        getSubContourGL(subToy->GetID());
        getSubCtrlPtsGL(subToy->GetID());
        //2D Mesh Visulization
        MeshGL *meshglCoarse = getSubMeshGLCoarse(subToy->GetID(), &subToy->m_curMesh2DTemp);
        MeshGL *meshglFine = getSubMeshGLFine(subToy->GetID(), &subToy->m_curMesh2D);
        MeshGL *meshglDeform = getSubMeshGLDeform(subToy->GetID(), &subToy->m_defMesh2D);
        //3D Mesh Visulization
        MeshGL *meshgl = getSubMeshGL3D(subToy->GetID(), &subToy->m_curMesh3D);

        updateSubToyGL();
    }
}

void ToyView::compute_chordal_axis()
{
    // if (m_toy->m_subToys.size() > 1)
    // {
    //     cout << __FILE__ << " " << __LINE__ << " only support one SubToy for Debug purpose" << endl;
    //     return;
    // }

    // subChordals.resize(m_toy->m_subToys.size());
    // subChordal_gl.resize(m_toy->m_subToys.size());
    // for (int i = 0; i < m_toy->m_subToys.size(); i++)
    // {
    //     XSubToy *subToy = m_toy->m_subToys[i];
    //     if (!subToy->m_isChordalInit)
    //     {
    //         subChordals[i].resize(subToy->m_vAxisLines.size());
    //         subChordal_gl[i].resize(subToy->m_vAxisLines.size(), nullptr);

    //         for (int li = 0; li < subToy->m_vAxisLines.size(); li++) //li: line index
    //         {
    //             C3DLine &axisLine = subToy->m_vAxisLines[li];
    //             Line3 &chordal = subChordals[i][li];
    //             chordal.resize(axisLine.GetiptCount() + 2);
    //             // layerid * L_THICK
    //             const Pnt3 &p_st = axisLine.Getcpt(0);
    //             const Pnt3 &p_end = axisLine.Getcpt(1);
    //             chordal[0] = GLVec3(p_st.x(), p_st.y(), p_st.z());
    //             for (int j = 0; j < axisLine.GetiptCount(); j++)
    //             {
    //                 const Pnt3 &p = axisLine.Getipt(j);
    //                 chordal[j + 1] = GLVec3(p.x(), p.y(), p.z());
    //                 // cout << __FILE__ << " " << __LINE__ << " " << j << " p=" << p.transpose() << " c=" << subChordals[i][li][j + 1] << endl;
    //             }
    //             chordal[chordal.size() - 1] = GLVec3(p_end.x(), p_end.y(), p_end.z());

    //             EdgeContainer *edgeContainer = new EdgeContainer;
    //             edgeContainer->edge_ptr = &subChordals[i][li];
    //             if (li < 18)
    //                 edgeContainer->setColor(colors[li]);
    //             else
    //                 edgeContainer->setColor(rnd_colors[li]);
    //             subChordal_gl[i][li] = edgeContainer;
    //             QColor &lineC = edgeContainer->color;
    //             cout << __FILE__ << " " << __LINE__ << " "
    //                  << li << ":" << axisLine.Getcf(0) << "-" << axisLine.Getcf(1)
    //                  << " isects=" << axisLine.GetiptCount()
    //                  << " color=" << lineC.redF() << "," << lineC.greenF() << "," << lineC.blueF() << endl;
    //         }
    //         subToy->m_isChordalInit = true;
    //     }
    // }
}

void ToyView::draw_chordal_axis(EasyGL *gl)
{
    // for (int i = 0; i < subChordal_gl.size(); i++)
    // {
    //     for (int li = 0; li < subChordal_gl[i].size(); li++)
    //     {
    //         if (subChordal_gl[i][li])
    //         {

    //             EdgeContainer *edgeContainer = subChordal_gl[i][li];
    //             Line3 &chordal = *edgeContainer->edge_ptr;
    //             // edgeContainer->edge_ptr = &subChordals[i][li];
    //             // C3DLine &axisLine = m_toy->m_subToys[i]->m_vAxisLines[li];
    //             // const Pnt3 &p_st = axisLine.Getcpt(0);
    //             // const Pnt3 &p_end = axisLine.Getcpt(1);
    //             // const GLVec3 &c_st = chordal.front();
    //             // const GLVec3 &c_end = chordal.back();
    //             // double err = 0;
    //             // err += (Pnt3(c_st.x, c_st.y, c_st.z) - p_st).norm();
    //             // for (int j = 0; j < axisLine.GetiptCount(); j++)
    //             // {
    //             //     const Pnt3 &p = axisLine.Getcpt(j);
    //             //     const GLVec3 &c = chordal[j + 1];
    //             //     err += (Pnt3(c.x, c.y, c.z) - p).norm();
    //             // }
    //             // err += (Pnt3(c_end.x, c_end.y, c_end.z) - p_end).norm();
    //             // qDebug() << __FILE__ << " " << __LINE__ << " subToy" << i << " chordal" << li << " err=" << err;
    //             // qDebug() << __FILE__ << " " << __LINE__ << " subToy" << i << " chordal" << li << " isects=" << chordal.size();
    //             subChordal_gl[i][li]->draw(gl);
    //         }
    //     }
    // }
}

void ToyView::addSubMeshCoarse(EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();
    subToy->TriangulateCoarseFrom3DContour();
    getSubMeshGLCoarse(subToy->GetID(), &subToy->m_curMesh2DTemp);
}

MeshGL *ToyView::getSubMeshGLCoarse(int id, TMesh *mesh_ptr)
{
    int n = static_cast<int>(m_subMeshGLCoarse.size());
    if (id > n - 1)
    {
        MeshGL *meshgl = new MeshGL(mesh_ptr);
        m_subMeshGLCoarse.push_back(meshgl);
        return meshgl;
    }

    MeshGL *meshgl = m_subMeshGLCoarse[id];
    if (meshgl->mesh != mesh_ptr)
    {
        M_DEBUG << "subMeshGLCoarse" << id << ": old_mesh_ptr=" << meshgl->mesh << " new_mesh_ptr=" << mesh_ptr << endl;
        meshgl->set_mesh(mesh_ptr);
    }
    return meshgl;
}

void ToyView::deleteSubMeshCoarse(int id)
{
    int n = static_cast<int>(m_subMeshGLCoarse.size());
    if (id != n - 1)
    {
        std::swap(m_subMeshGLCoarse[id], m_subMeshGLCoarse.back());
    }
    m_subMeshGLCoarse.pop_back();
}

void ToyView::draw_subMeshCoarse(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLCoarse.size(); ++i)
    {
        m_subMeshGLCoarse[i]->draw_mesh(gl);
    }
}

void ToyView::updateSubMeshCoarse(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLCoarse.size(); i++)
    {
        m_subMeshGLCoarse[i]->update(gl);
    }
}

void ToyView::pick_all_subtoys()
{
    M_DEBUG << " pickAllSubToys " << endl;
    m_toy->pick_all_subtoys();
    update_mesh_color();
}

void ToyView::pick_current_subtoy()
{
    M_DEBUG << " pick_current_subtoy" << endl;
    m_toy->pick_current_subtoy();
    update_mesh_color();
}

void ToyView::unpick_all_subtoys()
{
    M_DEBUG << " unpickAllSubToys " << endl;
    m_toy->unpick_all_subtoys();
    update_mesh_color();
}

bool ToyView::pickSubToy(const QPoint &pos, EasyGL *gl, bool ctrl_down)
{
    using namespace Eigen;
    qDebug() << __FILE__ << " " << __LINE__ << " pickSubToy " << pos;
    m_toy->pick_subtoy(pos.x(), pos.y(), gl->width(), gl->height(), gl->m_mvp.data(), ctrl_down);
    if (!ctrl_down)
        pick_current_subtoy();
    update_mesh_color();
    return true;
}

bool ToyView::pickParentSubToy(const QPoint &pos, EasyGL *gl)
{
    using namespace Eigen;
    qDebug() << __FILE__ << " " << __LINE__ << " pickParentSubToy " << pos;
    m_toy->pick_parent_subtoy(pos.x(), pos.y(), gl->width(), gl->height(), gl->m_mvp.data());

    update_mesh_color();
    return true;
}

void ToyView::update_mesh_color()
{
    for (int i = 0; i < static_cast<int>(m_toy->m_subToys.size()); ++i)
    {
        XSubToy *subToy = m_toy->m_subToys[i]; // ToDO: consider the situation we delete model
        if (subToy == m_toy->GetParentToy())
        {
            QColor color(PARENT_MESH_COLOR);
            if (subToy->m_isSelected)
            {
                color = QColor(color.red() - 50, color.green() - 50, color.blue() - 50);
            }
            if (m_subMeshGLFine[i]->mesh_color != color)
            {
                // qDebug() << __FILE__ << " " << __LINE__ << subToy->name().c_str() << " is selected"
                //          << " its original color=" << m_subMeshGLFine[i]->mesh_color;
                m_subMeshGLFine[i]->setMeshColor(color);
                m_subMeshGLFine[i]->compute_color();
                m_subMeshGL3D[i]->setMeshColor(color);
                m_subMeshGL3D[i]->compute_color();

                for (EasyGL *v : EasyGL::EasyGLPool())
                {
                    m_subMeshGLFine[i]->update(v);
                    m_subMeshGL3D[i]->update(v);
                }
            }

            continue;
        }
        if (subToy->m_isSelected)
        {
            const QColor &c = SUBMESH_COLOR;
            QColor color(c.red() - 50, c.green() - 50, c.blue() - 50);
            // QColor color(255, 255, 0);
            // M_DEBUG << "update_mesh_color XSubToy " << subToy->name() << endl;
            if (m_subMeshGLFine[i]->mesh_color != color)
            {
                // qDebug() << __FILE__ << " " << __LINE__ << subToy->name().c_str() << " is selected"
                //          << " its original color=" << m_subMeshGLFine[i]->mesh_color;
                m_subMeshGLFine[i]->setMeshColor(color);
                m_subMeshGLFine[i]->compute_color();
                m_subMeshGL3D[i]->setMeshColor(color);
                m_subMeshGL3D[i]->compute_color();

                for (EasyGL *v : EasyGL::EasyGLPool())
                {
                    m_subMeshGLFine[i]->update(v);
                    m_subMeshGL3D[i]->update(v);
                }
            }
        }
        else
        {
            if (m_subMeshGLFine[i]->mesh_color != SUBMESH_COLOR)
            {
                // qDebug() << __FILE__ << " " << __LINE__ << subToy->name().c_str() << " is de-selected"
                //          << " its original color=" << m_subMeshGLFine[i]->mesh_color;
                m_subMeshGLFine[i]->setMeshColor(SUBMESH_COLOR);
                m_subMeshGLFine[i]->compute_color();
                m_subMeshGL3D[i]->setMeshColor(SUBMESH_COLOR);
                m_subMeshGL3D[i]->compute_color();
                for (EasyGL *v : EasyGL::EasyGLPool())
                {
                    m_subMeshGLFine[i]->update(v);
                    m_subMeshGL3D[i]->update(v);
                }
            }
        }
    }
}

void ToyView::translate(const QPoint &pos, EasyGL *gl)
{
    if (m_toy->m_curSubToyIdx == -1)
        return;
    m_toy->move_subtoy(pos.x(), pos.y(), gl->width(), gl->height(), gl->m_mvp.data());

    int id = m_toy->m_curSubToyIdx;
    computeSubContour(id);
    m_subMeshGLCoarse[id]->compute_element();
    m_subMeshGLFine[id]->compute_element();
    m_subMeshGLDeform[id]->compute_element();
    m_subMeshGL3D[id]->compute_element();
    m_skelGL->compute_element(gl);
    m_restSkelGL->compute_element(gl);
    update_mesh_color();
}

void ToyView::rotate(int axis, const QPoint &curP, const QPoint &lastP, EasyGL *gl)
{
    const Pnt3 &centroid = m_arcBallGL->centroid();
    Pnt2 p = gl->Cvt3Dto2D(centroid);
    Pnt3 p0(p[0], p[1], 0);
    Pnt3 p1(curP.x(), curP.y(), 0);
    Pnt3 p2(lastP.x(), lastP.y(), 0);
    Vec3 p01 = (p1 - p0).normalized();
    Vec3 p02 = (p2 - p0).normalized();
    double det = p01.x() * p02.y() - p01.y() * p02.x();
    double sign = det > 0 ? 1 : -1;
    double angle = sign * std::acos(p01.dot(p02)) * 180.0 / M_PI;
    // qDebug() << "rotate... curP=" << curP << " lastP=" << lastP << " angle=" << angle;
    m_arcBallGL->set_cur_point(gl->Cvt2Dto3D(curP.x(), curP.y(), centroid[2]), gl);

    rotate(axis, angle, gl);
}

void ToyView::rotate(int axis, int angle, EasyGL *gl)
{
    if (m_toy->m_curSubToyIdx == -1)
        return;
    m_toy->rotate_subtoy(axis, angle);

    int id = m_toy->m_curSubToyIdx;
    computeSubContour(id);
    m_subMeshGLCoarse[id]->compute_element();
    m_subMeshGLFine[id]->compute_element();
    m_subMeshGLDeform[id]->compute_element();
    m_subMeshGL3D[id]->compute_element();
    m_skelGL->compute_element(gl);
    m_restSkelGL->compute_element(gl);
    update_mesh_color();
}

void ToyView::setRotCenter()
{
    const Pnt3 &centroid = m_toy->GetCurToy()->m_centroid;
    m_arcBallGL->set_centroid(centroid);
}

void ToyView::draw_arcBall(EasyGL *gl)
{
    m_arcBallGL->draw(gl);
}

void ToyView::addSubMeshFine(EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();
    subToy->TriangulateFineFrom3DContour();
    subToy->m_defMesh2D = subToy->m_curMesh2D;
    getSubMeshGLFine(subToy->GetID(), &subToy->m_curMesh2D);
    getSubMeshGLDeform(subToy->GetID(), &subToy->m_defMesh2D);
}

MeshGL *ToyView::getSubMeshGLFine(int id, TMesh *mesh_ptr)
{
    int n = static_cast<int>(m_subMeshGLFine.size());
    if (id > n - 1)
    {
        MeshGL *meshgl = new MeshGL(mesh_ptr);
        m_subMeshGLFine.push_back(meshgl);
        return meshgl;
    }

    MeshGL *meshgl = m_subMeshGLFine[id];
    if (meshgl->mesh != mesh_ptr)
    {
        M_DEBUG << "subMeshGLFine" << id << ": old_mesh_ptr=" << meshgl->mesh << " new_mesh_ptr=" << mesh_ptr << endl;
        meshgl->set_mesh(mesh_ptr);
    }
    return meshgl;
}

MeshGL *ToyView::getSubMeshGLDeform(int id, TMesh *mesh_ptr)
{
    int n = static_cast<int>(m_subMeshGLDeform.size());
    if (id > n - 1)
    {
        MeshGL *meshgl = new MeshGL(mesh_ptr);
        m_subMeshGLDeform.push_back(meshgl);
        return meshgl;
    }

    MeshGL *meshgl = m_subMeshGLDeform[id];
    if (meshgl->mesh != mesh_ptr)
    {
        M_DEBUG << "subMeshGLDeform" << id << ": old_mesh_ptr=" << meshgl->mesh << " new_mesh_ptr=" << mesh_ptr << endl;
        meshgl->set_mesh(mesh_ptr);
    }
    return meshgl;
}

void ToyView::draw_subMeshFine(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLFine.size(); ++i)
    {
        m_subMeshGLFine[i]->draw_mesh(gl);
    }
}

void ToyView::updateSubMeshFine(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLFine.size(); ++i)
    {
        m_subMeshGLFine[i]->update(gl);
    }
}

void ToyView::updateSubMeshFine(int id, EasyGL *gl)
{
    if (id == -1)
        return;
    m_subMeshGLFine[id]->update(gl);
}

void ToyView::computeSubMeshFine(int id)
{
    if (id == -1)
        return;
    m_subMeshGLFine[id]->compute_element();
}

void ToyView::deleteSubMeshFine(int id)
{
    int n = static_cast<int>(m_subMeshGLFine.size());
    if (id != n - 1)
    {
        std::swap(m_subMeshGLFine[id], m_subMeshGLFine.back());
    }
    m_subMeshGLFine.pop_back();
}

void ToyView::draw_subMeshDeform(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLDeform.size(); ++i)
    {
        m_subMeshGLDeform[i]->draw_mesh(gl);
    }
}

void ToyView::updateSubMeshDeform(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGLDeform.size(); ++i)
    {
        m_subMeshGLDeform[i]->update(gl);
    }
}

void ToyView::deleteSubMeshDeform(int id)
{
    int n = static_cast<int>(m_subMeshGLDeform.size());
    if (id != n - 1)
    {
        std::swap(m_subMeshGLDeform[id], m_subMeshGLDeform.back());
    }
    m_subMeshGLDeform.pop_back();
}

void ToyView::paintAuto2D(EasyGL *gl)
{
    for (int i = 0; i < m_subMeshGLFine.size(); i++)
    {
        m_subMeshGLFine[i]->setTexture(gl, gl->bg_texture);
        m_subMeshGLFine[i]->compute_uv(gl);
        m_subMeshGLFine[i]->update(gl);
    }
}

void ToyView::pickBone(const QPoint &pixel, bool shift_down, bool ctrl_down)
{
    EasyGL *m_gl = qobject_cast<EasyGL *>(Three::currentViewer());
    // Bone *bone = m_toy->skel->pick_bone(pixel.x(), pixel.y(), m_gl->width(), m_gl->height(), m_gl->m_mvp.data(), shift_down, ctrl_down);
    Bone *bone = m_toy->pick_bone(pixel.x(), pixel.y(), m_gl->width(), m_gl->height(), m_gl->m_mvp.data(), shift_down, ctrl_down);
    if (ctrl_down && bone != nullptr)
    {
        m_selBones.push_back(bone);
    }
    if (m_skelGL)
    {
        EasyGL *gl = qobject_cast<EasyGL *>(Three::currentViewer());
        update_skel(gl);
    }
}

void ToyView::dragBone(const QPoint &pixel, bool right_click, bool shift_down, bool ctrl_down)
{
    EasyGL *gl = qobject_cast<EasyGL *>(Three::currentViewer());
    m_toy->drag_bone(pixel.x(), pixel.y(),
                     gl->width(), gl->height(),
                     gl->m_viewMatrix.data(), gl->m_mvp.data(),
                     right_click, shift_down, ctrl_down);

    if (m_skelGL)
    {
        update_skel(gl);
    }

    if (!m_toy->skel->get_editing())
    {
        if (m_meshGL3D != nullptr)
        {
            m_meshGL3D->compute_element();
        }
    }
}

void ToyView::releaseBone(EasyGL *gl)
{
    m_toy->skel->up();

    if (m_skelGL)
    {
        update_skel(gl);
    }
}

void ToyView::deleteBone()
{ // ToDO:
    m_toy->skel->delete_bone();
    if (m_skelGL)
    {
        EasyGL *gl = qobject_cast<EasyGL *>(Three::currentViewer());
        update_skel(gl);
    }

    if (m_restSkelGL)
    {
        EasyGL *gl = qobject_cast<EasyGL *>(Three::currentViewer());
        update_restSkel(gl);
    }
}

void ToyView::recoverRestPose(EasyGL *gl)
{
    m_toy->set_rest_pose();
    if (m_meshGL3D)
    {
        m_meshGL3D->compute_element();
    }
}

void ToyView::update_skel(EasyGL *gl)
{
    m_skelGL->compute_element(gl);
    m_skelGL->update(gl);
}

void ToyView::update_restSkel(EasyGL *gl)
{
    m_restSkelGL->compute_element(gl);
    m_restSkelGL->update(gl);
}

void ToyView::setBoneParent()
{
    if (m_selBones.size() != 2)
    {
        QMessageBox::information(nullptr, "",
                                 "There should be exactly two bones, the first one is child, the second one is parent!");
        for (Bone *b : m_selBones)
        {
            b->is_tip_selected = false;
            b->is_selected = false;
        }
        m_selBones.clear();
        return;
    }

    Bone *child = m_selBones[0];
    Bone *parent = m_selBones[1];
    if (child->parent != nullptr)
    {
        QMessageBox::information(nullptr, "",
                                 "Child already has a parent! Could not specify new one!");
        for (Bone *b : m_selBones)
        {
            b->is_tip_selected = false;
            b->is_selected = false;
        }
        m_selBones.clear();
        return;
    }
    Skeleton<Bone> *skel = m_toy->skel;
    skel->roots.erase(std::remove(skel->roots.begin(), skel->roots.end(), child));
    child->offset = child->rest_tip() - parent->rest_tip();
    child->set_parent(parent);
    child->set_wi(gather_bones(skel->roots).size() - skel->roots.size());
    m_selBones.clear();
}

void ToyView::splitBone()
{
    if (m_selBones.size() != 1)
    {
        QMessageBox::information(nullptr, "",
                                 "There should be exactly one bone!");
        for (Bone *b : m_selBones)
        {
            b->is_line_segment_selected = false;
        }
        m_selBones.clear();
        return;
    }
    Bone *sel_bone = m_selBones[0];
    if (!sel_bone->is_line_segment_selected)
    {
        QMessageBox::information(nullptr, "",
                                 "Line segment should be selected!");
        return;
    }
    sel_bone->split();
    m_selBones.clear();
}

void ToyView::mergeBone()
{
}

void ToyView::drawBone(EasyGL *gl)
{
    m_skelGL->draw(gl);
}

void ToyView::drawRestBone(EasyGL *gl)
{
    m_restSkelGL->draw(gl);
}

void ToyView::draw_mesh2D(EasyGL *gl)
{
    if (m_meshGL2D)
    {
        m_meshGL2D->draw_mesh(gl);
    }
}

void ToyView::update_mesh3D(EasyGL *gl)
{
    if (m_meshGL3D)
    {
        m_meshGL3D->compute_element();
        m_meshGL3D->update(gl);
    }
}

void ToyView::draw_mesh3D(EasyGL *gl)
{
    glEnable(GL_CULL_FACE); // https://blog.csdn.net/shuan9999/article/details/105453951
    if (m_meshGL3D)
    {
        m_meshGL3D->draw_mesh(gl);
    }
}

void ToyView::plus_wi(EasyGL *gl)
{
    if (m_selected_wi >= m_toy->OW.cols() - 1)
    {
        m_selected_wi = 0;
    }
    else
    {
        m_selected_wi += 1;
    }

    if (m_toy->OW.size() > 0)
    {
        m_meshGL3D->compute_skinning_colors(m_toy->OW, m_selected_wi);
        m_meshGL3D->update(gl);
    }
}

void ToyView::minus_wi(EasyGL *gl)
{
    if (m_selected_wi <= 0)
    {
        m_selected_wi = m_toy->OW.cols() - 1;
    }
    else
    {
        m_selected_wi -= 1;
    }

    if (m_toy->OW.size() > 0)
    {
        m_meshGL3D->compute_skinning_colors(m_toy->OW, m_selected_wi);
        m_meshGL3D->update(gl);
    }
}

void ToyView::set_selected_bone_dof_type()
{
    if (m_cur_dof_type >= static_cast<int>(DegreeOfFreedomType::NUM_DOF_TYPES))
    {
        m_cur_dof_type = 0;
    }
    else
    {
        m_cur_dof_type++;
    }

    vector<Bone *> sel = m_toy->skel->find_all_selected();
    for (vector<Bone *>::iterator bi = sel.begin(); bi != sel.end(); bi++)
    {
        // case current value to bool
        int dof_type = (*bi)->tip_dof_type + 1;
        (*bi)->tip_dof_type = (DegreeOfFreedomType)(dof_type);
    }
}

bool ToyView::compute_weightsTet(EasyGL *gl)
{
    // Get the whole mesh to be deformed
    bool flag = m_toy->compute_V3D();
    if (flag == false)
    {
        QMessageBox::warning(QApplication::activeWindow(),
                             "ERROR",
                             "Merge 3D mesh fails, please move the toy to reconnect!\n");
        return false;
    }

    // Automatically compute weights by BBW
    m_toy->compute_weightsTet();
    if (m_meshGL3D == nullptr)
    {
        m_meshGL3D = new MeshGL(&m_toy->m_mesh3D);
    }
    if (m_meshGL3D)
    {
        Q_DEBUG << " m_toy->m_mesh3D=" << m_toy->m_mesh3D.number_of_vertices() << " weights=" << m_toy->OW.rows();
        if (m_toy->m_isSubToyMoved)
        {
            m_meshGL3D->compute_element();
            m_toy->m_isSubToyMoved = false;
        }
        m_selected_wi = 0;
        m_meshGL3D->compute_skinning_colors(m_toy->OW, m_selected_wi);

        gl->m_meshMode = MESH_MODE_SKINNING_COLORED;
        m_meshGL3D->update(gl);
        return true;
    }
    else
    {
        return false;
    }
}

void ToyView::switch_to_deform_mode(EasyGL *gl)
{
    if (m_toy->OW.rows() == 0)
    {
        compute_weightsTet(gl);
        for (EasyGL *v : EasyGL::EasyGLPool())
        {
            update_mesh3D(v);
        }
    }
}

// This is another very important function!!!
void ToyView::addSubMesh3D(EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();
    subToy->InflateLaplaceFrom3DPlane(subToy->m_layerThick); // mesh
    subToy->Skeletonize(subToy->m_skelOpt);                  // skeleton
    m_toy->SetupParentChildRelation3D(subToy);               // decide which part this subToy should be connected to
    subToy->Translate(subToy->GetAxisDepthVec());
    m_toy->global_skel_optim();
    getSubMeshGL3D(subToy->GetID(), &subToy->m_curMesh3D);
}

MeshGL *ToyView::getSubMeshGL3D(int id, TMesh *mesh_ptr)
{
    int n = static_cast<int>(m_subMeshGL3D.size());
    if (id > n - 1)
    {
        MeshGL *meshgl = new MeshGL(mesh_ptr);
        m_subMeshGL3D.push_back(meshgl);
        return meshgl;
    }

    MeshGL *meshgl = m_subMeshGL3D[id];
    if (meshgl->mesh != mesh_ptr)
    {
        M_DEBUG << "subMeshGL3D" << id << ": old_mesh_ptr=" << meshgl->mesh << " new_mesh_ptr=" << mesh_ptr << endl;
        meshgl->set_mesh(mesh_ptr);
    }
    return meshgl;
}

void ToyView::deleteSubMesh3D(int id)
{
    int n = static_cast<int>(m_subMeshGL3D.size());
    if (id != n - 1)
    {
        std::swap(m_subMeshGL3D[id], m_subMeshGL3D.back());
    }
    m_subMeshGL3D.pop_back();
}

void ToyView::computeSubMesh3D(int id)
{
    if (id == -1)
        return;
    m_subMeshGL3D[id]->compute_element();
}

void ToyView::computeSubMesh3D()
{
    for (size_t i = 0; i < m_subMeshGL3D.size(); ++i)
    {
        m_subMeshGL3D[i]->compute_element();
    }
}

void ToyView::updateSubMesh3D(int id, EasyGL *gl)
{
    if (id == -1)
        return;
    m_subMeshGL3D[id]->update(gl);
}

void ToyView::updateSubMesh3D(EasyGL *gl)
{
    for (int i = 0; i < m_subMeshGL3D.size(); i++)
    {
        m_subMeshGL3D[i]->update(gl);
    }
}

void ToyView::drawSelectedSubMesh3D(EasyGL *gl)
{
    int i = m_toy->m_curSubToyIdx;
    if (i >= 0 && i < static_cast<int>(m_subMeshGL3D.size()))
        m_subMeshGL3D[i]->draw_mesh(gl);
}

void ToyView::draw_subMesh3D(EasyGL *gl)
{
    for (size_t i = 0; i < m_subMeshGL3D.size(); ++i)
    {
        m_subMeshGL3D[i]->draw_mesh(gl);
    }
}

bool ToyView::pickDefCtrlPts(const QPoint &point, EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();

    if (!subToy)
        return false;

    // ToDO: remove IsFrontView
    if (subToy->m_curMesh3D.number_of_vertices() == 0)
        subToy->m_vDefCptIdx = pickPtOnSketchSpline(point, subToy->m_sketchSpline, gl);
    else
        subToy->m_vDefCptIdx = pickPtOnSketchSpline(point, subToy->m_defSpline, gl);

    qDebug() << __FILE__ << " " << __LINE__ << ": pick subToy" << subToy->name().c_str() << ", spline ctrlPt" << subToy->m_vDefCptIdx;

    return subToy->m_vDefCptIdx >= 0;
}

int ToyView::pickPtOnSketchSpline(const QPoint &point, const CSpline &spline, EasyGL *gl)
{
    // point: mouse point on screen space
    Pnt2 cursor(point.x(), point.y());
    cout << __FILE__ << " " << __LINE__ << " pickPtOnSketchSpline cursor=" << cursor.transpose() << endl;
    for (int i = 0; i < spline.GetCtrlPointCount(); i++)
    {
        const Pnt3 &ctrlP = spline.GetCtrlPoint(i);
        Pnt2 p = gl->Cvt3Dto2D(ctrlP);
        if ((p - cursor).norm() < POINT_SIZE)
        {
            cout << __FILE__ << " " << __LINE__ << " pickCtrlPt=" << p.transpose() << endl;
            return i;
        }
    }
    return -1;
}

void ToyView::init_contour_deform(EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();
    if (!subToy)
        return;
    // capture contour image
    Vec3 viewDir;
    gl->GetViewDir(viewDir[0], viewDir[1], viewDir[2]);
    gl->capture_contour();
    gl->repaint();
    // extract contour line
    cv::Mat &img = gl->m_contourImg;
    vector<vector<double>> &depth = gl->m_depthImg;
    vector<vector<cv::Point>> contours;
    ContourExtractor::detect_contour(img, contours);
    assert(contours.size() >= 1 && "Contour is not captured, cannot perform shape deformation!");
    vector<Pnt3> cont;
    double avgDepth = 0;
    for (const cv::Point &p : contours[0])
    {
        avgDepth += depth[p.y][p.x];
    }
    avgDepth /= contours[0].size();
    for (const cv::Point &p : contours[0])
    {
        cont.push_back(gl->Cvt2Dto3D(p.x, p.y, avgDepth));
        // cout << __FILE__ << " " << __LINE__ << " p=" << p
        //      << " avgDepth=" << avgDepth
        //      << " depth=" << depth[p.y][p.x] << " 3d=" << cont.back().transpose() << endl;
    }

    CSpline &spline = subToy->m_defSpline;
    vector<vector<Pnt3>> &sketchPolyLines = subToy->m_defPolyLines;
    vector<Pnt3> &sketchPolyLine = subToy->m_defPolyLine;

    spline.Clear();
    spline.ChangeMode(CSpline::SPLMODE_CLOSED_SPLINE);
    sketchPolyLines.clear();
    sketchPolyLine.clear();

    sketchline2Spline(cont, spline, gl);
    CSketchManager::ConvertSplineToPolyLines(spline, sketchPolyLines);
    CSketchManager::GetPolyLine(sketchPolyLines, sketchPolyLine);
    subToy->InitContourDeform(viewDir);

    // visualization
    computeDefContour();
    updateDefContour(gl);
}

void ToyView::modifySubContourOnMouseMove(const QPoint &point, EasyGL *gl)
{
    XSubToy *subToy = m_toy->GetCurToy();
    if (!subToy)
        return;

    int pickPtIdx = subToy->m_vDefCptIdx;
    if (pickPtIdx < 0)
        return;

    Vec3 viewDir;
    gl->GetViewDir(viewDir[0], viewDir[1], viewDir[2]);
    auto newP = gl->Cvt2Dto3D(point.x(), point.y());
    if (subToy->m_curMesh3D.number_of_vertices() == 0) // the newly created contour line, the meshes has not be created yet
    {
        vector<Pnt3> &ctrlPts = subToy->m_sketchSpline.P;
        ctrlPts[pickPtIdx] = Pnt3(newP.x(), newP.y(), newP.z());
        // update contour line
        vector<vector<Pnt3>> &sketchPolyLines = subToy->m_sketchPolyLines;
        vector<Pnt3> &sketchPolyLine = subToy->m_sketchPolyLine;
        CSketchManager::ConvertSplineToPolyLines(subToy->m_sketchSpline, sketchPolyLines);
        CSketchManager::GetPolyLine(sketchPolyLines, sketchPolyLine);
        subToy->SetCentroid();
        subToy->SetRotToXY();
        subToy->SetSketchPolyLineOnXY();

        // visualization
        // update control points
        int id = subToy->GetID();
        const GLVec3 &origP = subCtrlPts[id][pickPtIdx];
        subCtrlPts[id][pickPtIdx] = GLVec3(newP.x(), newP.y(), newP.z());

        std::vector<GLVec3> &gl_spline = subContours[id];
        gl_spline.clear();
        for (const Pnt3 &p : sketchPolyLine)
        {
            gl_spline.push_back(GLVec3(p.x(), p.y(), p.z()));
        }
    }
    else
    {
        vector<Pnt3> &ctrlPts = subToy->m_defSpline.P;
        ctrlPts[pickPtIdx] = Pnt3(newP.x(), newP.y(), newP.z());
        vector<vector<Pnt3>> &sketchPolyLines = subToy->m_defPolyLines;
        vector<Pnt3> &sketchPolyLine = subToy->m_defPolyLine;
        CSketchManager::ConvertSplineToPolyLines(subToy->m_defSpline, sketchPolyLines);
        CSketchManager::GetPolyLine(sketchPolyLines, sketchPolyLine);
        computeDefContour();

        subToy->DeformContour(viewDir);
        computeSubMesh3D();

        if (gl->m_viewMode != VIEW_PERSP)
        {
            subToy->InitContour(subToy->m_defSpline.P);
        }
    }
}

void ToyView::computeDefContour()
{
    XSubToy *subToy = m_toy->GetCurToy();
    if (!subToy)
        return;
    defContour.clear();
    for (const Pnt3 &p : subToy->m_defPolyLine)
    {
        defContour.push_back(GLVec3(p.x(), p.y(), p.z()));
    }

    CSpline &spline = subToy->m_defSpline;
    defCtrlPts.clear();
    for (int i = 0; i < spline.GetCtrlPointCount(); i++)
    {
        const Pnt3 &p = spline.GetCtrlPoint(i);
        defCtrlPts.push_back(GLVec3(p.x(), p.y(), p.z()));
    }
}

void ToyView::updateDefContour(EasyGL *gl)
{
    defContour_gl.update(gl);
    defCtrlPts_gl.update(gl);
}

void ToyView::drawDefContour(EasyGL *gl)
{
    if (defContour.size() <= 3)
        return;
    // !!! must add this line, because
    defContour_gl.setEdgePtr(&defContour);
    defContour_gl.draw(gl);
    float point_size = POINT_SIZE * gl->GetNDCDisOfOnePixelOn2D();
    defCtrlPts_gl.setPointSize(point_size);
    defCtrlPts_gl.setPointPtr(&defCtrlPts);
    defCtrlPts_gl.draw(gl);
}

void ToyView::addMeshes(EasyGL *gl)
{
    if (gl->m_viewMode != VIEW_FRONT && gl->m_curLayer != 0 && m_toy->GetCurToy()->m_curMesh3D.number_of_vertices() == 0)
    {
        QMessageBox::warning(QApplication::activeWindow(),
                             "ERROR",
                             "Layered draw function is only enabled under front view\n");
        delete_subContour();
        return;
    }

    addSubMeshCoarse(gl);
    addSubMeshFine(gl);
    addSubMesh3D(gl);

    XSubToy *subToy = m_toy->GetCurToy();
    int id = subToy->GetID();
    computeSubContour(id);
    m_subMeshGLCoarse[id]->compute_element();
    m_subMeshGLFine[id]->compute_element();
    m_subMeshGL3D[id]->compute_element();

    pick_current_subtoy();
    updateSubToyGL();
}

void ToyView::deleteSubToy()
{
    XSubToy *subToy = m_toy->GetCurToy();
    if (subToy)
    {
        int id = subToy->GetID();
        deleteSubContour(id);
        deleteSubMeshCoarse(id);
        deleteSubMeshFine(id);
        deleteSubMesh3D(id);
        m_toy->DelSubToy(subToy);

        updateSubToyGL();
    }
}

void ToyView::updateSubToyGL()
{
    for (auto v : EasyGL::EasyGLPool())
    {
        update_subContours(v);
        updateSubMeshCoarse(v);
        updateSubMeshFine(v);
        updateSubMesh3D(v);
        update_skel(v);
        update_restSkel(v);
        v->repaint();
    }
}

void ToyView::TexturePaintOnLBDown(QPoint point, EasyGL *gl)
{
    bool picked = m_toy->pick_subtoy(point.x(), point.y(), gl->width(), gl->height(), gl->m_mvp.data());
    if (picked)
    {
        XSubToy *subToy = m_toy->GetCurToy();
        TMesh &mesh = subToy->m_curMesh3D;
        FreehandLine &texPaintLine = mesh.m_texPaint.m_texPaintLine;
        int fid = m_toy->m_pickFid;
        Pnt3 barycoord = m_toy->m_pickFbc;
        Pnt3 baseP = m_toy->m_pickFPnt;

        texPaintLine.Clear();
        texPaintLine.fid.push_back(fid);
        texPaintLine.Pos3d.push_back(baseP);
        texPaintLine.eid.push_back(-1);
        texPaintLine.uvw.push_back(barycoord);

        TexturePaintDraw();
    }
}

void ToyView::TexturePaintDraw()
{
    QColor color(SUBMESH_COLOR);
    ColorI paintColor((BYTE)color.red(), (BYTE)color.green(), (BYTE)color.blue());
    m_toy->TexturePaintDraw(paintColor);
}

void ToyView::TextureImageOnMouseMove(QPoint point, EasyGL *gl)
{
}

void ToyView::TextureImageOnLBUp(QPoint point, EasyGL *gl)
{
    // extract region from background picture
    // CSketchImagePaintDlg
    //
}

void ToyView::SelTexImageBy2DSplines(EasyGL *gl)
{
    // CSketchImagePaintDlg: onOK
    // PolygonImage
}

void ToyView::TextureImageOnDbClk(QPoint point, EasyGL *gl)
{
    // PolygonImage
    // get map cpts and vfids
    // mesh.SurfaceTrim
    // map to bg picture and get uv of verts
    // copy image to texture
}

void ToyView::skeletonize(XSubToy *subToy)
{
    // cout << __FILE__ << " " << __LINE__ << " partition...." << endl;
    // std::vector<Pnt3> &sketchLine = subToy->m_sketchPolyLine;
    // double line_len = CalLineLength(sketchLine);
    // std::vector<Pnt3> simp_polyline;
    // SimplifyPolyLine(sketchLine, simp_polyline, 10);
    // if ((simp_polyline.back() - simp_polyline.front()).norm() < 0.1)
    // {
    //     simp_polyline.pop_back();
    // }
    // egl::writeLine("output/poly.txt", simp_polyline);
    // cout << __FILE__ << " " << __LINE__ << " simp_polyline=" << simp_polyline.size() << " line_len=" << line_len << endl;
    // std::vector<std::vector<Pnt3>> part_polys;
    // igl::copyleft::cgal::partition(simp_polyline, part_polys);
    // cout << __FILE__ << " " << __LINE__ << " part polys=" << part_polys.size() << endl;

    cout << __FILE__ << " " << __LINE__ << " skeletonize...." << endl;
    //----------------------------Convert Polygon to Binary Image----------------------

    BBox2 bbox;
    XSubToy *parent = subToy->m_parent;
    if (parent)
    {
        for (const Pnt3 &p : parent->m_sketchPolyLine)
        {
            if (p.x() < bbox.xmin)
            {
                bbox.xmin = p.x();
            }
            if (p.y() < bbox.ymin)
            {
                bbox.ymin = p.y();
            }
            if (p.x() > bbox.xmax)
            {
                bbox.xmax = p.x();
            }
            if (p.y() > bbox.ymax)
            {
                bbox.ymax = p.y();
            }
        }

        for (XSubToy *child : parent->m_children)
            for (const Pnt3 &p : child->m_sketchPolyLine)
            {
                if (p.x() < bbox.xmin)
                {
                    bbox.xmin = p.x();
                }
                if (p.y() < bbox.ymin)
                {
                    bbox.ymin = p.y();
                }
                if (p.x() > bbox.xmax)
                {
                    bbox.xmax = p.x();
                }
                if (p.y() > bbox.ymax)
                {
                    bbox.ymax = p.y();
                }
            }
    }
    else
    {
        for (const Pnt3 &p : subToy->m_sketchPolyLine)
        {
            if (p.x() < bbox.xmin)
            {
                bbox.xmin = p.x();
            }
            if (p.y() < bbox.ymin)
            {
                bbox.ymin = p.y();
            }
            if (p.x() > bbox.xmax)
            {
                bbox.xmax = p.x();
            }
            if (p.y() > bbox.ymax)
            {
                bbox.ymax = p.y();
            }
        }
    }

    double xmin = bbox.xmin + 0.5;
    double ymin = bbox.ymin + 0.5;
    double xmax = bbox.xmax + 0.5;
    double ymax = bbox.ymax + 0.5;
    double padding_x = (xmax - xmin) / 4.0; // 0.25
    double padding_y = (ymax - ymin) / 4.0;

    int W = (xmax - xmin) + 2 * padding_x;
    int H = (ymax - ymin) + 2 * padding_y;

    cv::Mat img = cv::Mat::zeros(H, W, CV_8UC1);
    if (parent)
    {
        std::vector<std::vector<cv::Point>> contours;
        vector<cv::Point> cont;
        for (const Pnt3 &p : parent->m_sketchPolyLine)
        {
            int x = int(p.x() + 0.5);
            int y = int(p.y() + 0.5);
            cont.emplace_back(x - xmin + padding_x, y - ymin + padding_y);
        }
        contours.push_back(cont);
        cv::drawContours(img, contours, -1, cv::Scalar(1), -1);

        cont.clear();
        contours.clear();
        for (XSubToy *child : parent->m_children)
        {
            for (const Pnt3 &p : child->m_sketchPolyLine)
            {
                int x = int(p.x() + 0.5);
                int y = int(p.y() + 0.5);
                cont.emplace_back(x - xmin + padding_x, y - ymin + padding_y);
            }
            contours.push_back(cont);
            cv::drawContours(img, contours, -1, cv::Scalar(1), -1);
        }
    }
    else
    {
        std::vector<std::vector<cv::Point>> contours;
        vector<cv::Point> cont;
        for (const Pnt3 &p : subToy->m_sketchPolyLine)
        {
            int x = int(p.x() + 0.5);
            int y = int(p.y() + 0.5);
            cont.emplace_back(x - xmin + padding_x, y - ymin + padding_y);
        }
        contours.push_back(cont);
        cv::drawContours(img, contours, -1, cv::Scalar(1), -1);
    }

    std::vector<std::vector<cv::Point>> contours;

    cout << __FILE__ << " " << __LINE__ << " img size W=" << W << " H=" << H << endl;
    // cv::Mat flip_img;
    // cv::flip(img, flip_img, 1);
    cv::imwrite("output/contour.jpg", img * 255);

    contours.clear();
    ContourExtractor::detect_contour(img, contours);
    cout << __FILE__ << " " << __LINE__ << " partition...." << endl;
    std::vector<Pnt3> sketchLine;
    for (const cv::Point &p : contours[0])
    {
        sketchLine.push_back(Pnt3(p.x, p.y, 0));
    }
    double line_len = CalLineLength(sketchLine);
    std::vector<Pnt3> simp_polyline;
    SimplifyPolyLine(sketchLine, simp_polyline, 3);
    if ((simp_polyline.back() - simp_polyline.front()).norm() < 0.1)
    {
        simp_polyline.pop_back();
    }
    if (IsClockWise(simp_polyline))
    {
        std::reverse(simp_polyline.begin(), simp_polyline.end());
    }
    egl::writeLine("output/poly.txt", simp_polyline);
    cout << __FILE__ << " " << __LINE__ << " simp_polyline=" << simp_polyline.size() << " line_len=" << line_len << endl;
    std::vector<std::vector<Pnt3>> part_polys;
    igl::copyleft::cgal::partition(simp_polyline, part_polys);
    cout << __FILE__ << " " << __LINE__ << " part polys=" << part_polys.size() << endl;

    cv::Mat img_tmp = img.clone();
    Skeletor skeletor(img_tmp.data, img_tmp.cols, img_tmp.rows);
    skeletor.thinning_zs();
    cv::imwrite("output/thin.jpg", img_tmp * 255);
    skeletor.get_branches();
}

void ToyView::push_keyframe()
{
    for (int i = 0; i < m_toy->m_subToys.size(); i++)
    {
        XSubToy *subToy = m_toy->m_subToys[i];
    }
}

void ToyView::pop_keyframe()
{
}

#ifdef ToBEDel
MeshGL::MeshGL(TMesh &mesh_, ToyView *toyview_, XToy *toy_, XSubToy *subToy_)
    : mesh(mesh_),
      toyview(toyview_),
      toy(toy_),
      subToy(subToy_)
{
    mesh_color = toyview_->SUBMESH_COLOR; //
    compute_element();
    compute_color();
    tc.vertices = &vertices;
    tc.idx_data = &idx_data;
    tc.v_normals = &v_normals;
    tc.v_colors = &v_colors;
    tc.v_skinning_colors = &v_skinning_colors;
    tc.v_texCoords = &v_texCoords;
}
#endif

MeshGL::MeshGL(TMesh *mesh_ptr)
    : mesh_color(SUBMESH_COLOR)
{
    set_mesh(mesh_ptr);
}

void MeshGL::removeViewer(EasyGL *gl)
{
    tc.removeViewer(gl);
}

void MeshGL::update(EasyGL *gl)
{
    tc.update(gl);
}

void MeshGL::set_mesh(TMesh *mesh_ptr)
{
    mesh = mesh_ptr;
    compute_element();
    compute_color();
    tc.vertices = &vertices;
    tc.idx_data = &idx_data;
    tc.v_normals = &v_normals;
    tc.v_colors = &v_colors;
    tc.v_skinning_colors = &v_skinning_colors;
    tc.v_texCoords = &v_texCoords;
}

void MeshGL::draw_mesh(EasyGL *gl)
{
    tc.draw(gl);
}

void MeshGL::compute_element()
{
    GetVData();
    GetFData();
    GetVNormal();
}

void MeshGL::compute_color()
{
    GetVColor();
}

void MeshGL::compute_uv(EasyGL *gl)
{
    if (v_texCoords.size() > 0) //ToDO: now only works when sketchline is totally overlapped with displayed image
        return;
    Q_DEBUG << "compute_uv";
    // ToDO: Notice: only works in front view now!!!
    const GLVec3 &v0 = gl->getBgVert(0);
    const GLVec3 &v3 = gl->getBgVert(3);
    Pnt3 p0(v0.x, v0.y, v0.z), p3(v3.x, v3.y, v3.z);
    const Pnt2 &lb_p = gl->Cvt3Dto2D(p0); //left bottom point
    Vec2 wh = gl->Cvt3Dto2D(p3) - gl->Cvt3Dto2D(p0);
    // v_colors.clear();

    for (Vd vd : mesh->vertices())
    {
        const Pnt3 &p = mesh->point(vd);
        Pnt2 p_screen = gl->Cvt3Dto2D(p);
        Vec2 cur_lb = p_screen - lb_p;
        double x_tex = cur_lb.x() / wh.x();
        double y_tex = cur_lb.y() / wh.y();
        v_texCoords.push_back(GLVec2(x_tex, y_tex));
        // QColor color = viewer->m_bgImg.pixelColor(x_tex * viewer->m_bgImg.width(), (1 - y_tex) * viewer->m_bgImg.height());
        // v_colors.push_back(GLVec3(color.redF(), color.greenF(), color.blueF()));
        // qDebug() << vd.idx() << ": " << p_screen.x() << "," << p_screen.y()
        //          << " cur_lb=" << cur_lb.x() << ", " << cur_lb.y()
        //          << " tex=" << x_tex << ", " << y_tex
        //          << " color=" << v_colors[vd.idx()].x << ", " << v_colors[vd.idx()].y << ", " << v_colors[vd.idx()].z;
    }
}

void MeshGL::compute_skinning_colors(const Eigen::MatrixXd &W, int wi /*weight index*/)
{
    //ToDO: change to GPU
    if (W.rows() <= 0)
    {
        return;
    }
    assert(W.rows() == mesh->number_of_vertices());
    v_skinning_colors.clear();
    for (int i = 0; i < W.rows(); i++)
    {
        v_skinning_colors.push_back(heat_color(W(i, wi)));
    }
    qDebug() << __FILE__ << " " << __LINE__ << " v_skinning_colors=" << v_skinning_colors.size();
}

void MeshGL::GetVData()
{
    vertices.clear();
    for (Vd vd : mesh->vertices())
    {
        const Pnt3 &p = mesh->point(vd);
        vertices.push_back(GLVec3(p.x(), p.y(), p.z())); //ToDO:
    }
}

void MeshGL::GetFData()
{
    idx_data.clear();
    for (Fd fd : mesh->faces())
    {
        Hd hd = mesh->halfedge(fd);
        Hd nxt_hd = mesh->next(hd);
        unsigned int v0 = mesh->source(hd).idx();
        unsigned int v1 = mesh->target(hd).idx();
        unsigned int v2 = mesh->target(nxt_hd).idx();
        idx_data.push_back(v0);
        idx_data.push_back(v1);
        idx_data.push_back(v2);
    }
}

void MeshGL::GetVNormal()
{
    v_normals.clear();
    mesh->ComputeVNormals();

    for (Vd vd : mesh->vertices())
    {
        const Vec3 &normal = mesh->GetVNormal(vd);
        v_normals.push_back(GLVec3(normal.x(), normal.y(), normal.z()));
    }
}

void MeshGL::GetVColor()
{
    v_colors.clear();
    for (Vd vd : mesh->vertices())
    {
        v_colors.push_back(GLVec3(mesh_color.redF(), mesh_color.greenF(), mesh_color.blueF()));
    }
}

// QColor SkelGL::bone_color = QColor(106., 106., 255.);
QColor SkelGL::bone_color = QColor(255, 0, 0);

SkelGL::SkelGL(Skeleton<Bone> *skel_) : skel(skel_)
{
    line_container.setDrawLines(true);
}

void SkelGL::removeViewer(EasyGL *gl)
{
    point_container.removeViewer(gl);
    line_container.removeViewer(gl);
    dragline_container[0].removeViewer(gl);
    dragline_container[1].removeViewer(gl);
}

void SkelGL::update(EasyGL *gl)
{
    float point_size = Bone::BONE_POINT_RADIUS * gl->GetNDCDisOfOnePixelOn2D();
    point_container.setPointSize(point_size);
    line_container.setColor(bone_color);
    line_container.setEdgeWidth(static_cast<float>(Bone::BONE_DIRECTED_LINE_SEGMENT_WIDTH));

    point_container.update(gl);
    line_container.update(gl);
    dragline_container[0].update(gl);
    dragline_container[1].update(gl);
}

void SkelGL::draw(EasyGL *gl)
{
    if (tips.size() < 0)
        return;

    point_container.setPointPtr(&tips);
    line_container.setEdgePtr(&lines);
    dragline_container[0].setEdgePtr(&drag_line[0]);
    dragline_container[1].setEdgePtr(&drag_line[1]);

    point_container.draw(gl);
    line_container.draw(gl);
    dragline_container[0].draw(gl);
    dragline_container[1].draw(gl);
}

void SkelGL::compute_element(EasyGL *gl)
{
    vector<Bone *> bones = gather_bones(skel->roots);
    vector<int> selected;
    vector<GLVec3> colors;
    tips.clear();
    lines.clear();

    int i = 0;
    for (Bone *b : bones)
    {
        if (b->is_root())
        {
            if (b->isAttachBone)
            {
                if (b->isAttachBoneActivated)
                {
                    const Pnt3 &tip = b->rest_tip();
                    tips.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                }
            }
            else
            {
                if (gl->m_opMode == OPMODE_BBW_DEFORM)
                {
                    const Pnt3 &tip = b->tip_as_drawn();
                    tips.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                }
                else
                {
                    const Pnt3 &tip = b->rest_tip();
                    tips.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                }
            }

            double pcolor[3] = {1, 1, 0};
            // b->tip_color(pcolor);
            if (!b->isAttachBone || (b->isAttachBone && b->isAttachBoneActivated))
                colors.push_back(GLVec3(pcolor[0] * 0.5, pcolor[1] * 0.5, pcolor[2] * 0.5));
        }

        if (b->get_wi() >= 0)
        {
            if (gl->m_opMode == OPMODE_BBW_DEFORM)
            {
                const Pnt3 &tail = b->tail_as_drawn();
                const Pnt3 &tip = b->tip_as_drawn();
                tips.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                lines.push_back(GLVec3(tail.x(), tail.y(), tail.z() + 0.2));
                lines.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
            }
            else
            {
                const Pnt3 &tail = b->rest_tail();
                const Pnt3 &tip = b->rest_tip();
                if (!b->isAttachBone || (b->isAttachBone && b->isAttachBoneActivated))
                {
                    tips.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                    lines.push_back(GLVec3(tail.x(), tail.y(), tail.z() + 0.2));
                    lines.push_back(GLVec3(tip.x(), tip.y(), tip.z() + 0.2));
                }
            }

            // get foreground color for points
            double pcolor[3];
            b->tip_color(pcolor);
            colors.push_back(GLVec3(pcolor[0], pcolor[1], pcolor[2]));
        }

        if (b->is_tip_selected)
        {
            selected.push_back(i);
        }

        if (b->dragging_rotation)
        {
            compute_drag_line(gl, b);
        }
        else
        {
            drag_line[0].clear();
            drag_line[1].clear();
        }
        i++;
    }

    point_container.setSelectedIdxs(selected);
    point_container.setPointColors(colors);
}

void SkelGL::compute_drag_line(EasyGL *gl, Bone *bone)
{
    using namespace Eigen;
    // Draw a line from mouse down to mouse last
    Vector3d d = bone->tip_as_drawn();

    Vector3d s_d;
    EGL::project(d[0], d[1], d[2],
                 gl->width(), gl->height(), gl->m_mvp.data(),
                 s_d[0], s_d[1], s_d[2]);

    Vector3d last;
    EGL::unproject(bone->last_x, bone->last_y, s_d[2],
                   gl->width(), gl->height(), gl->m_mvp.data(),
                   last[0], last[1], last[2]);

    Vector3d down;
    EGL::unproject(bone->down_x, bone->down_y, s_d[2],
                   gl->width(), gl->height(), gl->m_mvp.data(),
                   down[0], down[1], down[2]);

    Vector3d last_x_only;
    EGL::unproject(bone->last_x, bone->down_y, s_d[2],
                   gl->width(), gl->height(), gl->m_mvp.data(),
                   last_x_only[0], last_x_only[1], last_x_only[2]);

    // std::vector<GLVec3> line[2];
    float fDis = 3 * gl->Get3dDisOfOnePixelOn2D();
    drag_line[0].clear();
    compute_stipple_line(down, last, fDis, drag_line[0]);
    compute_stipple_line(last, last_x_only, fDis, drag_line[0]);

    drag_line[1].clear();
    drag_line[1].emplace_back(down[0], down[1], down[2]);
    drag_line[1].emplace_back(last_x_only[0], last_x_only[1], last_x_only[2]);

    QColor guide_color(Bone::DRAG_LINE_GUIDE_COLOR[0],
                       Bone::DRAG_LINE_GUIDE_COLOR[1],
                       Bone::DRAG_LINE_GUIDE_COLOR[2], 255.);

    QColor drag_color(Bone::DRAG_LINE_COLOR[0],
                      Bone::DRAG_LINE_COLOR[1],
                      Bone::DRAG_LINE_COLOR[2], 255.);

    dragline_container[0].setColor(guide_color);
    dragline_container[0].setEdgeWidth(2.0);
    dragline_container[0].setDrawLines(true);
    dragline_container[1].setColor(drag_color);
    dragline_container[1].setEdgeWidth(3.0);
}

void ArcBallGL::removeViewer(EasyGL *gl)
{
    point_container.removeViewer(gl);
}

void ArcBallGL::update(EasyGL *gl)
{
    point_container.setPointPtr(&points);
    dragline_container.setEdgePtr(&drag_line);

    float point_size = 10 * gl->GetNDCDisOfOnePixelOn2D();
    point_container.setPointSize(point_size);
    point_container.setColor(QColor(255, 0, 0));

    dragline_container.setColor(QColor(255, 128, 0));
    dragline_container.setEdgeWidth(1.0);
    dragline_container.setDrawLines(true);

    point_container.update(gl);
    dragline_container.update(gl);
}

void ArcBallGL::draw(EasyGL *gl)
{
    point_container.draw(gl);
    dragline_container.draw(gl);
}

void ArcBallGL::set_centroid(const Pnt3 &p)
{
    centroid_ = p;
    points.clear();
    points.push_back(GLVec3(p.x(), p.y(), p.z()));
}

void ArcBallGL::set_cur_point(const QPoint &qp, EasyGL *gl)
{
    set_cur_point(gl->Cvt2Dto3D(qp.x(), qp.y(), centroid()[2]), gl);
}

void ArcBallGL::set_cur_point(const Pnt3 &cur_point, EasyGL *gl)
{
    cur_point_ = cur_point;

    float fDis = gl->Get3dDisOfOnePixelOn2D();
    drag_line.clear();
    compute_stipple_line(centroid(), cur_point, fDis, drag_line);
}

void PlaneGL::compute_element()
{
    idx_data.clear();
    vertices.clear();
    vnormals.clear();
    vcolors.clear();
    line.clear();

    idx_data.resize(2 * 3);
    vertices.resize(4);
    vnormals.resize(4);
    vcolors.resize(4);

    double half_len = edge_length * 0.5;
    Vec3 nor = plane.normal();
    Vec3 up;
    int min_idx;
    nor.minCoeff(&min_idx);
    if (min_idx == COORD_AXIS_X)
    {
        up = Vec3(0, -nor.z(), nor.y());
    }
    else if (min_idx == COORD_AXIS_Y)
    {
        up = Vec3(-nor.z(), 0, nor.x());
    }
    else if (min_idx == COORD_AXIS_Z)
    {
        up = Vec3(-nor.y(), nor.x(), 0);
    }
    Vec3 side = nor.cross(up);
    up = nor.cross(side);

    // M_DEBUG << "nor=" << nor.transpose() << " side=" << side.transpose() << " up=" << up.transpose() << endl;
    nor.normalize();
    up.normalize();
    side.normalize();

    Pnt3 cent;
    IntersectPlane(plane, Ray(Pnt3(0, 0, 0), nor), cent);
    // M_DEBUG << "plane_cent=" << cent.transpose() << endl;
    Pnt3 p0 = cent - side * half_len + up * half_len; // (-1, 1)
    Pnt3 p1 = cent - side * half_len - up * half_len; // (-1, -1)
    Pnt3 p2 = cent + side * half_len - up * half_len; // (-1, 1)
    Pnt3 p3 = cent + side * half_len + up * half_len; // (1, 1)

    vertices[0] = GLVec3(p0.x(), p0.y(), p0.z());
    vertices[1] = GLVec3(p1.x(), p1.y(), p1.z());
    vertices[2] = GLVec3(p2.x(), p2.y(), p2.z());
    vertices[3] = GLVec3(p3.x(), p3.y(), p3.z());

    for (int i = 0; i < 4; ++i)
    {
        vnormals[i] = GLVec3(nor.x(), nor.y(), nor.z());
        vcolors[i] = GLVec3(222. / 255., 222. / 255., 222. / 255.);
    }

    idx_data[0] = 0;
    idx_data[1] = 1;
    idx_data[2] = 2;
    idx_data[3] = 0;
    idx_data[4] = 2;
    idx_data[5] = 3;

    for (int i = 0; i < 4; ++i)
    {
        line.push_back(vertices[i]);
    }
    line.push_back(vertices[0]);
}

void PlaneGL::update(EasyGL *gl)
{
    tc.vertices = &vertices;
    tc.idx_data = &idx_data;
    tc.v_normals = &vnormals;
    tc.v_colors = &vcolors;

    ec.setEdgePtr(&line);
    ec.setColor(QColor(0, 0, 0));

    tc.update(gl);
    ec.update(gl);
}

void PlaneGL::draw(EasyGL *gl)
{
    tc.draw(gl);
    ec.draw(gl);
}

void AxisGL::compute_element(EasyGL *gl)
{
    calArrow(0.06, 0.12, 10, GLVec3(0, 0, 0), GLVec3(axis_length, 0, 0), axis_x_color, coord_data);
    calArrow(0.06, 0.12, 10, GLVec3(0, 0, 0), GLVec3(0, axis_length, 0), axis_y_color, coord_data);
    calArrow(0.06, 0.12, 10, GLVec3(0, 0, 0), GLVec3(0, 0, axis_length), axis_z_color, coord_data);
}

//  Draws a 3D arrow between the 3D point \p from and the 3D point \p to.
// \p data is filled with the three components of a point, then its normal, and then its color, which makes it filled like this:
// [P1.x-P1.y-P1.z-N1.x-N1.y-N1.z-C1.r-C1.g-C1.b|P2.x-P2.y-P2.z-N2.x-N2.y-N2.z-C2.r-C2.g-C2.b|...]
void AxisGL::calArrow(double r, double R, int prec, // cylinder radius, cone radius, precision
                      GLVec3 from, GLVec3 to,
                      GLVec3 color,
                      std::vector<float> &data)
{
    using std::acos;
    using std::cos;
    using std::sin;
    GLVec3 temp = to - from;
    QVector3D dir = QVector3D(float(temp.x), float(temp.y), float(temp.z));
    QMatrix4x4 mat;
    mat.setToIdentity();
    mat.translate(float(from.x), float(from.y), float(from.z));
    mat.scale(dir.length());
    dir.normalize();
    float angle = 0.0;
    if (std::sqrt((dir.x() * dir.x() + dir.y() * dir.y())) > 1)
        angle = 90.0f;
    else
        angle = float(acos(dir.y() / std::sqrt(dir.lengthSquared())) * 180.0 / M_PI);

    QVector3D axis;
    axis = QVector3D(dir.z(), 0, -dir.x());
    mat.rotate(angle, axis);

    //Head
    const float Rf = static_cast<float>(R);
    for (int d = 0; d < 360; d += 360 / prec)
    {
        float D = (float)(d * M_PI / 180.);
        float a = (float)std::atan(Rf / 0.33);
        QVector4D p(0., 1., 0, 1.);
        QVector4D n(Rf * sin(D), sin(a), Rf * cos(D), 1.);
        QVector4D pR = mat * p;
        QVector4D nR = mat * n;

        //point A1
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back((float)color.x);
        data.push_back((float)color.y);
        data.push_back((float)color.z);

        //point B1
        p = QVector4D(Rf * sin(D), 0.66f, Rf * cos(D), 1.f);
        n = QVector4D(sin(D), sin(a), cos(D), 1.);
        pR = mat * p;
        nR = mat * n;
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back((float)color.x);
        data.push_back((float)color.y);
        data.push_back((float)color.z);
        //point C1
        D = float((d + 360 / prec) * M_PI / 180.0);
        p = QVector4D(Rf * sin(D), 0.66f, Rf * cos(D), 1.f);
        n = QVector4D(sin(D), sin(a), cos(D), 1.0);
        pR = mat * p;
        nR = mat * n;

        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back((float)color.x);
        data.push_back((float)color.y);
        data.push_back((float)color.z);
    }

    //cylinder
    //body of the cylinder
    const float rf = static_cast<float>(r);
    for (int d = 0; d < 360; d += 360 / prec)
    {
        //point A1
        float D = float(d * M_PI / 180.0);
        QVector4D p(rf * sin(D), 0.66f, rf * cos(D), 1.f);
        QVector4D n(sin(D), 0.f, cos(D), 1.f);
        QVector4D pR = mat * p;
        QVector4D nR = mat * n;

        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
        //point B1
        p = QVector4D(rf * sin(D), 0, rf * cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat * p;
        nR = mat * n;

        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
        //point C1
        D = float((d + 360 / prec) * M_PI / 180.0);
        p = QVector4D(rf * sin(D), 0, rf * cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat * p;
        nR = mat * n;
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
        //point A2
        D = float((d + 360 / prec) * M_PI / 180.0);

        p = QVector4D(rf * sin(D), 0, rf * cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat * p;
        nR = mat * n;
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
        //point B2
        p = QVector4D(rf * sin(D), 0.66f, rf * cos(D), 1.f);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat * p;
        nR = mat * n;
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
        //point C2
        D = float(d * M_PI / 180.0);
        p = QVector4D(rf * sin(D), 0.66f, rf * cos(D), 1.f);
        n = QVector4D(sin(D), 0.f, cos(D), 1.f);
        pR = mat * p;
        nR = mat * n;
        data.push_back(pR.x());
        data.push_back(pR.y());
        data.push_back(pR.z());
        data.push_back(nR.x());
        data.push_back(nR.y());
        data.push_back(nR.z());
        data.push_back(float(color.x));
        data.push_back(float(color.y));
        data.push_back(float(color.z));
    }
}