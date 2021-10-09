#ifndef TOYVIEW_H
#define TOYVIEW_H

#include <vector>
#include <Eigen/Eigen>

#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <QMatrix4x4>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>

#include <core/XToy.h>
#include <core/SymmetrySketch.h>

#include "GLData.h"
#include "Viewer_interface.h"
#include "containers.h"
#include "common.h"

class EasyGL;
class MeshGL;
class PlaneGL;
class SkelGL;
class ArcBallGL;

const QColor CTRL_PT_COLOR(102, 51, 0, 255);
const QColor SUBMESH_COLOR(222, 222, 222);
const QColor PARENT_MESH_COLOR(204, 153, 255);

// Toyview store the backend data for a XToy entity
class ToyView : public QObject
{
    Q_OBJECT

private:
    MemoryPool<EdgeContainer> m_subContourPool;
    MemoryPool<PointContainer> m_subCtrlPtsPool;

public:
    ToyView();
    ~ToyView();
    void clearSubToyGL();

    void load(QString in_dir);
    void save(std::string out_dir);

    //--------------------------Common Settings----------------------------
public Q_SLOTS:
    void removeViewer(EasyGL *gl);

public:
    XSubToy *getCurToy();
    //--------------------------Lines-------------------------------------
    // user-input sketch_line
    void draw_line(EasyGL *gl);
    void update_line(EasyGL *gl);
    void add_to_sym_line(const QPoint &x, EasyGL *gl);

    //depth plane
    void set_plane(const Plane &plane);
    void draw_plane(EasyGL *gl);

    // outer_contour
    void add_outer_contour(const std::vector<Pnt3> &outer_contour, EasyGL *gl);
    void draw_outer_contour(EasyGL *gl);
    void update_outer_contour(EasyGL *gl);

    // body-part contours
    void sketchline2Spline(const vector<Pnt3> &sketchLine, CSpline &spline, EasyGL *gl);
    void extract_part_contour(const std::vector<Pnt3> &outer_contour, EasyGL *gl); //ToDO: remove

    EdgeContainer *getSubContourGL(int subToyID); // create a GL if new subToy is added, and also feed data to gl
    PointContainer *getSubCtrlPtsGL(int subToyID);
    void computeSubContour(int subToyID);
    bool add_subContour(const std::vector<GLVec3> &sketchLine, EasyGL *gl); // the first most important function!!!
    void deleteSubContour(int id);
    bool delete_subContour();
    void draw_subContours(EasyGL *gl);
    void draw_subCtrlPts(EasyGL *gl);
    void update_subContours(EasyGL *gl);
    void update_subContours(int subToyID, EasyGL *gl);
    void add_symetry_toy(EasyGL *gl, SymetryPlane sym, bool wstParent);

    void compute_chordal_axis();
    void draw_chordal_axis(EasyGL *gl);

    //--------------------------Selection, Translation, Rotation -------------------------------------
    void pick_all_subtoys();
    void unpick_all_subtoys();
    void pick_current_subtoy();
    bool pickSubToy(const QPoint &point, EasyGL *gl, bool ctrl_down = false);
    bool pickParentSubToy(const QPoint &point, EasyGL *gl);
    void translate(const QPoint &point, EasyGL *gl);
    void rotate(int axis, const QPoint &curP, const QPoint &lastP, EasyGL *gl);
    void rotate(int axis, int angle, EasyGL *gl);

    void setRotCenter();
    void draw_arcBall(EasyGL *gl);
    void update_mesh_color();

    //-------------------------- Coarse Mesh 2D --------------------------------------
    void addSubMeshCoarse(EasyGL *gl);
    MeshGL *getSubMeshGLCoarse(int subToyID, TMesh *mesh_ptr);
    void updateSubMeshCoarse(EasyGL *gl);
    void deleteSubMeshCoarse(int subToyID);
    void draw_subMeshCoarse(EasyGL *gl);

    //--------------------------Fine Mesh 2D -------------------------------------------
    void addSubMeshFine(EasyGL *gl);
    MeshGL *getSubMeshGLFine(int subToyID, TMesh *mesh_ptr);
    MeshGL *getSubMeshGLDeform(int subToyID, TMesh *mesh_ptr);

    void draw_subMeshFine(EasyGL *gl);
    void updateSubMeshFine(EasyGL *gl);
    void updateSubMeshFine(int subToyID, EasyGL *gl);
    void deleteSubMeshFine(int subToyID);
    void computeSubMeshFine(int id);

    void draw_subMeshDeform(EasyGL *gl);
    void updateSubMeshDeform(EasyGL *gl);
    void deleteSubMeshDeform(int subToyID);

    //--------------------------Fine Mesh 3D -------------------------------------------
    void addSubMesh3D(EasyGL *gl);
    MeshGL *getSubMeshGL3D(int subToyID, TMesh *mesh_ptr);
    void deleteSubMesh3D(int subToyID);
    void computeSubMesh3D(int subtoy_id);
    void computeSubMesh3D();
    void updateSubMesh3D(int subtoy_id, EasyGL *gl);
    void updateSubMesh3D(EasyGL *gl);
    void drawSelectedSubMesh3D(EasyGL *gl);
    void draw_subMesh3D(EasyGL *gl);
    //--------------------------Modify Contour for Shape Deformation ---------------------
    bool pickDefCtrlPts(const QPoint &point, EasyGL *gl);
    int pickPtOnSketchSpline(const QPoint &point, const CSpline &spline, EasyGL *gl);
    void init_contour_deform(EasyGL *gl);
    void modifySubContourOnMouseMove(const QPoint &point, EasyGL *gl); //ToDO
    void computeDefContour();
    void updateDefContour(EasyGL *gl);
    void drawDefContour(EasyGL *gl);
    // -------------------------Fast Automatic Skinning for Posture Deformation-------------
    void pickBone(const QPoint &pixel, bool shift_down, bool ctrl_down);
    void dragBone(const QPoint &pixel, bool right_click, bool shift_down, bool ctrl_down);
    void releaseBone(EasyGL *gl);
    void deleteBone();
    void recoverRestPose(EasyGL *gl);
    void update_skel(EasyGL *gl);
    void update_restSkel(EasyGL *gl);
    void setBoneParent();
    void splitBone();
    void mergeBone();

    void drawBone(EasyGL *gl);
    void drawRestBone(EasyGL *gl);
    void draw_mesh2D(EasyGL *gl);

    void update_mesh3D(EasyGL *gl);
    void draw_mesh3D(EasyGL *gl);
    bool compute_weightsTet(EasyGL *gl);
    void switch_to_deform_mode(EasyGL *gl);

    void plus_wi(EasyGL *gl);
    void minus_wi(EasyGL *gl);
    void set_selected_bone_dof_type();

    // ------------------------2D/3D SubMeshes------------------------------------------
    void addMeshes(EasyGL *gl);
    void deleteSubToy();
    void updateSubToyGL();

    // ------------------------Painting------------------------------------------

public:
    void TexturePaintOnLBDown(QPoint point, EasyGL *gl);
    void TexturePaintDraw();
    void TextureImageOnLBDown(QPoint point, EasyGL *gl) {}
    void TextureImageOnMouseMove(QPoint point, EasyGL *gl);
    void TextureImageOnLBUp(QPoint point, EasyGL *gl);
    void TextureImageOnDbClk(QPoint point, EasyGL *gl);
    void SelTexImageBy2DSplines(EasyGL *gl);
    bool IsTendToBeClosed(const vector<Pnt2> &pts) {} // ToDO
    // Texture
    void paintAuto2D(EasyGL *gl);
    // ------------------------Skeletonize------------------------------------------
    void skeletonize(XSubToy *subToy); // Thinning

    //--------------------------Animation -------------------------------------------
    void push_keyframe();
    void pop_keyframe();

public:
    //--------------------------Common Settings-------------------------------------
    static QVector<QColor> rnd_colors;

private:
    //--------------------------Lines-------------------------------------
    // user-input sketch_line
    EdgeContainer sketchgl;

    // body-outer contour
    EdgeContainer contour_gl; // silouette
    GLLine3 contourPts;

    // body-part contour
    vector<EdgeContainer *> m_subContourGL;
    vector<PointContainer *> m_subCtrlPtsGL;
    std::vector<GLLine3> subContours; //m_subToy.m_sketchPolyLine
    std::vector<GLLine3> subCtrlPts;

    // Chordal Axis
    // vector<vector<EdgeContainer *>> subChordal_gl;
    // vector<vector<Line3>> subChordals;

    //--------------------------Mesh --------------------------------------------------
    std::vector<MeshGL *> m_subMeshGLCoarse; // 2D Coarse Mesh
    std::vector<MeshGL *> m_subMeshGLFine;   // 2D Fine Mesh
    std::vector<MeshGL *> m_subMeshGLDeform; // 2D Fine Mesh after LBS deformation
    std::vector<MeshGL *> m_subMeshGL3D;
    //--------------------------Contour Deformation for Shape(Mesh3D)-------------------
    EdgeContainer defContour_gl;
    PointContainer defCtrlPts_gl;
    vector<GLVec3> defContour;
    vector<GLVec3> defCtrlPts;

public:
    // interactively capture user-input
    std::vector<GLVec3> m_sketchline;
    SymmetrySketch m_sym_sketch;

    // underlying datastructure
    XToy *m_toy = 0;

    // Fast-Automatic Skinning
    MeshGL *m_meshGL2D = 0; //ToDO: remove
    MeshGL *m_meshGL3D = 0;
    int m_selected_wi = 0;  //
    int m_cur_dof_type = 0; // Fixed position

    //--------------------------Skeleton --------------------------------------------------
    SkelGL *m_skelGL = nullptr;
    SkelGL *m_restSkelGL = nullptr;
    std::vector<Bone *> m_selBones; // size must == 2!!! m_selBones[0]: parent, m_selBones[1]: child

    //--------------------------ArcBall for Rotation -------------------
    ArcBallGL *m_arcBallGL;

    //--------------------------3D plane: slice -------------------
    PlaneGL *m_planeGL;
};

class MeshGL //Actually is subToy GL
{
public:
    // MeshGL(TMesh &mesh_, ToyView *toyview, XToy *toy_, XSubToy *subToy);
    MeshGL() : mesh(nullptr), mesh_color(SUBMESH_COLOR) {}
    MeshGL(TMesh *mesh_ptr);

    void removeViewer(EasyGL *gl);
    void update(EasyGL *gl);
    void set_mesh(TMesh *mesh_ptr);
    void draw_mesh(EasyGL *gl);
    void setMeshColor(const QColor &color)
    {
        mesh_color = color;
    }
    void setTexture(EasyGL *gl, QOpenGLTexture *texture)
    {
        tc.setTexture(gl, texture);
    }

    void compute_element();
    void compute_color();
    void compute_uv(EasyGL *gl);
    void compute_skinning_colors(const Eigen::MatrixXd &W, int wi);
    void GetVData();
    void GetFData();
    void GetVNormal();
    void GetVColor();

public:
    TMesh *mesh = nullptr;
    bool show_skinning_color = false;
    std::vector<unsigned int> idx_data;
    std::vector<GLVec3> vertices;
    std::vector<GLVec3> v_normals;
    std::vector<GLVec3> v_colors;
    std::vector<GLVec3> v_skinning_colors;
    std::vector<GLVec2> v_texCoords;
    TriangleContainer tc;
    QColor mesh_color = SUBMESH_COLOR;
};

class SkelGL
{
public:
    SkelGL(Skeleton<Bone> *skel_);

    void removeViewer(EasyGL *gl);
    void update(EasyGL *gl);
    void draw(EasyGL *gl);
    void compute_element(EasyGL *gl);
    void compute_drag_line(EasyGL *gl, Bone *bone);

    Skeleton<Bone> *skel;
    std::vector<GLVec3> tips;         // Bone tip
    std::vector<GLVec3> lines;        // Bone segment
    std::vector<GLVec3> drag_line[2]; // Dragging line
    PointContainer point_container;
    EdgeContainer line_container;
    EdgeContainer dragline_container[2];
    static QColor bone_color;
};

class ArcBallGL
{
public:
    ArcBallGL()
    {
        point_container.setPointPtr(&points);
        dragline_container.setEdgePtr(&drag_line);
    }

    void removeViewer(EasyGL *gl);
    void update(EasyGL *gl);
    void draw(EasyGL *gl);
    void set_centroid(const Pnt3 &p);
    const Pnt3 &centroid() const { return centroid_; }
    void set_cur_point(const QPoint &qp, EasyGL *gl);
    void set_cur_point(const Pnt3 &cur_point, EasyGL *gl);

    Pnt3 centroid_;
    std::vector<GLVec3> points;
    PointContainer point_container;
    Pnt3 cur_point_;
    std::vector<GLVec3> drag_line; // Dragging line
    EdgeContainer dragline_container;
};

class PlaneGL
{
public:
    PlaneGL()
        : plane(Vec3(0, 0, 1), Pnt3(0, 0, 0))
    {
        compute_element();
        tc.vertices = &vertices;
        tc.idx_data = &idx_data;
        tc.v_normals = &vnormals;
        tc.v_colors = &vcolors;

        ec.setEdgePtr(&line);
        ec.setColor(QColor(0, 0, 0));
    }

    void removeViewer(EasyGL *gl)
    {
        tc.removeViewer(gl);
        ec.removeViewer(gl);
    }

    void set_plane(const Plane &plane_) { plane = plane_; }
    void compute_element();
    void update(EasyGL *gl);
    void draw(EasyGL *gl);

    double edge_length = 800;
    Plane plane;

    std::vector<unsigned int> idx_data;
    std::vector<GLVec3> vertices;
    std::vector<GLVec3> vnormals;
    std::vector<GLVec3> vcolors;

    std::vector<GLVec3> line;
    EdgeContainer ec;
    TriangleContainer tc;
};

class AxisGL
{
    AxisGL() : axis_x_color(1, 0, 0),
               axis_y_color(0, 1, 0),
               axis_z_color(0, 0, 1)
    {
    }

    void calArrow(double r, double R, int prec, // cylinder radius, cone radius, precision
                  GLVec3 from, GLVec3 to,
                  GLVec3 color,
                  std::vector<float> &data);
    void compute_element(EasyGL *gl);

    GLVec3 axis_x_color;
    GLVec3 axis_y_color;
    GLVec3 axis_z_color;
    std::vector<float> coord_data;
    double axis_length;
};

#endif // TOYVIEW_H
