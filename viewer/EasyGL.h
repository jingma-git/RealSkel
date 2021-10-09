#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QImage>

#include <vector>

#include <opencv2/opencv.hpp>

#include "Viewer_interface.h"
#include "toyview.h"
#include "trackball.h"
#include "common.h"

// Display the stored data from different viewpoints
class EasyGL : public Viewer_interface
{
    Q_OBJECT

public:
    EasyGL(QWidget *parent = nullptr, EasyGL *sharedViewer = nullptr);
    ~EasyGL();

    static std::vector<QOpenGLShaderProgram *> shader_programs;
    static int num_viewers;
    static QList<EasyGL *> &EasyGLPool();
    static QOpenGLShaderProgram *getShaderProgram(int name);
    static void repaint_all();

    void makeCurrent();
Q_SIGNALS:
    void contextIsDestroyed();
    void doneInitGL(EasyGL *);

public:
    //------------------------------
    double GetWorldCenterZ();
    QVector4D Cvt2Dto3D(int x, int y, QVector3D ref = QVector3D(0, 0, 0));
    QVector3D Cvt2Dto3D(QPoint p, QVector3D ref = QVector3D(0, 0, 0));
    QVector3D Cvt2Dto3D(QPoint p, float depth);
    Pnt3 Cvt2Dto3D(int x, int y, float depth);
    Pnt3 Cvt2Dto3DIWorld(int x, int y, float depth); // world matrix is identity
    double GetNearSZ();                              // z value in depth buffer
    double GetFarSZ();                               // z value in depth buffer
    // Convert Screen-Space Point to World-Space Point
    void Cvt2Dto3D(const int sx, const int sy, const float depth, int &x, int &y, int &z);
    QVector4D Cvt3Dto2D(const QVector3D &p);
    Pnt2 Cvt3Dto2D(const Pnt3 &p);
    void Cvt3Dto2D(const int x, const int y, const int z, int &sx, int &sy);
    QVector3D MousePosToNDC(const QPoint &p);
    void SetViewMatByTrackBall();
    float Get3dDisOfOnePixelOn2D();
    float GetNDCDisOfOnePixelOn2D();
    QVector4D ProjectToNDC(const QVector3D &p);     // [world_coordinate]-->world->view->projection--->[NDC coordinate]
    QVector4D UnprojectFromNDC(const QVector3D &p); // [NDC coordinate]-->projection->view->world-->[world coordinate]
    bool IsFrontView();
    bool IsPerspectiveView(); // non-canonical view, views that are not front-back-left-right-top-bottom view

    template <typename T>
    void GetViewDir(T &x, T &y, T &z)
    {
        x = -m_viewMatrix(2, 0);
        y = -m_viewMatrix(2, 1);
        z = -m_viewMatrix(2, 2);
    }

    Vec3 GetViewDir()
    {
        double x, y, z;
        x = -m_viewMatrix(2, 0);
        y = -m_viewMatrix(2, 1);
        z = -m_viewMatrix(2, 2);
        return Vec3(x, y, z);
    }

public:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    // void timerEvent(QTimerEvent *e);
    void wheelEvent(QWheelEvent *event);
    void takeSnapshot();

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();
    void draw();
    void update_mvp();
    void displayFPS();
    void drawText(int x, int y, const QString &text, const QFont &fnt = QFont());

private:
    void setDepthState();
    void setTransparentState();

public:
    //--------------------------Background Image -------------------------------------
    void draw_bgImg();
    void update_bgImg();
    void compute_texScl(); // scale && translate the raw_img(img_space) to screen space
    void compute_bgVerts();
    void set_bgImg(const QImage &bgImg);
    const GLVec3 &getBgVert(int i) { return bg_verts[i].position; }
    //--------------------------Contour Deformation -------------------------------------
    bool is_capture_contour = false;
    bool drawMeshWithSingleColor = false;
    vector<vector<double>> m_depthImg;
    cv::Mat m_contourImg;
    void capture_contour();
    void render_single_submesh3d();
    void screenshot();

public:
    // -----Selection-----------------
    QMatrix4x4 m_mvp;
    QMatrix4x4 m_mv;

    float m_camerapos; //distance camera to the world center
    int m_viewMode;
    QVector3D m_eye;
    QVector3D m_up;
    QVector3D m_center;
    QMatrix4x4 m_worldMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_projMatrix;
    TrackBall m_trackBall;
    bool m_isTrackBall = false;
    double m_zoom;

    static QColor ambient_color;
    static QColor diffuse_color;
    static QColor spec_color;
    static QVector3D light_pos;
    static QVector4D light_amb;
    static QVector4D light_diff;
    static QVector4D light_spec;
    static float spec_power;
    static float alpha;

    static int m_minLayer;
    static int m_maxLayer;
    static int m_curLayer; //ToDO: make this as a static variable
    static bool m_isAdvanced;
    static int m_rotAxis;
    static int m_rotAngle;
    QPoint m_lastPoint;

    //----Different View may have different background image----------------------------
    // 2D texture
    QOpenGLTexture *bg_texture = 0;
    ImageContainer imgContainer;
    QImage m_bgImg;
    std::vector<GLVertex3> bg_verts;
    GLVec3 m_bgCent; // on world space
    float m_bgScl = 1.0;
    QVector3D m_bgPressPos;
    float m_texScl = 1.0;
    float m_texHalfW = 0;
    float m_texHalfH = 0;

    // ----Mesh-----------------------
    int m_meshMode = MeshMode::MESH_MODE_PLAIN;
    bool isDrawBgImg = true;
    bool isDrawSkel = true;
    bool isPolygonMode = false;
    bool isTransparentMode = true;
    bool isDepthTest = true;
    QColor m_bgColor;
    bool is2DMode = true;

    // ----Sketch---------------------
    ToyView *m_curView;
    bool m_isDrawPlane = false;

    // ----Deform | Animation --------
    OperationMode m_opMode = OperationMode::OPMODE_DRAW;
    bool with_weights = false;

    // --- fps -----------------------
    QTime m_fpsTime;
    unsigned int m_fpsCount;
    qreal m_fps;
    QString m_fpsString;
    unsigned int m_maxCount = 20;

public:
    // --- multiple windows -----------------------
    EasyGL *m_sharedViewer;
    bool m_isNewViewer;
};

#endif // OPENGLWIDGET_H
