#include <cmath>
#include <QApplication>
#include <QMouseEvent>
#include <QFileDialog>
#include <QPainter>
#include <QMessageBox>

#include <opencv2/opencv.hpp>

#include "Detector.h"
#include "CSketchManager.h"
#include "MapsData.h"
#include "EasyGL.h"
#include "Three.h"
#include "SetInfoDialog.h"
#include "SymetryDialog.h"
#include "SymmetryAxisDialog.h"
#include "BoneComplexityDialog.h"

#define CAMERADIST 500

static void qimage_to_mat(const QImage &image, cv::OutputArray out)
{

    switch (image.format())
    {
    case QImage::Format_Invalid:
    {
        cv::Mat empty;
        empty.copyTo(out);
        break;
    }
    case QImage::Format_RGB32:
    {
        // M_DEBUG << "Format_RGB32" << endl;
        cv::Mat view(image.height(), image.width(), CV_8UC4, (void *)image.constBits(), image.bytesPerLine());
        view.copyTo(out);
        cvtColor(out, out, cv::COLOR_BGRA2BGR);
        break;
    }
    case QImage::Format_RGB888:
    {
        // M_DEBUG << "Format_RGB888" << endl;
        cv::Mat view(image.height(), image.width(), CV_8UC3, (void *)image.constBits(), image.bytesPerLine());
        cvtColor(view, out, cv::COLOR_RGB2BGR);
        break;
    }
    default:
    {
        // M_DEBUG << "QImage::Format_ARGB32" << endl;
        QImage conv = image.convertToFormat(QImage::Format_ARGB32);
        cv::Mat view(conv.height(), conv.width(), CV_8UC4, (void *)conv.constBits(), conv.bytesPerLine());
        view.copyTo(out);
        cvtColor(out, out, cv::COLOR_BGRA2BGR);
        break;
    }
    }
}

int EasyGL::num_viewers = 0;

QList<EasyGL *> &EasyGL::EasyGLPool()
{
    static QList<EasyGL *> EasyGLPool_;
    void *p = qApp->property("easygl pool").value<void *>();
    if (p == 0)
    {
        p = (void *)(&EasyGLPool_);
        qApp->setProperty("easygl pool", QVariant::fromValue(p));
    }
    return *static_cast<QList<EasyGL *> *>(p);
}

std::vector<QOpenGLShaderProgram *> EasyGL::shader_programs = std::vector<QOpenGLShaderProgram *>(NB_OF_PROGRAMS);

QColor EasyGL::ambient_color = QColor(77, 77, 77);
QColor EasyGL::diffuse_color = QColor(204, 204, 204);
QColor EasyGL::spec_color = QColor(0, 0, 0);
QVector3D EasyGL::light_pos = QVector3D(0, 800.f, 0.f); //ToDO, add settings.h
QVector4D EasyGL::light_amb = QVector4D(ambient_color.redF(), ambient_color.greenF(), ambient_color.blueF(), 1.f);
QVector4D EasyGL::light_diff = QVector4D(diffuse_color.redF(), diffuse_color.greenF(), diffuse_color.blueF(), 1.f);
QVector4D EasyGL::light_spec = QVector4D(spec_color.redF(), spec_color.greenF(), spec_color.blueF(), 1.f);
float EasyGL::spec_power = 51.8f;
float EasyGL::alpha = 0.8f;
int EasyGL::m_minLayer = -3;
int EasyGL::m_maxLayer = 3;
int EasyGL::m_curLayer = 0;
bool EasyGL::m_isAdvanced = true;
int EasyGL::m_rotAxis = ROT_AXIS_Z;
int EasyGL::m_rotAngle = 0;

void EasyGL::makeCurrent()
{
    Three::setCurrentViewer(this);
    QOpenGLWidget::makeCurrent();
}

QOpenGLShaderProgram *EasyGL::getShaderProgram(int name)
{
    switch (name)
    {
    case PROGRAM_POINT:
    {
        if (shader_programs[name])
        {
            return shader_programs[name];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram;
            program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/shader_point.vert");
            program->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/shader/shader_point.geom");
            program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/shader_point.frag");
            if (!program->link())
            {
                qDebug() << "Failed to link line shader program\n";
            }
            shader_programs[name] = program;
            return program;
        }
    }
    case PROGRAM_LINE:
    {
        if (shader_programs[name])
        {
            return shader_programs[name];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram;
            program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/shader_line.vert");
            program->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/shader/shader_line.geom");
            program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/shader_line.frag");
            if (!program->link())
            {
                qDebug() << "Failed to link line shader program\n";
            }
            shader_programs[name] = program;
            return program;
        }
    }
    case PROGRAM_BG:
    {
        if (shader_programs[name])
        {
            return shader_programs[name];
        }
        else
        {
            QOpenGLShaderProgram *tex2D_shaderProgram = new QOpenGLShaderProgram;
            tex2D_shaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/shader_texture.vert");
            tex2D_shaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/shader_texture.frag");
            if (!tex2D_shaderProgram->link())
            {
                qDebug() << "Failed to link tex2D shader program\n";
            }
            shader_programs[name] = tex2D_shaderProgram;
            return tex2D_shaderProgram;
        }
    }
    case PROGRAM_MESH:
    {
        if (shader_programs[name])
        {
            return shader_programs[name];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram;
            program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/shader_mesh.vert");
            program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/shader_mesh.frag");
            if (!program->link())
            {
                qDebug() << "Failed to link line shader program\n";
            }
            shader_programs[name] = program;
            return program;
        }
    }
    case PROGRAM_MESH_WITH_TEXTURE:
    {
        if (shader_programs[name])
        {
            return shader_programs[name];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram;
            program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/shader_mesh_with_texture.vert");
            program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/shader_mesh_with_texture.frag");
            if (!program->link())
            {
                qDebug() << "Failed to link line shader program\n";
            }
            shader_programs[name] = program;
            return program;
        }
    }
    default:
        std::cerr << "ERROR : Program not found." << std::endl;
        return 0;
    }
}

void EasyGL::repaint_all()
{
    for (auto v : EasyGL::EasyGLPool())
    {
        v->repaint();
    }
}

EasyGL::EasyGL(QWidget *parent, EasyGL *sharedViewer)
    : Viewer_interface(parent),
      m_sharedViewer(sharedViewer),
      //   m_bgColor(240, 240, 245),
      //   m_bgColor(QColor(Qt::transparent)),
      m_bgColor(255, 255, 255),
      m_bgCent(0, 0, 0),
      m_bgScl(1.0)
{
    setObjectName(QString("Viewer%1").arg(num_viewers));
    ++num_viewers;
    setFocusPolicy(::Qt::StrongFocus);

    EasyGLPool().append(this);
    m_camerapos = CAMERADIST;
    m_zoom = 1.0;

    m_worldMatrix.setToIdentity();

    // Set view matrix
    m_viewMode = VIEW_FRONT; //front view
    m_eye = QVector3D(0, 0, m_camerapos + 300);
    m_center = QVector3D(0, 0, 0);
    m_up = QVector3D(0, 1, 0);
    m_viewMatrix.lookAt(m_eye, m_center, m_up);

    // Set proj matrix
    float aspect = width() / height();
    m_projMatrix.ortho(-m_camerapos, m_camerapos, -m_camerapos / aspect, m_camerapos / aspect, -10000.f, 10000.f);

    m_mvp = m_projMatrix * m_viewMatrix * m_worldMatrix;
    m_mv = m_viewMatrix * m_worldMatrix;

    // fps
    m_fpsTime.start();
    m_fpsCount = 0;
    m_fps = 0.0;
    m_fpsString = tr("%1fps", "Frames per seconds").arg("?");
    m_maxCount = 10;

    if (m_sharedViewer == nullptr) // add main viewer
    {
        m_curView = new ToyView();
        m_isNewViewer = false;
    }
    else
    { // assitant view points to the same 3D toy model
        m_curView = m_sharedViewer->m_curView;
        m_opMode = m_sharedViewer->m_opMode;
        m_meshMode = m_sharedViewer->m_meshMode;
        // ToDO
        m_isNewViewer = true;
    }

    // bgImg
    bg_texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
    imgContainer.uv_ptr = &bg_verts;
}

EasyGL::~EasyGL()
{
    // Make OpenGL context current and call cleanupGL()
    Q_DEBUG << "~EasyGL() " << objectName();
    --num_viewers;

    delete bg_texture;
    if (EasyGLPool().size() == 1)
    {
        if (m_curView)
        {
            delete m_curView;
            m_curView = 0;
        }

        for (int i = 0; i < shader_programs.size(); ++i)
        {
            if (shader_programs[i])
            {
                delete shader_programs[i];
                shader_programs[i] = 0;
            }
        }
    }
    EasyGL::EasyGLPool().removeAll(this);
}

double EasyGL::GetWorldCenterZ()
{
    QMatrix4x4 pvm = m_projMatrix * m_viewMatrix * m_worldMatrix;

    // acquire projected coordinate for world_center (get projected z (NDC_z) for world_center)
    QVector4D world_center(0, 0, 0, 1.0);
    world_center = pvm * world_center;
    return world_center[2];
}

QVector4D EasyGL::Cvt2Dto3D(int x, int y, QVector3D ref)
{
    // screen-space to NDC-spac
    float xproj = (x - (float)width() / 2.f) / ((float)width() / 2.f);
    float yproj = (-y + (float)height() / 2.f) / ((float)height() / 2.f);

    QMatrix4x4 pvm = m_projMatrix * m_viewMatrix * m_worldMatrix;

    // acquire projected coordinate for world_center (get projected z (NDC_z) for world_center)
    QVector4D world_center(ref.x(), ref.y(), ref.z(), 1.0f);
    world_center = pvm * world_center;

    // NDC-->view space (unproject)-->world
    QVector4D v(xproj, yproj, world_center.z(), 1);
    // NDC-space to world space
    QVector4D v_world = pvm.inverted() * v;
    return v_world;
}

QVector3D EasyGL::Cvt2Dto3D(QPoint p, QVector3D ref)
{
    QVector4D v_world = Cvt2Dto3D(p.x(), p.y());
    return {v_world.x(), v_world.y(), v_world.z()};
}

QVector3D EasyGL::Cvt2Dto3D(QPoint p, float depth)
{
    Pnt3 v_world = Cvt2Dto3D(p.x(), p.y(), depth);
    return QVector3D(v_world.x(), v_world.y(), v_world.z());
}

Pnt3 EasyGL::Cvt2Dto3D(int x, int y, float depth)
{
    // screen-space to NDC-spac
    float xproj = (x - (float)width() / 2.f) / ((float)width() / 2.f);
    float yproj = (-y + (float)height() / 2.f) / ((float)height() / 2.f);

    QMatrix4x4 pvm = m_projMatrix * m_viewMatrix * m_worldMatrix;

    // NDC-->view space (unproject)-->world
    QVector4D v(xproj, yproj, depth, 1);
    // NDC-space to world space
    QVector4D v_world = pvm.inverted() * v;
    return Pnt3(v_world.x(), v_world.y(), v_world.z());
}

Pnt3 EasyGL::Cvt2Dto3DIWorld(int x, int y, float depth)
{
    // screen-space to NDC-spac
    float xproj = (x - (float)width() / 2.f) / ((float)width() / 2.f);
    float yproj = (-y + (float)height() / 2.f) / ((float)height() / 2.f);

    QMatrix4x4 pv = m_projMatrix * m_viewMatrix;

    // NDC-->view space (unproject)-->world
    QVector4D v(xproj, yproj, depth, 1);
    // NDC-space to world space
    QVector4D v_world = pv.inverted() * v;
    return Pnt3(v_world.x(), v_world.y(), v_world.z());
}

double EasyGL::GetNearSZ()
{
    QVector3D viewDir;
    GetViewDir(viewDir[0], viewDir[1], viewDir[2]);
    QVector3D p = -viewDir.normalized() * (m_camerapos + 300);
    return ProjectToNDC(p).z();
}

double EasyGL::GetFarSZ()
{
    QVector3D viewDir;
    GetViewDir(viewDir[0], viewDir[1], viewDir[2]);
    QVector3D p = viewDir.normalized() * (m_camerapos + 300);
    return ProjectToNDC(p).z();
}

void EasyGL::Cvt2Dto3D(const int sx, const int sy, const float depth, int &x, int &y, int &z)
{
    // screen-space to NDC-spac
    float xproj = (sx - (float)width() / 2.f) / ((float)width() / 2.f);
    float yproj = (-sy + (float)height() / 2.f) / ((float)height() / 2.f);

    QMatrix4x4 pvm = m_projMatrix * m_viewMatrix * m_worldMatrix;

    // NDC-->view space (unproject)-->world
    QVector4D v(xproj, yproj, depth, 1);
    // NDC-space to world space
    QVector4D v_world = pvm.inverted() * v;
    x = v_world.x();
    y = v_world.y();
    z = v_world.z();
}

QVector4D EasyGL::Cvt3Dto2D(const QVector3D &p)
{
    QVector4D v(p.x(), p.y(), p.z(), 1.0);
    // world-space to NDC-space
    QVector4D v_ndc = m_projMatrix * m_viewMatrix * m_worldMatrix * v;
    // viewport transf: NDC-space to screen space
    float x = (v_ndc.x() + 1.0) * 0.5 * width();
    float y = (v_ndc.y() + 1.0) * 0.5 * (-height()) + height();
    float z = v_ndc.z();
    // float z = (v_ndc.z() + 1.0f) * 0.5; //[-1, 1]-->[0, 1], ToDO: make it to stay in the range [-1, 1]
    return {x, y, z, 1};
}

Pnt2 EasyGL::Cvt3Dto2D(const Pnt3 &p)
{
    QVector4D v(p.x(), p.y(), p.z(), 1.0);
    // world-space to NDC-space
    QVector4D v_ndc = m_projMatrix * m_viewMatrix * m_worldMatrix * v;
    // viewport transf: NDC-space to screen space
    float x = (v_ndc.x() + 1.0) * 0.5 * width();
    float y = (v_ndc.y() + 1.0) * 0.5 * (-height()) + height();
    return {x, y};
}

void EasyGL::Cvt3Dto2D(const int x, const int y, const int z, int &sx, int &sy)
{
    QVector4D v(x, y, z, 1.0);
    // world-space to NDC-space
    QVector4D v_ndc = m_projMatrix * m_viewMatrix * m_worldMatrix * v;
    // viewport transf: NDC-space to screen space
    sx = (v_ndc.x() + 1.0) * 0.5 * width();
    sy = (v_ndc.y() + 1.0) * 0.5 * (-height()) + height();
}

float EasyGL::Get3dDisOfOnePixelOn2D()
{
    QVector4D vt1 = Cvt2Dto3D(0, 0);
    QVector4D vt2 = Cvt2Dto3D(0, 1);
    float fDis = (vt1 - vt2).length();
    return fDis;
}

float EasyGL::GetNDCDisOfOnePixelOn2D()
{
    QVector4D vt1 = Cvt2Dto3D(0, 0);
    QVector4D vt2 = Cvt2Dto3D(0, 1);
    QMatrix4x4 mvp = m_projMatrix * m_viewMatrix * m_worldMatrix;
    QVector4D v1_ndc = mvp * vt1;
    QVector4D v2_ndc = mvp * vt2;
    return (v1_ndc - v2_ndc).length();
}

QVector4D EasyGL::ProjectToNDC(const QVector3D &p) // [world_coordinate]-->world->view->projection--->[NDC coordinate]
{
    QVector4D v(p.x(), p.y(), p.z(), 1.0);
    // world-space to NDC-space
    QVector4D v_ndc = m_projMatrix * m_viewMatrix * m_worldMatrix * v;
    return v_ndc;
}

QVector4D EasyGL::UnprojectFromNDC(const QVector3D &p) // [NDC coordinate]-->projection->view->world-->[world coordinate]
{
    QVector4D v(p.x(), p.y(), p.z(), 1.0);
    QMatrix4x4 pvm = m_projMatrix * m_viewMatrix * m_worldMatrix;
    QVector4D v_world = pvm.inverted() * v;
    return v_world;
}

bool EasyGL::IsFrontView()
{
    Vec3 viewDir, norm(0, 0, -1);
    GetViewDir(viewDir[0], viewDir[1], viewDir[2]);
    return (viewDir - norm).norm() < 1e-3;
}

QVector3D EasyGL::MousePosToNDC(const QPoint &p)
{
    return QVector3D(
        2.0 * p.x() / (double)width() - 1.0,
        1.0 - 2.0 * p.y() / (double)height(),
        0.0);
}

void EasyGL::SetViewMatByTrackBall()
{
    if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
    {
        m_opMode = OPMODE_SELECT_TOY;
    }
    m_viewMode = VIEW_PERSP;
    m_eye = QVector3D(0, 0, m_camerapos + 300);
    m_up = QVector3D(0, 1, 0);
    m_viewMatrix.setToIdentity();
    m_viewMatrix.lookAt(m_eye, m_center, m_up);
    m_viewMatrix.rotate(m_trackBall.getRotation());
    update_bgImg();
    update_mvp();
}

void EasyGL::mousePressEvent(QMouseEvent *event)
{
    if (m_isNewViewer) //To prevent the default click when adding a subViewer
    {
        m_isNewViewer = false;
        return;
    }

    makeCurrent();

    if (event->buttons() & Qt::MidButton)
    {
        QVector3D p3d = MousePosToNDC(event->pos());
        m_trackBall.mousePress(p3d);
        SetViewMatByTrackBall();
    }
    else if (event->buttons() & Qt::RightButton)
    {

        if (event->modifiers() & Qt::ControlModifier)
        {
            qDebug() << __FILE__ << " " << __LINE__ << " mouse press move background img";
            m_bgPressPos = Cvt2Dto3D(event->pos());
        }
    }
    else
    {
        // operation
        if (m_opMode == OPMODE_DRAW)
        {
            m_curView->m_sketchline.clear();
        }
        else if (m_opMode == OPMODE_DRAW_SYM)
        {
            m_curView->m_sym_sketch.clear();
            m_curView->m_sketchline.clear();
            m_curView->add_to_sym_line(event->pos(), this);
        }
        else if (m_opMode == OPMODE_SELECT_TOY)
        {
            bool ctrl_down = (event->modifiers() & Qt::ControlModifier);
            m_curView->pickSubToy(event->pos(), this, ctrl_down);
        }
        else if (m_opMode == OPMODE_SELECT_PARENT_TOY)
        {
            m_curView->pickParentSubToy(event->pos(), this);
        }
        else if (m_opMode == OPMODE_MOVE)
        {
            m_curView->pickSubToy(event->pos(), this);
        }
        else if (m_opMode == OPMODE_ROTATE)
        {
            m_curView->m_arcBallGL->set_cur_point(event->pos(), this);
            m_lastPoint = event->pos();
        }
        else if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
        {
            m_curView->pickDefCtrlPts(event->pos(), this);
        }
        else if (m_opMode == OPMODE_EDIT_BONE ||
                 m_opMode == OPMODE_EDIT_POINT_HANDLE ||
                 m_opMode == OPMODE_BBW_DEFORM)
        {
            qDebug() << __FILE__ << " " << __LINE__ << " Pick Bone and SubToy...";
            m_curView->pickSubToy(event->pos(), this);
            if (m_opMode == OPMODE_EDIT_BONE)
            {
                m_curView->m_toy->skel->set_editing(true);
                m_curView->m_toy->skel->point_handle_mode = false;
            }
            else if (m_opMode == OPMODE_EDIT_POINT_HANDLE)
            {
                m_curView->m_toy->skel->set_editing(true);
                m_curView->m_toy->skel->point_handle_mode = true;
            }
            else if (m_opMode == OPMODE_BBW_DEFORM)
            {
                m_curView->m_toy->skel->set_editing(false);
            }

            bool shift_down = (event->modifiers() & Qt::ShiftModifier);
            bool ctrl_down = (event->modifiers() & Qt::ControlModifier);
            m_curView->pickBone(event->pos(), shift_down, ctrl_down);
        }
        else if (m_opMode == OPMODE_TEXTURE_PAINT)
        {
            m_curView->TexturePaintOnLBDown(event->pos(), this);
        }
        else if (m_opMode == OPMODE_TEXPAINT_IMAGE)
        {
            m_curView->TextureImageOnLBDown(event->pos(), this);
        }

        // visualization
        for (EasyGL *v : EasyGL::EasyGLPool())
        {
            if (m_opMode == OPMODE_DRAW)
            {
                m_curView->update_line(v);
            }
            else if (m_opMode == OPMODE_BBW_DEFORM)
            {
                m_curView->m_skelGL->update(v);
            }
            else if (m_opMode == OPMODE_EDIT_BONE)
            {
                m_curView->m_skelGL->update(v);
            }
            else if (m_opMode == OPMODE_SELECT_TOY ||
                     m_opMode == OPMODE_MOVE ||
                     m_opMode == OPMODE_ROTATE)
            {
                m_curView->update_subContours(v);
                m_curView->updateSubMeshCoarse(v);
                m_curView->updateSubMeshFine(v);
                m_curView->updateSubMesh3D(v);
                m_curView->m_skelGL->update(v);
                if (m_opMode == OPMODE_ROTATE)
                    m_curView->m_arcBallGL->update(v);
            }
            v->repaint();
        }
    }
}

void EasyGL::mouseMoveEvent(QMouseEvent *event)
{
    // https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Arcball
    // Add input sample and redraw
    if (event->buttons() & Qt::MidButton)
    {

        QVector3D p3d = MousePosToNDC(event->pos());
        m_trackBall.mouseMove(p3d);
        SetViewMatByTrackBall();
        return;
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (event->modifiers() & Qt::ControlModifier)
        {
            m_bgCent = GLVec3(Cvt2Dto3D(event->pos()) - m_bgPressPos);
            update_bgImg();
            return;
        }
    }
    // else if (event->buttons() & Qt::LeftButton)
    {
        // operation
        if (m_opMode == OPMODE_DRAW)
        {
            QPoint p = event->pos();
            QVector4D p3d = Cvt2Dto3D(p.x(), p.y());
            m_curView->m_sketchline.push_back(GLVec3(p3d.x(), p3d.y(), p3d.z()));
        }
        else if (m_opMode == OPMODE_DRAW_SYM)
        {
            m_curView->add_to_sym_line(event->pos(), this);
        }
        else if (m_opMode == OPMODE_MOVE)
        {
            m_curView->translate(event->pos(), this);
        }
        else if (m_opMode == OPMODE_ROTATE)
        {
            m_curView->rotate(m_rotAxis, event->pos(), m_lastPoint, this);
            m_lastPoint = event->pos();
        }
        else if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
        {
            m_curView->modifySubContourOnMouseMove(event->pos(), this);
        }
        else if (m_opMode == OPMODE_EDIT_BONE ||
                 m_opMode == OPMODE_EDIT_POINT_HANDLE ||
                 m_opMode == OPMODE_BBW_DEFORM)
        {
            bool right_click = (event->buttons() & Qt::RightButton);
            bool shift_down = (event->modifiers() & Qt::ShiftModifier);
            bool ctrl_down = (event->modifiers() & Qt::ControlModifier);

            // qDebug() << __FILE__ << " " << __LINE__ << " dragBone right_click=" << right_click;
            m_curView->dragBone(event->pos(), right_click, shift_down, ctrl_down);
        }
        else if (m_opMode == OPMODE_TEXPAINT_IMAGE)
        {
            m_curView->TextureImageOnMouseMove(event->pos(), this);
        }

        // visulization
        for (EasyGL *v : EasyGL::EasyGLPool())
        {
            if (m_opMode == OPMODE_DRAW || m_opMode == OPMODE_DRAW_SYM)
            {
                m_curView->update_line(v);
            }
            else if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
            {
                m_curView->update_subContours(v);
                m_curView->updateDefContour(v);
                m_curView->updateSubMesh3D(v);
            }
            else if (m_opMode == OPMODE_MOVE || m_opMode == OPMODE_ROTATE)
            {
                m_curView->update_subContours(v);
                m_curView->updateSubMeshCoarse(v);
                m_curView->updateSubMeshFine(v);
                m_curView->updateSubMesh3D(v);
                m_curView->m_skelGL->update(v);
                m_curView->m_restSkelGL->update(v);
                if (m_opMode == OPMODE_ROTATE)
                    m_curView->m_arcBallGL->update(v);
            }
            else if (m_opMode == OPMODE_BBW_DEFORM)
            {
                m_curView->updateSubMeshDeform(v);
                m_curView->m_meshGL3D->update(v);
                m_curView->m_skelGL->update(v);
            }
            else if (m_opMode == OPMODE_EDIT_BONE)
            {
                m_curView->m_skelGL->update(v);
            }
            v->repaint();
        }
    }
}

void EasyGL::mouseReleaseEvent(QMouseEvent *event)
{

    if (event->buttons() & Qt::MidButton)
    {
        QVector3D p3d = MousePosToNDC(event->pos());
        m_trackBall.mouseRelease(p3d);
        SetViewMatByTrackBall();
    }
    else
    {
        // opeartion
        if (m_opMode == OPMODE_DRAW || m_opMode == OPMODE_DRAW_SYM)
        {
            if (m_curView->add_subContour(m_curView->m_sketchline, this))
            {
                SetInfoDialog *dialog = new SetInfoDialog(this);
                dialog->show();
                m_opMode = OPMODE_MODIFY_SKETCH_LINES;
            }
            m_curView->m_sketchline.clear();
        }
        else if (m_opMode == OPMODE_EDIT_BONE || m_opMode == OPMODE_BBW_DEFORM)
        {
            m_curView->releaseBone(this);
        }
        else if (m_opMode == OPMODE_TEXPAINT_IMAGE)
        {
            m_curView->TextureImageOnLBUp(event->pos(), this);
        }

        if (m_opMode == OPMODE_ROTATE || m_opMode == OPMODE_MOVE)
        {
            m_curView->save("output");
        }
        // visualization
        for (EasyGL *v : EasyGL::EasyGLPool())
        {
            if (m_opMode == OPMODE_BBW_DEFORM)
            {
                m_curView->m_skelGL->update(v);
            }
            else if (m_opMode == OPMODE_EDIT_BONE)
            {
                m_curView->m_skelGL->update(v);
            }
            v->repaint();
        }
    }
}

void EasyGL::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (m_opMode == OPMODE_SELECT_TOY)
    {
        SetInfoDialog *dialog = new SetInfoDialog(this);
        dialog->show();
    }
    else if (m_opMode == OPMODE_TEXPAINT_IMAGE)
    {
        m_curView->TextureImageOnDbClk(event->pos(), this);
    }
}

void EasyGL::wheelEvent(QWheelEvent *event)
{
    if (event->modifiers() & Qt::ControlModifier)
    {
        qDebug() << __FILE__ << " " << __LINE__ << " scaling background..." << endl;
        m_bgScl += event->delta() * 0.001;
        if (m_bgScl <= 0.1)
        {
            m_bgScl = 0.1;
        }
        update_bgImg();
    }
    else
    {
        // scale viewport
        m_zoom += event->delta() * 0.001;
        if (m_zoom <= 0.1)
        {
            m_zoom = 0.1;
        }
        m_worldMatrix.setToIdentity();
        m_worldMatrix.scale(m_zoom);
        update_mvp();
    }
}

void EasyGL::keyPressEvent(QKeyEvent *e)
{

    // qDebug() << __FILE__ << " " << __LINE__ << " EasyGL " << this;

    if (!e->modifiers())
    {
        if (e->key() == Qt::Key_Delete)
        {
            Q_DEBUG << "delete subtoy";
            m_curView->deleteSubToy();
        }
        // --------------------------- Post Processing -----------------------------------------
        if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
        {
            // m_curView->modifyMeshContour(); // ToDO
        }
        // --------------------------- Unitlity Function ------------------------------------
        if (e->key() == Qt::Key_F2)
        {
            screenshot();
        }
        // --------------------------- Rotation ------------------------------------
        if (m_opMode == OPMODE_ROTATE)
        {
            if (e->key() == Qt::Key_X)
            {
                m_rotAxis = ROT_AXIS_X;
                M_DEBUG << "ROT_AXIS_X" << endl;
            }
            else if (e->key() == Qt::Key_Y)
            {
                m_rotAxis = ROT_AXIS_Y;
                M_DEBUG << "ROT_AXIS_Y" << endl;
            }
            else if (e->key() == Qt::Key_Z)
            {
                m_rotAxis = ROT_AXIS_Z;
                M_DEBUG << "ROT_AXIS_Z" << endl;
            }
        }
        if (m_opMode == OPMODE_DRAW)
        {
            if (e->key() == Qt::Key_X)
            {
                M_DEBUG << "set sym axis X" << endl;
                m_curView->m_sym_sketch.set_sym_axis(SYM_AXIS_X);
            }
            else if (e->key() == Qt::Key_Y)
            {
                M_DEBUG << "set sym axis Y" << endl;
                m_curView->m_sym_sketch.set_sym_axis(SYM_AXIS_Y);
            }
        }
        // --------------------------- Switch Operation Mode ------------------------------------
        if (e->key() == Qt::Key_S) // Swith to Select Mode
        {
            qDebug() << "switch to OPMODE_SELECT_TOY mode";
            for (EasyGL *v : EasyGL::EasyGLPool())
            {
                v->m_opMode = OperationMode::OPMODE_SELECT_TOY;
                v->repaint();
            }
        }
        else if (e->key() == Qt::Key_F3)
        {
            qDebug() << "switch to OPMODE_SELECT_PARENT_TOY mode";
            m_opMode = OperationMode::OPMODE_SELECT_PARENT_TOY;
            repaint_all();
        }
        else if (e->key() == Qt::Key_Space) // Swith to Draw Mode
        {
            qDebug() << "switch to OPMODE_DRAW mode";
            int i = 0;
            for (EasyGL *v : EasyGL::EasyGLPool())
            {
                if (i == 0)
                {
                    v->is2DMode = true;
                    v->isTransparentMode = true;
                    v->isDepthTest = true;
                }

                v->m_opMode = OperationMode::OPMODE_DRAW;
                v->repaint();
                i++;
            }
        }
        else if (e->key() == Qt::Key_M) // Modify Contour (Shape Deformation)
        {
            qDebug() << "switch to OPMODE_MODIFY_SKETCH_LINES mode";
            m_opMode = OperationMode::OPMODE_MODIFY_SKETCH_LINES;
            capture_contour();
            m_curView->init_contour_deform(this);
            repaint();
        }
        else if (e->key() == Qt::Key_D) // Posture Deformation
        {
            qDebug() << "switch to OPMODE_BBW mode Draw skinning colors " << this;
            m_curView->switch_to_deform_mode(this);
            for (EasyGL *v : EasyGL::EasyGLPool())
            {
                v->m_opMode = OPMODE_BBW_DEFORM;
                v->is2DMode = false;
                v->m_meshMode = MESH_MODE_SKINNING_COLORED;
                v->isDepthTest = false;
                v->isTransparentMode = false;

                v->repaint();
            }
        }
        else if (e->key() == Qt::Key_G) // Move SubPart
        {
            if (m_opMode == OPMODE_BBW_DEFORM)
            {
                return;
            }
            else
            {
                m_curView->pick_current_subtoy();
                // notice!!! make all views's opmode as move
                for (EasyGL *v : EasyGLPool())
                {
                    v->m_opMode = OPMODE_MOVE;
                    v->repaint();
                }
                M_DEBUG << " Switch to " << opmode_names[m_opMode] << endl;
            }
        }
        else if (e->key() == Qt::Key_R) // Rotate SubPart
        {
            if (m_opMode == OPMODE_BBW_DEFORM)
            {
                return;
            }
            else
            {
                m_curView->pick_current_subtoy();
                m_curView->setRotCenter();
                m_curView->m_arcBallGL->update(this);
                m_opMode = OPMODE_ROTATE;
                repaint();
                M_DEBUG << " Switch to " << opmode_names[m_opMode] << endl;
            }
        }
        else if (e->key() == Qt::Key_U)
        {
            // UV mapping
            qDebug() << "texturing by ortho projection";
            m_meshMode = MESH_MODE_TEXTURE;
            m_curView->addSubMeshFine(this);
            m_curView->paintAuto2D(this);
            isDrawBgImg = false;
            repaint();
        }
        else if (e->key() == Qt::Key_E) // Edit Bone
        {
            qDebug() << "switch to OPMODE_EDIT_BONE mode"; // make all views to be EDIT mode
            m_curView->m_toy->skel->point_handle_mode = false;

            for (auto v : EasyGL::EasyGLPool())
            {
                v->m_opMode = OperationMode::OPMODE_EDIT_BONE;
                m_curView->update_skel(v);
                v->repaint();
            }
        }
        else if (e->key() == Qt::Key_W) // Compute Skinning Weights
        {
            qDebug() << "switch to OPMODE_BBW mode";
            m_opMode = OperationMode::OPMODE_BBW;
            // m_curView->addSubMeshFine(this);
            // m_curView->addSubMesh3D(this);
            bool flag = m_curView->compute_weightsTet(this);
        }
        else if (e->key() == Qt::Key_Plus)
        {
            qDebug() << __FILE__ << " " << __LINE__ << " push key frame frame_size=";
            m_curView->push_keyframe();
        }
        else if (e->key() == Qt::Key_Minus)
        {
            qDebug() << __FILE__ << " " << __LINE__ << " pop key frame frame_size=";
            m_curView->pop_keyframe();
        }
        else if (e->key() == Qt::Key_L) // Switch Depth Test
        {
            // enable/disable depth test
            isDepthTest = !isDepthTest;
            qDebug() << __FILE__ << " " << __LINE__ << " isDepthTest=" << isDepthTest;
            repaint();
        }
        else if (e->key() == Qt::Key_Backslash) //  Switch 2D/3D Mode
        {
            is2DMode = !is2DMode;
            qDebug() << __FILE__ << " " << __LINE__ << " is2DMode=" << is2DMode;
            repaint();
        }
        else if (e->key() == Qt::Key_I) // Draw Background Img
        {
            isDrawBgImg = !isDrawBgImg;
            Q_DEBUG << " isDrawBgImg=" << isDrawBgImg;
            repaint();
        }
        else if (e->key() == Qt::Key_F4)
        {
            isDrawSkel = !isDrawSkel;
            Q_DEBUG << " isDrawSkel=" << isDrawSkel;
            repaint();
        }
        else if (e->key() == Qt::Key_C) // Bone Complexity
        {
            BoneComplexityDialog *dialog = new BoneComplexityDialog(this);
            dialog->show();
        }

        //-------------------------- Switch weight index------------------------------------------------------
        if (m_opMode == OPMODE_BBW || m_opMode == OPMODE_BBW_DEFORM)
        {
            if (e->key() == Qt::Key_Period)
            {
                m_curView->plus_wi(this);
                qDebug() << "Switch weight index to " << m_curView->m_selected_wi;
                repaint();
            }
            else if (e->key() == Qt::Key_Comma)
            {
                m_curView->minus_wi(this);
                qDebug() << "Switch weight index to " << m_curView->m_selected_wi;
                repaint();
            }
        }
        //-------------------------- Delete, Split, Connect Bones--------------------------------------------
        if (m_opMode == OPMODE_EDIT_BONE)
        {
            if (e->key() == Qt::Key_X)
            {
                m_curView->deleteBone();
                for (auto v : EasyGL::EasyGLPool())
                {
                    v->repaint();
                }
            }
        }

        //-------------------------- Adjust Pose && Change Arap-DOF Constraint Type--------------------------------------------
        if (m_opMode == OPMODE_BBW_DEFORM)
        {
            // ToDO
            if (e->key() == Qt::Key_R)
            {
                m_curView->recoverRestPose(this);
            }

            if (e->key() == Qt::Key_Semicolon)
            {
                m_curView->set_selected_bone_dof_type();
                qDebug() << __FILE__ << " " << __LINE__ << " Change Dof Type to " << DofTypeNames[m_curView->m_cur_dof_type].c_str();
            }
            if (e->key() == Qt::Key_C)
            {
                m_curView->m_toy->auto_dof = !m_curView->m_toy->auto_dof;
                qDebug() << __FILE__ << " " << __LINE__ << " auto_dof=" << m_curView->m_toy->auto_dof;
            }
            for (auto v : EasyGL::EasyGLPool())
            {
                m_curView->update_skel(v);
                if (e->key() == Qt::Key_R)
                {
                    m_curView->updateSubMeshDeform(v);
                    m_curView->update_mesh3D(v);
                }
                v->repaint();
            }
        }

        //-------------------------- Switch mesh drawing mode-------------------------------------------------
        if (e->key() == Qt::Key_P)
        {
            isPolygonMode = (!isPolygonMode);
            if (isPolygonMode)
            {
                qDebug() << "enable glPolygonMode";
            }
            else
            {
                qDebug() << "disable glPolygonMode";
            }
            repaint();
        }
        else if (e->key() == Qt::Key_T)
        {
            isTransparentMode = (!isTransparentMode);
            if (isTransparentMode)
            {
                qDebug() << "enable TransparentMode";
            }
            else
            {
                qDebug() << "disable TransparentMode";
            }
            repaint();
        }
        else if (e->key() == Qt::Key_Backspace)
        {
            m_meshMode += 1;
            if (m_meshMode >= NB_MESH_MODES)
            {
                m_meshMode = 0;
            }
            qDebug() << __FILE__ << " " << __LINE__ << " " << this
                     << " switch to MESH_MODE=" << meshmode_names[m_meshMode].c_str();
            repaint();
        }
    }
    else
    {
        if (e->key() == Qt::Key_Space && e->modifiers() & Qt::ControlModifier)
        {
            qDebug() << "switch to OPMODE_DRAW_SYM mode";
            m_opMode = OperationMode::OPMODE_DRAW_SYM;
        }
        //-------------------------- Open Image, Load and Save Project --------------------------------------------
        if (e->key() == Qt::Key_O && e->modifiers() & Qt::ControlModifier)
        {
            QString path = QFileDialog::getOpenFileName(this,
                                                        tr("Open Image"),
                                                        tr(""),
                                                        tr("Images Files (*.png *.jpg *.jpeg);;"
                                                           "PNG Files (*.png);;"
                                                           "JPG Files (*.jpg);;"
                                                           "JPEG Files (*.jpeg);;"));

            if (!path.isEmpty())
            {
                m_bgImg.load(path);
                M_DEBUG << " bgImg width=" << m_bgImg.width() << " height=" << m_bgImg.height() << endl;
                std::string fileName = QFileInfo(path).fileName().toStdString();
                size_t pos = fileName.find_first_of(".");
                std::string prefix = fileName.substr(0, pos);
                std::string postfix = fileName.substr(pos + 1, fileName.length() - pos);
                m_curView->m_toy->SetName(prefix);
                M_DEBUG << "prefix: " << prefix << endl;
                M_DEBUG << "postfix: " << postfix << endl;
                m_curView->m_sketchline.clear();
                set_bgImg(m_bgImg); // Notice: this line must be put before dectector

                // automatically detect contour
                cv::Mat bgMat;
                qimage_to_mat(m_bgImg, bgMat);
                vector<vector<cv::Point>> contours;
                Detector::detect_contour(bgMat, contours);
                // ToDO: comment this out
                cv::Mat contour_img;
                Detector::draw_contour(contour_img, bgMat, contours[0], cv::Scalar(0, 0, 255));
                cv::imwrite("output/" + prefix + "_contour.png", contour_img);

                // img space to 3D space
                int x_cent = width() / 2;
                int y_cent = height() / 2;
                float scl = m_texScl;
                float half_w = m_texHalfW;
                float half_h = m_texHalfH;
                float off_x = x_cent - half_w;
                float off_y = y_cent - half_h;

                double z = GetWorldCenterZ();
                vector<cv::Point> &contour = contours[0];
                vector<Pnt3> outer_contour;
                outer_contour.reserve(contour.size());

                for (int i = 0; i < contour.size(); ++i)
                {
                    Pnt3 p3d = Cvt2Dto3D(contour[i].x * scl + off_x, contour[i].y * scl + off_y, z);
                    Pnt3 bgCent(m_bgCent.x, m_bgCent.y, m_bgCent.z);
                    p3d += bgCent;
                    p3d *= m_bgScl;
                    outer_contour.push_back(p3d);
                }
                // m_curView->extract_part_contour(outer_contour, this);
                m_curView->add_outer_contour(outer_contour, this);

#ifdef AUTO_3DREC
                Line3 contour;                                //visualization
                vector<Pnt2> ct_pts;                          //backend datastructure
                vector<Pnt3> ct_pts3, ct_simPts3, ct_uniPts3; // Debug

                for (const cv::Point &p : contours[0])
                {
                    contour.push_back(GLVec3(p.x, p.y, 0.0));
                    ct_pts.push_back(Pnt2(p.x, p.y));
                    ct_pts3.push_back(Pnt3(p.x, p.y, 0.0));
                }
                contour.push_back(contour.front());
                ct_pts.push_back(ct_pts.front());

                SimplifyPolyLine(ct_pts3, ct_simPts3, 3);
                UniformPolyLine(ct_simPts3, ct_uniPts3, 8 * 3);
                CSpline spline;
                spline.ChangeMode(CSpline::SPLMODE_CLOSED_SPLINE);
                if ((ct_uniPts3[0] - ct_uniPts3.back()).norm() < 0.001)
                {
                    ct_uniPts3.pop_back();
                }

                qDebug() << __FILE__ << " " << __LINE__ << " sketch pts=" << ct_pts3.size() << " ct_simPts3=" << ct_simPts3.size();
                qDebug() << __FILE__ << " " << __LINE__ << " sketch pts=" << ct_pts3.size() << " ct_uniPts3=" << ct_uniPts3.size();
                contour.clear();
                for (const Pnt3 &p : ct_uniPts3)
                {
                    contour.push_back(GLVec3(p.x(), p.y(), p.z()));
                }
                contour.push_back(contour.front());

                for (const Pnt3 &p : ct_uniPts3)
                {
                    spline.AddCtrlPoint(p);
                }
                vector<vector<Pnt3>> sketchPolyLines;
                vector<Pnt3> sketchPolyLine;
                vector<vector<GLVec3>> gl_splines;
                CSketchManager::ConvertSplineToPolyLines(spline, sketchPolyLines);
                CSketchManager::GetPolyLine(sketchPolyLines, sketchPolyLine);

                gl_splines.resize(sketchPolyLines.size());
                int pts_count = 0;
                for (int i = 0; i < sketchPolyLines.size(); i++)
                {
                    for (const Pnt3 &p : sketchPolyLines[i])
                    {
                        gl_splines[i].push_back(GLVec3(p.x(), p.y(), p.z()));
                        pts_count++;
                    }
                }
                qDebug() << __FILE__ << " " << __LINE__ << " splines pts=" << pts_count << " spline=" << sketchPolyLine.size();
                m_curView->m_sketchline.clear();
#endif
                repaint();
            }
        }
        else if (e->key() == Qt::Key_L && e->modifiers() & Qt::ControlModifier)
        {
            QString dirName = QFileDialog::getExistingDirectory(this, tr("select directory"));
            if (!dirName.isEmpty())
            {
                m_curView->load(dirName);
                m_opMode = OPMODE_DRAW;
                repaint();
            }
        }
        else if (e->key() == Qt::Key_S && e->modifiers() & Qt::ControlModifier)
        {
            QString dirName = QFileDialog::getExistingDirectory(this, tr("select directory"));
            if (!dirName.isEmpty())
            {
                m_curView->save(dirName.toStdString());
            }
        }

        //-------------------------- Multiple Selection    --------------------------------------------
        if (e->key() == Qt::Key_A && e->modifiers() & Qt::ControlModifier)
        {
            m_curView->pick_all_subtoys();
            repaint_all();
        }
        else if (e->key() == Qt::Key_A && e->modifiers() & Qt::AltModifier)
        {
            m_curView->unpick_all_subtoys();
            repaint_all();
        }

        //-------------------------- ToDO: Draw line symetrically --------------------------------------------
        if (e->key() == Qt::Key_Space && e->modifiers() & Qt::ControlModifier)
        {
            qDebug() << "switch to OPMODE_DRAW_SYM mode";
            m_opMode = OPMODE_DRAW_SYM;
            SymmetryAxisDialog *symAxisDlg = new SymmetryAxisDialog(this);
            symAxisDlg->show();
        }
        //-------------------------- Symmetrize    --------------------------------------------
        if (e->key() == Qt::Key_M && e->modifiers() & Qt::ControlModifier)
        {
            if (m_opMode == OPMODE_SELECT_TOY ||
                m_opMode == OPMODE_EDIT_BONE)
            {
                qDebug() << __FILE__ << " " << __LINE__ << " Symmetrize...";
                SymetryDialog *symDlg = new SymetryDialog(this);
                symDlg->show();
            }
        }

        //-------------------------- Delete, Split, Connect Bones--------------------------------------------
        if (m_opMode == OPMODE_EDIT_BONE)
        {
            if (e->key() == Qt::Key_P && e->modifiers() & Qt::ControlModifier)
            {
                m_curView->setBoneParent();
            }
            else if (e->key() == Qt::Key_S && e->modifiers() & Qt::ControlModifier)
            {
                m_curView->splitBone();
            }
            else if (e->key() == Qt::Key_M && e->modifiers() & Qt::ControlModifier)
            {
                m_curView->mergeBone();
            }
        }
        //-------------------------- Switch view------------------------------------------------------
        if (e->modifiers() & Qt::KeypadModifier)
        {
            if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
            {
                m_opMode = OPMODE_SELECT_TOY;
            }
            if (e->key() == Qt::Key_1 && (e->modifiers() & Qt::ControlModifier))
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............back view";
                m_viewMode = VIEW_BACK;
                m_eye = QVector3D(0, 0, -m_camerapos - 300);
                m_up = QVector3D(0, 1, 0);
            }
            else if (e->key() == Qt::Key_1)
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............front view";
                m_viewMode = VIEW_FRONT;
                m_eye = QVector3D(0, 0, m_camerapos + 300);
                m_up = QVector3D(0, 1, 0);
            }
            else if (e->key() == Qt::Key_3 && (e->modifiers() & Qt::ControlModifier))
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............left view";
                m_viewMode = VIEW_LEFT;
                m_eye = QVector3D(-m_camerapos - 300, 0, 0);
                m_up = QVector3D(0, 1, 0);
            }
            else if (e->key() == Qt::Key_3)
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............right view";
                m_viewMode = VIEW_RIGHT;
                m_eye = QVector3D(m_camerapos + 300, 0, 0);
                m_up = QVector3D(0, 1, 0);
            }
            else if (e->key() == Qt::Key_7 && (e->modifiers() & Qt::ControlModifier))
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............bottom view";
                m_viewMode = VIEW_BOTTOM;
                m_eye = QVector3D(0, -m_camerapos - 300, 0);
                m_up = QVector3D(0, 0, 1);
            }
            else if (e->key() == Qt::Key_7)
            {
                qDebug() << __FILE__ << " " << __LINE__ << " ..............top view";
                m_viewMode = VIEW_TOP;
                m_eye = QVector3D(0, m_camerapos + 300, 0);
                m_up = QVector3D(0, 0, -1);
            }
            m_viewMatrix.setToIdentity();
            m_viewMatrix.lookAt(m_eye, m_center, m_up);
            update_bgImg();
            update_mvp();
        }

        //-------------------------- Switch Operation Mode-------------------------------------------------------------------
        // painting
        if ((e->modifiers() & Qt::ControlModifier) && e->key() == Qt::Key_B) // use brush to draw color lines
        {
            m_opMode = OPMODE_TEXTURE_PAINT;
            qDebug() << __FILE__ << " " << __LINE__ << " switch to OPMODE_TEXTURE_PAINT";
        }
        else if ((e->modifiers() & Qt::ControlModifier) && e->key() == Qt::Key_P) // copy part of background image onto model
        {
            m_opMode = OPMODE_TEXPAINT_IMAGE;
            qDebug() << __FILE__ << " " << __LINE__ << " switch to OPMODE_TEXPAINT_IMAGE";
        }
        else if ((e->modifiers() & Qt::ControlModifier) && e->key() == Qt::Key_F) // fill color
        {
            m_opMode = OPMODE_COLOR_FILL;
            qDebug() << __FILE__ << " " << __LINE__ << " switch to OPMODE_COLOR_FILL";
        }
        else if ((e->modifiers() & Qt::AltModifier) && e->key() == Qt::Key_F) // get color
        {
            m_opMode = OPMODE_GET_COLOR;
            qDebug() << __FILE__ << " " << __LINE__ << " switch to OPMODE_GET_COLOR";
        }
        if (e->key() == Qt::Key_E && e->modifiers() & Qt::ControlModifier)
        {
            qDebug() << "switch to OPMODE_EDIT_POINT_HANDLE mode";
            m_opMode = OperationMode::OPMODE_EDIT_POINT_HANDLE;

            repaint();
        }
        else if (e->key() == Qt::Key_D && e->modifiers() & Qt::ControlModifier)
        {
            m_curView->m_toy->initialize_auto_dof1();
        }
    }
}

void EasyGL::keyReleaseEvent(QKeyEvent *)
{
}

void EasyGL::initializeGL()
{
    // Set OpenGL 3.3, core profile
    QSurfaceFormat format = context()->format();
    format.setDepthBufferSize(24);
    // format.setStencilBufferSize(8);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);

    // format.setSamples(16);
    // format.setSwapInterval(0);
    QSurfaceFormat::setDefaultFormat(format);

    connect(context(), &QOpenGLContext::aboutToBeDestroyed,
            this, &EasyGL::contextIsDestroyed);
    Q_DEBUG << " initializeGL " << this;
    initializeOpenGLFunctions();
    makeCurrent();
    // Q_EMIT doneInitGL(this);
    // glEnable(GL_MULTISAMPLE);
    // Set clear color
    // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void EasyGL::resizeGL(int w, int h)
{
    // Set proj matrix
    float aspect = (float)width() / (float)height();
    m_projMatrix.setToIdentity();
    m_projMatrix.ortho(-m_camerapos, m_camerapos, -m_camerapos / aspect, m_camerapos / aspect, -10000.f, 10000.f);

    m_trackBall.resizeViewport(w, h);
    update_mvp();
}

void EasyGL::update_mvp()
{
    m_mvp = m_projMatrix * m_viewMatrix * m_worldMatrix;
    m_mv = m_viewMatrix * m_worldMatrix;

    // qDebug() << __FILE__ << " " << __LINE__ << " viewMatrix:\n";
    // qDebug() << m_viewMatrix;
    // qDebug() << __FILE__ << " " << __LINE__ << " normalMatrix1:\n";
    // qDebug() << m_viewMatrix.normalMatrix();
    // qDebug() << __FILE__ << " " << __LINE__ << " normalMatrix2:\n";
    // qDebug() << QMatrix4x4(m_viewMatrix.normalMatrix());
    // qDebug() << __FILE__ << " " << __LINE__ << " normalMatrix3:\n";
    // qDebug() << m_mv.normalMatrix();
    // qDebug() << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    repaint();
}

void EasyGL::paintGL()
{
    draw();
    if (++m_fpsCount == m_maxCount)
    {
        int timeElapsed = m_fpsTime.restart(); // million seconds
        m_fps = 1000.0 * m_maxCount / timeElapsed;
        m_fpsString = tr("%1fps", "Frames per seconds").arg(m_fps);
        m_fpsCount = 0;
        // qDebug() << m_fpsString;
    }
    displayFPS();
}

void EasyGL::displayFPS()
{
    drawText(10,
             int(1.5 * ((QApplication::font().pixelSize() > 0)
                            ? QApplication::font().pixelSize()
                            : QApplication::font().pointSize())),
             m_fpsString);
}

void EasyGL::drawText(int x, int y, const QString &text, const QFont &font)
{

    QColor fontColor = QColor(0, 0,
                              0, 255);

    // Render text
    QPainter painter(this);
    painter.setPen(fontColor);
    painter.setFont(font);
    painter.drawText(x, y, text);
    painter.end();
}

void EasyGL::draw()
{

    glClearColor(m_bgColor.redF(), m_bgColor.greenF(), m_bgColor.blueF(), m_bgColor.alphaF());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setDepthState();
    setTransparentState();

    //-------------------- Draw Opaque Item-----------------
    if (is_capture_contour)
    {
        render_single_submesh3d();
        is_capture_contour = false;
        return;
    }
    if (!m_bgImg.isNull() && isDrawBgImg)
    {
        draw_bgImg();
    }

    if (m_opMode == OPMODE_DRAW ||
        m_opMode == OPMODE_DRAW_SYM ||
        m_opMode == OPMODE_MOVE ||
        m_opMode == OPMODE_ROTATE ||
        m_opMode == OPMODE_SCALE ||
        m_opMode == OPMODE_MODIFY_SKETCH_LINES ||
        (m_opMode == OPMODE_EDIT_BONE && is2DMode))
    {
        glEnable(GL_DEPTH_TEST);

        // contour lines
        if (m_opMode == OPMODE_DRAW ||
            m_opMode == OPMODE_DRAW_SYM ||
            m_opMode == OPMODE_MOVE ||
            m_opMode == OPMODE_ROTATE ||
            m_opMode == OPMODE_SCALE)
        {
            m_curView->draw_subContours(this);
            m_curView->draw_line(this); // Notice: sketch_line should be drawn lastly!!!
        }
        else if (m_opMode == OPMODE_MODIFY_SKETCH_LINES)
        {
            if (m_curView->getCurToy()->m_curMesh3D.number_of_vertices() == 0) // create initial subtoy
            {
                m_curView->draw_subContours(this);
                m_curView->draw_subCtrlPts(this);
            }
            else
            {
                m_curView->drawDefContour(this);
            }
        }

        setDepthState();
    }

    //-------------------- Draw Transparent Item-----------------
    if (isPolygonMode)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    if (is2DMode)
    {
        // if (m_opMode == OPMODE_DRAW ||
        //     m_opMode == OPMODE_MOVE ||
        //     m_opMode == OPMODE_ROTATE)
        // {
        //     m_curView->draw_subMeshCoarse(this);
        //     // m_curView->draw_chordal_axis(this);
        // }

        if (m_opMode == OPMODE_DRAW ||
            m_opMode == OPMODE_DRAW_SYM ||
            m_opMode == OPMODE_SELECT_TOY ||
            m_opMode == OPMODE_SELECT_PARENT_TOY ||
            m_opMode == OPMODE_MOVE ||
            m_opMode == OPMODE_ROTATE ||
            m_opMode == OPMODE_SCALE ||
            // m_opMode == OPMODE_MODIFY_SKETCH_LINES || //ToDO
            m_opMode == OPMODE_EDIT_BONE ||
            m_opMode == OPMODE_EDIT_POINT_HANDLE)
            m_curView->draw_subMeshFine(this);

        if (m_opMode == OPMODE_BBW ||
            m_opMode == OPMODE_BBW_DEFORM)
        {
            m_curView->draw_subMeshDeform(this);
        }
    }
    else
    {
        if (m_opMode == OPMODE_DRAW ||
            m_opMode == OPMODE_DRAW_SYM ||
            m_opMode == OPMODE_SELECT_TOY ||
            m_opMode == OPMODE_SELECT_PARENT_TOY ||
            m_opMode == OPMODE_MOVE ||
            m_opMode == OPMODE_ROTATE ||
            m_opMode == OPMODE_SCALE ||
            m_opMode == OPMODE_MODIFY_SKETCH_LINES ||
            m_opMode == OPMODE_EDIT_BONE ||
            m_opMode == OPMODE_EDIT_POINT_HANDLE)
        {
            m_curView->draw_subMesh3D(this);
        }

        if (m_opMode == OPMODE_BBW ||
            m_opMode == OPMODE_BBW_DEFORM)
        {
            m_curView->draw_mesh3D(this);
        }
    }

    if (isDrawSkel)
    {
        glEnable(GL_DEPTH_TEST);
        if (m_opMode == OPMODE_DRAW ||
            m_opMode == OPMODE_DRAW_SYM ||
            m_opMode == OPMODE_SELECT_TOY ||
            m_opMode == OPMODE_SELECT_PARENT_TOY ||
            m_opMode == OPMODE_MOVE ||
            m_opMode == OPMODE_ROTATE ||
            m_opMode == OPMODE_SCALE ||
            m_opMode == OPMODE_EDIT_BONE ||
            m_opMode == OPMODE_EDIT_POINT_HANDLE)
        {

            m_curView->drawRestBone(this);
            if (m_opMode == OPMODE_ROTATE)
                m_curView->draw_arcBall(this);
        }
        else if (m_opMode == OPMODE_BBW ||
                 m_opMode == OPMODE_BBW_DEFORM)
        {
            m_curView->drawBone(this);
        }

        setDepthState();
    }

    if (m_isDrawPlane)
    {
        glEnable(GL_DEPTH_TEST);
        m_curView->draw_plane(this);
        setDepthState();
    }

    //glReadPixels(m_cursor.x(), height() - 1 - m_cursor.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &m_depth); //ToDO: add GetDepth()
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void EasyGL::setDepthState()
{
    if (isDepthTest)
    {
        glEnable(GL_DEPTH_TEST);
    }
    else
    {
        glDisable(GL_DEPTH_TEST);
    }
}

void EasyGL::setTransparentState()
{
    if (isTransparentMode)
    {
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    else
    {
        glDisable(GL_BLEND);
    }
}

void EasyGL::takeSnapshot()
{
    vector<unsigned short> imgBuffer(height() * width() * 4);
    glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_SHORT, &imgBuffer.front());

    vector<vector<int>> mask;
    vector<vector<double>> depth;

    mask.assign(height(), vector<int>(width(), false));
    depth.assign(height(), vector<double>(width(), 1.0));

    MapsData::flipVertical(imgBuffer, imgBuffer, width(), height(), 4);
    int offset = 0;
    for (int row = 0; row < height(); row++)
    {
        for (int col = 0; col < width(); col++)
        {
            unsigned short r = imgBuffer[offset];
            unsigned short g = imgBuffer[offset + 1];
            unsigned short b = imgBuffer[offset + 2];
            unsigned short a = imgBuffer[offset + 3];
            double x = r / (double)32768 - 1.0;
            double y = g / (double)32768 - 1.0;
            double z = b / (double)32768 - 1.0;
            double d = a / (double)32768 - 1.0;
            // vec3d n(x, y, z);
            if (d < 0.9)
            {
                mask[row][col] = true;
                depth[row][col] = d;
                // normals[row][col] = n;
            }
            offset += 4;
        }
    }
    MapsData::visualizeDepth("output/depth.png", depth);
    MapsData::visualizeMask("output/mask.png", mask);
}

void EasyGL::draw_bgImg()
{
    imgContainer.draw(this);
}

void EasyGL::update_bgImg()
{
    compute_bgVerts();
    imgContainer.update(this);
    repaint();
}

void EasyGL::compute_texScl()
{
    float x_scl = (float)width() / (float)bg_texture->width();
    float y_scl = (float)height() / (float)bg_texture->height();
    m_texScl = x_scl < y_scl ? x_scl : y_scl;
    m_texScl *= 0.9;
    m_texHalfW = (bg_texture->width() * m_texScl * 0.5);
    m_texHalfH = (bg_texture->height() * m_texScl * 0.5);
}

void EasyGL::compute_bgVerts()
{
    if (!isDrawBgImg || !bg_texture->isCreated())
        return;

    using namespace Eigen;

    bg_verts.resize(4);
    // float z = -m_camerapos + 1;
    float z = 0.5; // since screen z is in [0,0.16] (near, far)

    int x_cent = width() / 2;
    int y_cent = height() / 2;
    compute_texScl();
    float half_w = m_texHalfW;
    float half_h = m_texHalfH;

    Pnt3 v0 = Cvt2Dto3DIWorld(x_cent - half_w, y_cent - half_h, z);
    bg_verts[0].position = (GLVec3(v0) + m_bgCent) * m_bgScl;
    bg_verts[0].tex_coord = {0.0, 0.0};

    Pnt3 v1 = Cvt2Dto3DIWorld(x_cent + half_w, y_cent - half_h, z);
    bg_verts[1].position = (GLVec3(v1) + m_bgCent) * m_bgScl;
    bg_verts[1].tex_coord = {1.0, 0.0};

    Pnt3 v2 = Cvt2Dto3DIWorld(x_cent - half_w, y_cent + half_h, z);
    bg_verts[2].position = (GLVec3(v2) + m_bgCent) * m_bgScl;
    bg_verts[2].tex_coord = {0.0, 1.0};

    Pnt3 v3 = Cvt2Dto3DIWorld(x_cent + half_w, y_cent + half_h, z);
    bg_verts[3].position = (GLVec3(v3) + m_bgCent) * m_bgScl;
    bg_verts[3].tex_coord = {1.0, 1.0};
}

void EasyGL::set_bgImg(const QImage &bgImg)
{
    // https://code.woboq.org/qt5/qtbase/src/gui/opengl/qopengltexture.cpp.html#_ZN14QOpenGLTexture7setDataERK6QImageNS_16MipMapGenerationE
    bg_texture->destroy();
    bg_texture->create();
    bg_texture->setSize(m_bgImg.width(), m_bgImg.height(), m_bgImg.depth());
    bg_texture->setData(m_bgImg);

    imgContainer.setTexture(bg_texture);
    compute_bgVerts();
    imgContainer.update(this);
}

void EasyGL::capture_contour()
{
    is_capture_contour = true;
    qDebug() << __FILE__ << " " << __LINE__ << " capture contour successfully";
}

void EasyGL::render_single_submesh3d()
{
    // set contour capture state
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    // draw selected mesh
    drawMeshWithSingleColor = true;
    m_curView->drawSelectedSubMesh3D(this);

    vector<unsigned short> imgBuffer(height() * width() * 4);
    glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_SHORT, &imgBuffer.front());

    m_depthImg.assign(height(), vector<double>(width(), 1.0));
    m_contourImg = cv::Mat::zeros(height(), width(), CV_8UC1);
    MapsData::flipVertical(imgBuffer, imgBuffer, width(), height(), 4);
    int offset = 0;
    double z_near = GetNearSZ();
    double z_far = GetFarSZ();
    for (int row = 0; row < height(); row++)
    {
        for (int col = 0; col < width(); col++)
        {
            unsigned short r = imgBuffer[offset];
            unsigned short g = imgBuffer[offset + 1];
            unsigned short b = imgBuffer[offset + 2];
            unsigned short a = imgBuffer[offset + 3];

            double d = a / (double)65535;

            if (r < 20 && d <= z_far && d >= z_near)
            {
                m_contourImg.at<unsigned char>(row, col) = 255;
                m_depthImg[row][col] = d;
            }

            offset += 4;
        }
    }
    MapsData::visualizeDepth("output/depth.png", m_depthImg); // ToDO: remove
    cv::imwrite("output/mask.png", m_contourImg);             // ToDO: remove

    // QImage img = grabFramebuffer();
    // img.save("output/contour.jpg"); //ToDO: remove
    // qimage_to_mat(img, m_contourImg);
    // recover normal rendering state
    drawMeshWithSingleColor = false;
}

void EasyGL::screenshot()
{
    QString filename = "screenshot/" + QDateTime::currentDateTime().toString("yyyyMMddHHmmsszzz") + QString(".png");
    QImage image = grabFramebuffer();
    image.save(filename);
    M_DEBUG << "Save screen shot to " << filename.toStdString() << std::endl;
}