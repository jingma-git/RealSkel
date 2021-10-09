#pragma once
#include <QMap>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QColor>
#include <vector>

#include "GLData.h"

// design principles: see Effective C++ item7, item34, item36
// item7: declare virtual deconstructor for polymorphic base
// item34: differentiate between inheritance of interface and inheritance of implementation
// item35: never redefine a non-virtual function (because non-virtual memfunc means an inheritance of invariant implmentation + interface)
// however, the encapsulation is bad see Effective C++ item22: declare data members private

class EasyGL;

class PrimitiveContainer
{
public:
    // dynamic variant behaviors
    virtual ~PrimitiveContainer();
    virtual void removeViewer(EasyGL *gl);

    virtual void init(EasyGL *gl){};
    virtual void bind(EasyGL *gl){};
    virtual void draw(EasyGL *gl){};

    // invariant behaviors, provide mandatory implementation
    void setColor(const QColor &color_) { color = color_; }
    void update(EasyGL *gl) { is_bind[gl] = false; }

protected:
    QMap<EasyGL *, QOpenGLVertexArrayObject *> vaos;
    QMap<EasyGL *, QOpenGLBuffer *> vbos;
    QMap<EasyGL *, bool> is_init;
    QMap<EasyGL *, bool> is_bind;
    QColor color;
};

class PointContainer : public PrimitiveContainer
{
public:
    virtual ~PointContainer();
    virtual void removeViewer(EasyGL *gl);

    void init(EasyGL *gl);
    void bind(EasyGL *gl);
    void draw(EasyGL *gl);

    void setPointPtr(std::vector<GLVec3> *point_ptr_) { point_ptr = point_ptr_; }
    void setSelectedIdxs(const vector<int> &sel_idxs)
    {
        selected_idxs.clear();
        selected_idxs = sel_idxs;
    }
    void setPointColors(const vector<GLVec3> &colors_)
    {
        colors.clear();
        colors = colors_;
        isColorsSet = true;
    }
    void setPointSize(float point_size_) { point_size = point_size_; }
    void setSelected(bool selected_) { selected = selected_; } //ToDO: remove

private:
    QMap<EasyGL *, QOpenGLBuffer *> selected_vbos;
    QMap<EasyGL *, QOpenGLBuffer *> color_vbos;

    std::vector<GLVec3> *point_ptr = nullptr;
    std::vector<GLVec3> colors;
    std::vector<GLfloat> selected_data;
    bool isColorsSet = false;

    std::vector<int> selected_idxs;
    float point_size = 3.0;
    bool selected = false; //ToDO: remove
};

class EdgeContainer : public PrimitiveContainer
{
public:
    void init(EasyGL *gl);
    void bind(EasyGL *gl);
    void draw(EasyGL *gl);
    void setEdgePtr(std::vector<GLVec3> *edge_ptr_) { edge_ptr = edge_ptr_; }
    void setEdgeWidth(float edge_width_) { edge_width = edge_width_; }

    // whether to draw solid line
    void setDrawLines(bool is_draw_lines_) { is_draw_lines = is_draw_lines_; }

private:
    std::vector<GLVec3> *edge_ptr = nullptr;
    float edge_width = 1.0;
    bool is_draw_lines = false;
};

class TriangleContainer : public PrimitiveContainer
{
public:
    void removeViewer(EasyGL *gl);
    virtual ~TriangleContainer();
    void update(EasyGL *gl);
    void init(EasyGL *gl);
    void bind(EasyGL *gl);
    void draw(EasyGL *gl);
    void setTexture(EasyGL *gl, QOpenGLTexture *texture)
    {
        mesh_texture = texture;
        is_bind_texture[gl] = false;
    }

public:
    std::vector<unsigned int> *idx_data = nullptr;
    std::vector<GLVec3> *vertices = nullptr;
    std::vector<GLVec3> *v_normals = nullptr;
    std::vector<GLVec3> *v_colors = nullptr;
    std::vector<GLVec3> *v_skinning_colors = nullptr;
    std::vector<GLVec2> *v_texCoords = nullptr;
    QOpenGLTexture *mesh_texture = nullptr;

private:
    QMap<EasyGL *, QOpenGLBuffer *> pos_vbos;
    QMap<EasyGL *, QOpenGLBuffer *> normal_vbos;
    QMap<EasyGL *, QOpenGLBuffer *> color_vbos;
    QMap<EasyGL *, QOpenGLBuffer *> texCoord_vbos;
    QMap<EasyGL *, QOpenGLBuffer *> mesh_ebos;
    QMap<EasyGL *, int> old_meshmodes;
    QMap<EasyGL *, bool> is_init_texture;
    QMap<EasyGL *, bool> is_bind_texture;
};

class ImageContainer : public PrimitiveContainer
{
public:
    void init(EasyGL *gl);
    void bind(EasyGL *gl);
    void draw(EasyGL *gl);
    void setTexture(QOpenGLTexture *texture_)
    {
        texture = texture_;
    }

    QOpenGLTexture *texture;
    std::vector<GLVertex3> *uv_ptr;
};
