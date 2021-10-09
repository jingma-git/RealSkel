#include <QOpenGLShaderProgram>

#include "containers.h"
#include "EasyGL.h"
#include "Three.h"

PrimitiveContainer::~PrimitiveContainer()
{
    QMap<EasyGL *, QOpenGLBuffer *>::iterator b_it = vbos.begin();
    for (; b_it != vbos.end(); ++b_it)
    {
        if (b_it.value())
            delete b_it.value();
    }

    QMap<EasyGL *, QOpenGLVertexArrayObject *>::iterator a_it = vaos.begin();
    for (; a_it != vaos.end(); ++a_it)
    {
        if (a_it.value())
            delete a_it.value();
    }
}

void PrimitiveContainer::removeViewer(EasyGL *gl)
{
    // qDebug() << __FILE__ << " " << __LINE__ << " vaos=" << vaos.size();
    is_init[gl] = false;
    is_bind[gl] = false;
    delete vbos[gl];
    vbos.remove(gl);
    delete vaos[gl];
    vaos.remove(gl); // https://stackoverflow.com/questions/799314/difference-between-erase-and-remove
}

void PointContainer::removeViewer(EasyGL *gl)
{
    is_init[gl] = false;
    is_bind[gl] = false;

    delete vbos[gl];
    vbos.remove(gl);
    delete selected_vbos[gl];
    selected_vbos.remove(gl);
    delete color_vbos[gl];
    color_vbos.remove(gl);

    delete vaos[gl];
    vaos.remove(gl);
}

PointContainer::~PointContainer()
{
    QMap<EasyGL *, QOpenGLBuffer *>::iterator sb_it = selected_vbos.begin();
    for (; sb_it != selected_vbos.end(); ++sb_it)
    {
        delete sb_it.value();
    }

    QMap<EasyGL *, QOpenGLBuffer *>::iterator cb_it = color_vbos.begin();
    for (; cb_it != color_vbos.end(); ++cb_it)
    {
        delete cb_it.value();
    }
}

void PointContainer::init(EasyGL *gl)
{
    QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_POINT);
    //------------------------------ Create VBO and VAO ------------------------------------------
    vaos[gl] = new QOpenGLVertexArrayObject;
    vbos[gl] = new QOpenGLBuffer;
    selected_vbos[gl] = new QOpenGLBuffer;
    color_vbos[gl] = new QOpenGLBuffer;

    vaos[gl]->create();
    vbos[gl]->create();
    selected_vbos[gl]->create();
    color_vbos[gl]->create();

    vaos[gl]->bind();
    vbos[gl]->bind(); // VBO: store vertex data(attribute)

    program->enableAttributeArray("aPos");
    program->setAttributeBuffer("aPos",          // attribute name
                                GL_FLOAT,        // type
                                0,               // offset
                                3,               // tuple size
                                sizeof(GLVec3)); // stride
    vbos[gl]->release();

    selected_vbos[gl]->bind();
    program->enableAttributeArray("aSelected");
    program->setAttributeBuffer("aSelected",      // attribute name
                                GL_FLOAT,         // type
                                0,                // offset
                                1,                // tuple size
                                sizeof(GLfloat)); // stride

    selected_vbos[gl]->release();

    color_vbos[gl]->bind();
    program->enableAttributeArray("aColor");
    program->setAttributeBuffer("aColor", // attribute name
                                GL_FLOAT, // type
                                0,        // offset
                                3,        // tuple size
                                0);       // stride

    color_vbos[gl]->release();

    vaos[gl]->release();
    is_init[gl] = true;
}

void PointContainer::bind(EasyGL *gl)
{
    vbos[gl]->bind();
    vbos[gl]->allocate(point_ptr->data(), point_ptr->size() * sizeof(GLVec3));
    vbos[gl]->release();

    int num_points = point_ptr->size();

    selected_data.clear();
    selected_data.resize(num_points, 0);
    // cout << __FILE__ << " " << __LINE__ << " selected_idxs=" << selected_idxs.size() << endl;
    if (selected_idxs.size() > 0)
    {
        for (const int &i : selected_idxs)
        {
            // cout << __FILE__ << " " << __LINE__ << " " << i << endl;
            selected_data[i] = 1;
        }
    }

    selected_vbos[gl]->bind();
    selected_vbos[gl]->allocate(selected_data.data(), selected_data.size() * sizeof(GLfloat));
    selected_vbos[gl]->release();

    if (!isColorsSet)
    {
        colors.resize(num_points, GLVec3(color.redF(), color.greenF(), color.blueF()));
    }
    else
    {
        assert(colors.size() == num_points && "num_points!=num_point_colors!");
    }

    color_vbos[gl]->bind();
    color_vbos[gl]->allocate(colors.data(), colors.size() * sizeof(GLVec3));
    color_vbos[gl]->release();

    is_bind[gl] = true;
}

void PointContainer::draw(EasyGL *gl)
{
    if (!is_init[gl])
    {
        init(gl);
    }

    if (!is_bind[gl])
    {
        bind(gl);
    }

    QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_POINT);

    program->bind();
    program->setUniformValue("mvp", gl->m_mvp);
    program->setUniformValue("uPointSize", point_size);
    program->setUniformValue("uColor", color);
    program->setUniformValue("uSelected", selected);

    vaos[gl]->bind();
    glDrawArrays(GL_POINTS, 0, point_ptr->size());
    vaos[gl]->release();
    program->release();
}

void EdgeContainer::init(EasyGL *gl)
{
    if (!is_init[gl])
    {
        // qDebug() << __FILE__ << " " << __LINE__ << " EdgeContainer::init " << is_init[gl];
        QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_LINE);

        vaos[gl] = new QOpenGLVertexArrayObject();
        vbos[gl] = new QOpenGLBuffer();
        vaos[gl]->create();
        vbos[gl]->create();

        vaos[gl]->bind();
        vbos[gl]->bind(); // VBO: store vertex data(attribute)

        program->enableAttributeArray("aPos");
        program->setAttributeBuffer("aPos",          // attribute name
                                    GL_FLOAT,        // type
                                    0,               // offset
                                    3,               // tuple size
                                    sizeof(GLVec3)); // stride
        vaos[gl]->release();
        vbos[gl]->release();
        is_init[gl] = true;
    }
}

void EdgeContainer::bind(EasyGL *gl)
{
    vbos[gl]->bind();
    vbos[gl]->allocate(edge_ptr->data(), edge_ptr->size() * sizeof(GLVec3));
    vbos[gl]->release();
    is_bind[gl] = true;
}

void EdgeContainer::draw(EasyGL *gl)
{
    if (!is_init[gl])
    {
        init(gl);
        // M_DEBUG << " EdgeContainer::draw init " << is_init[gl] << endl;
    }

    if (!is_bind[gl])
    {
        // M_DEBUG << " EdgeContainer::bind " << is_bind[gl] << " data_size=" << edge_ptr->size() << endl;
        bind(gl);
    }

    QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_LINE);
    program->bind();
    program->setUniformValue("mvp", gl->m_mvp);
    program->setUniformValue("uColor", color);

    QVector2D vp(gl->width(), gl->height());
    program->setUniformValue("viewport", vp);
    program->setUniformValue("width", edge_width);

    vaos[gl]->bind();
    if (is_draw_lines)
    {
        glDrawArrays(GL_LINES, 0, edge_ptr->size());
    }
    else
    {
        glDrawArrays(GL_LINE_STRIP, 0, edge_ptr->size());
    }

    vaos[gl]->release();
    program->release();
}

void TriangleContainer::removeViewer(EasyGL *gl)
{
    is_init[gl] = false;
    is_bind[gl] = false;
    is_init_texture[gl] = false;
    is_bind_texture[gl] = false;

    delete vbos[gl];
    vbos.remove(gl);

    delete pos_vbos[gl];
    pos_vbos.remove(gl);
    delete normal_vbos[gl];
    normal_vbos.remove(gl);
    delete color_vbos[gl];
    color_vbos.remove(gl);
    delete texCoord_vbos[gl];
    texCoord_vbos.remove(gl);
    delete mesh_ebos[gl];
    mesh_ebos.remove(gl);

    delete vaos[gl];
    vaos.remove(gl);
}

TriangleContainer::~TriangleContainer()
{
    QMap<EasyGL *, QOpenGLBuffer *>::iterator pos_it = pos_vbos.begin();
    for (; pos_it != pos_vbos.end(); ++pos_it)
    {
        if (pos_it.value())
            delete pos_it.value();
    }
    QMap<EasyGL *, QOpenGLBuffer *>::iterator normal_it = normal_vbos.begin();
    for (; normal_it != normal_vbos.end(); ++normal_it)
    {
        if (normal_it.value())
            delete normal_it.value();
    }
    QMap<EasyGL *, QOpenGLBuffer *>::iterator color_it = color_vbos.begin();
    for (; color_it != color_vbos.end(); ++color_it)
    {
        if (color_it.value())
            delete color_it.value();
    }
    QMap<EasyGL *, QOpenGLBuffer *>::iterator tex_it = texCoord_vbos.begin();
    for (; tex_it != texCoord_vbos.end(); ++tex_it)
    {
        if (tex_it.value())
            delete tex_it.value();
    }
    QMap<EasyGL *, QOpenGLBuffer *>::iterator mesh_it = mesh_ebos.begin();
    for (; mesh_it != mesh_ebos.end(); ++mesh_it)
    {
        if (mesh_it.value())
            delete mesh_it.value();
    }
}

void TriangleContainer::update(EasyGL *gl)
{
    PrimitiveContainer::update(gl);
    is_bind_texture[gl] = false;
}

void TriangleContainer::init(EasyGL *gl)
{
    QOpenGLShaderProgram *program;
    if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
    {
        program = gl->getShaderProgram(PROGRAM_MESH_WITH_TEXTURE);
        program->bind();
        program->setUniformValue("texSampler", 0);
        program->release();
    }
    else
    {
        program = gl->getShaderProgram(PROGRAM_MESH);
    }

    vaos[gl] = new QOpenGLVertexArrayObject;
    mesh_ebos[gl] = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    pos_vbos[gl] = new QOpenGLBuffer();
    normal_vbos[gl] = new QOpenGLBuffer();

    vaos[gl]->create();
    pos_vbos[gl]->create();
    normal_vbos[gl]->create();
    mesh_ebos[gl]->create();

    vaos[gl]->bind();

    pos_vbos[gl]->bind();
    program->enableAttributeArray("aPos");
    program->setAttributeBuffer("aPos",   // attribute name
                                GL_FLOAT, // type
                                0,        // offset
                                3,        // tuple size
                                0);       // stride

    pos_vbos[gl]->release(); //!!! important, once bind, must be released, otherwise, it will affect next vbo's binding

    normal_vbos[gl]->bind();
    program->enableAttributeArray("aNormal");
    program->setAttributeBuffer("aNormal", // attribute name
                                GL_FLOAT,  // type
                                0,         // offset
                                3,         // tuple size
                                0);        // stride

    normal_vbos[gl]->release();

    if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
    {
        texCoord_vbos[gl] = new QOpenGLBuffer();
        texCoord_vbos[gl]->create();
        texCoord_vbos[gl]->bind();
        program->enableAttributeArray("texCoord");
        program->setAttributeBuffer("texCoord", // attribute name
                                    GL_FLOAT,   // type
                                    0,          // offset
                                    2,          // tuple size
                                    0);         // stride

        texCoord_vbos[gl]->release();
        is_init_texture[gl] = true;
    }
    else
    {
        color_vbos[gl] = new QOpenGLBuffer();
        color_vbos[gl]->create();
        color_vbos[gl]->bind();
        program->enableAttributeArray("aColor");
        program->setAttributeBuffer("aColor", // attribute name
                                    GL_FLOAT, // type
                                    0,        // offset
                                    3,        // tuple size
                                    0);       // stride

        color_vbos[gl]->release();
        is_init[gl] = true;
    }
}

void TriangleContainer::bind(EasyGL *gl)
{

    pos_vbos[gl]->bind();
    pos_vbos[gl]->allocate(vertices->data(), sizeof(GLVec3) * vertices->size());
    pos_vbos[gl]->release();

    normal_vbos[gl]->bind();
    normal_vbos[gl]->allocate(v_normals->data(), sizeof(GLVec3) * v_normals->size());
    normal_vbos[gl]->release();

    if (mesh_texture == nullptr)
    {
        color_vbos[gl]->bind();
        // qDebug() << __FILE__ << " " << __LINE__ << " show_skinning_color=" << show_skinning_color << " v_skinning_colors=" << v_skinning_colors.size();
        if (gl->m_meshMode == MESH_MODE_SKINNING_COLORED && v_skinning_colors->size() > 0)
            color_vbos[gl]->allocate(v_skinning_colors->data(), sizeof(GLVec3) * v_skinning_colors->size());
        else
            color_vbos[gl]->allocate(v_colors->data(), sizeof(GLVec3) * v_colors->size());
        color_vbos[gl]->release();
    }
    else
    {
        if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
        {
            qDebug() << __FILE__ << " " << __LINE__ << " TriangleContainer::bind texture";
            assert(mesh_texture != nullptr && "Texture does not exist!");
            texCoord_vbos[gl]->bind();
            texCoord_vbos[gl]->allocate(v_texCoords->data(), sizeof(GLVec2) * v_texCoords->size());
            texCoord_vbos[gl]->release();
            is_bind_texture[gl] = true;
        }
    }

    is_bind[gl] = true;
}

void TriangleContainer::draw(EasyGL *gl)
{
    if (old_meshmodes[gl] != gl->m_meshMode)
    {
        update(gl);
        old_meshmodes[gl] = gl->m_meshMode;
    }

    if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
    {
        if (!is_init_texture[gl])
        {
            qDebug() << __FILE__ << " " << __LINE__ << " init_texture " << gl;
            init(gl);
        }
        if (!is_bind_texture[gl])
        {
            qDebug() << __FILE__ << " " << __LINE__ << " bind_texture " << gl;
            bind(gl);
        }
    }
    else
    {
        if (!is_init[gl])
        {
            // qDebug() << __FILE__ << " " << __LINE__ << " init mesh " << gl;
            init(gl);
        }
        if (!is_bind[gl])
        {
            // qDebug() << __FILE__ << " " << __LINE__ << " bind mesh " << gl;
            bind(gl);
        }
    }

    QOpenGLShaderProgram *program;
    if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
    {
        program = gl->getShaderProgram(PROGRAM_MESH_WITH_TEXTURE);
    }
    else
    {
        program = gl->getShaderProgram(PROGRAM_MESH);
    }

    program->bind();
    program->setUniformValue("mvp", gl->m_mvp);
    program->setUniformValue("mv", gl->m_mv);
    program->setUniformValue("normalMatrix", gl->m_mv.normalMatrix());
    program->setUniformValue("light_pos", gl->light_pos);
    program->setUniformValue("light_amb", gl->light_amb);
    program->setUniformValue("light_diff", gl->light_diff);
    program->setUniformValue("light_spec", gl->light_spec);
    program->setUniformValue("spec_power", gl->spec_power);
    program->setUniformValue("alpha", gl->alpha);
    program->setUniformValue("is_single_color", gl->drawMeshWithSingleColor);
    if (gl->m_meshMode == MESH_MODE_TEXTURE && mesh_texture != nullptr)
    {
        mesh_texture->bind();
    }

    vaos[gl]->bind();
    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
    mesh_ebos[gl]->bind();
    mesh_ebos[gl]->allocate(idx_data->data(), sizeof(unsigned int) * idx_data->size());
    glDrawElements(GL_TRIANGLES, idx_data->size(), GL_UNSIGNED_INT, 0);

    mesh_ebos[gl]->release();
    vaos[gl]->release();
    program->release();
}

void ImageContainer::init(EasyGL *gl)
{

    QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_BG);

    vaos[gl] = new QOpenGLVertexArrayObject();
    vbos[gl] = new QOpenGLBuffer();

    program->bind();
    program->setUniformValue("texSampler", 0);
    program->release();

    //------------------------------ Create VBO, EBO and VAO ------------------------------------------
    vaos[gl]->create();
    vbos[gl]->create();

    vaos[gl]->bind();
    vbos[gl]->bind();
    // vertex position
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",           // attribute name
                                GL_FLOAT,           // type
                                0,                  // offset
                                3,                  // tuple size
                                sizeof(GLVertex3)); // stride

    // texture position
    program->enableAttributeArray("texCoord");
    program->setAttributeBuffer("texCoord",                     // attribute name
                                GL_FLOAT,                       // type
                                offsetof(GLVertex3, tex_coord), // offset
                                2,                              // tuple size
                                sizeof(GLVertex3));             // stride

    assert(sizeof(float) * 5 == sizeof(GLVertex3));
    assert(3 * sizeof(float) == offsetof(GLVertex3, tex_coord));

    vbos[gl]->release();
    vaos[gl]->release();

    is_init[gl] = true;
}

void ImageContainer::bind(EasyGL *gl)
{
    vbos[gl]->bind();
    vbos[gl]->allocate(uv_ptr->data(), sizeof(GLVertex3) * uv_ptr->size());
    vbos[gl]->release();
    is_bind[gl] = false;
}

void ImageContainer::draw(EasyGL *gl)
{
    if (!is_init[gl])
    {
        init(gl);
    }
    if (!is_bind[gl])
    {
        bind(gl);
    }

    QOpenGLShaderProgram *program = gl->getShaderProgram(PROGRAM_BG);
    program->bind();
    program->setUniformValue("mvp", gl->m_mvp);
    texture->bind();
    vaos[gl]->bind();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    vaos[gl]->release();
    program->release();
}