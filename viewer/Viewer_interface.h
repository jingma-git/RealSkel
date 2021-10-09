#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

class Viewer_interface : public QOpenGLWidget, public QOpenGLFunctions
{
public:
    Viewer_interface(QWidget *parent) : QOpenGLWidget(parent) {}
};