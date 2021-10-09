#pragma once

#include <QMainWindow>
#include "Viewer_interface.h"

class Three
{
public:
    Three();
    virtual ~Three() {}

    static QMainWindow *mainWindow();
    static Viewer_interface *mainViewer();
    static Viewer_interface *currentViewer();
    static void setCurrentViewer(Viewer_interface *viewer);
    static Viewer_interface *activeViewer();

protected:
    static QMainWindow *s_mainwindow;
    static Viewer_interface *s_mainviewer;
    static Viewer_interface *s_currentviewer;
    static Three *s_three;
};