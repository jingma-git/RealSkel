#include "Three.h"
#include <QMdiArea>

QMainWindow *Three::s_mainwindow = NULL;
Viewer_interface *Three::s_mainviewer = NULL;
Viewer_interface *Three::s_currentviewer = NULL;
Three *Three::s_three = NULL;

Three::Three()
{
}

QMainWindow *Three::mainWindow()
{
    return s_mainwindow;
}

Viewer_interface *Three::mainViewer()
{
    return s_mainviewer;
}

Viewer_interface *Three::currentViewer()
{
    return s_currentviewer;
}

void Three::setCurrentViewer(Viewer_interface *viewer)
{
    s_currentviewer = viewer;
}

Viewer_interface *Three::activeViewer()
{
    QMdiArea *mdi = mainWindow()->findChild<QMdiArea *>();
    if (!mdi || !mdi->activeSubWindow())
    {
        return mainViewer();
    }

    return currentViewer();
}