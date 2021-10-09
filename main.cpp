#include <viewer/mainwindow.h>
#include <viewer/EasyGL.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    // Initialize application
    QApplication a(argc, argv);

    // // Set OpenGL 3.3, core profile
    // QSurfaceFormat format;
    // format.setDepthBufferSize(24);
    // // format.setStencilBufferSize(8);
    // format.setRenderableType(QSurfaceFormat::OpenGL);
    // format.setVersion(3, 3);
    // format.setProfile(QSurfaceFormat::CoreProfile);

    // // format.setSamples(16);
    // // format.setSwapInterval(0);
    // QSurfaceFormat::setDefaultFormat(format);

    MainWindow mw;
    mw.show();

    // Start event loop
    return a.exec();
}
