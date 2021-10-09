#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QMessageBox>
#include <QCloseEvent>
#include "EasyGL.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    setObjectName("mainwindow");
    Q_DEBUG << ": " << this;
    ui->setupUi(this);
    viewer_window = new SubViewer(ui->mdiArea, this, nullptr);
    viewer = viewer_window->viewer;
    viewer->setObjectName("mainViewer");
    toyview = viewer->m_curView;
    viewer_window->showMaximized();
    viewer_window->setWindowFlags(
        Qt::SubWindow | Qt::CustomizeWindowHint | Qt::WindowMaximizeButtonHint | Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
    viewer_window->setWindowTitle("Main Viewer");
    Three::s_mainwindow = this;
    Three::s_mainviewer = viewer;
    Three::s_currentviewer = viewer;
    // ToDO: setupviewer

    // painting
    color_widgets::ColorPalette palette;
    int palette_columns = 14;
    palette.setName("Palette");
    palette.setColumns(palette_columns);
    palette.appendColor(QColor(0, 0, 0));       // 0
    palette.appendColor(QColor(128, 128, 128)); // 1
    palette.appendColor(QColor(128, 0, 0));     // 2
    palette.appendColor(QColor(128, 128, 0));   // 3
    palette.appendColor(QColor(0, 128, 0));     // 4
    palette.appendColor(QColor(0, 128, 128));   // 5
    palette.appendColor(QColor(0, 0, 128));     // 6
    palette.appendColor(QColor(128, 0, 128));   // 7

    palette.appendColor(QColor(60, 179, 113)); // 8
    palette.appendColor(QColor(47, 64, 64));   // 9
    palette.appendColor(QColor(0, 191, 255));  // 10
    palette.appendColor(QColor(255, 165, 0));  // 11
    palette.appendColor(QColor(164, 42, 42));  // 12
    palette.appendColor(QColor(0, 255, 127));  // 13

    palette.appendColor(QColor(255, 255, 255)); // 14
    palette.appendColor(QColor(192, 192, 192)); // 15
    palette.appendColor(QColor(255, 0, 0));     // 16
    palette.appendColor(QColor(255, 255, 0));   // 17
    palette.appendColor(QColor(0, 255, 0));     // 18
    palette.appendColor(QColor(0, 255, 255));   // 19
    palette.appendColor(QColor(0, 0, 255));     // 20
    palette.appendColor(QColor(255, 0, 255));   // 21

    palette.appendColor(QColor(135, 206, 250)); // 22
    palette.appendColor(QColor(255, 192, 203)); // 23
    palette.appendColor(QColor(255, 127, 80));  // 24
    palette.appendColor(QColor(255, 215, 0));   // 25
    palette.appendColor(QColor(75, 0, 130));    // 26
    palette.appendColor(QColor(64, 224, 208));  // 27

    color_picker = new color_widgets::Swatch(this);
    ui->centralWidget->layout()->addWidget(color_picker);
    color_picker->setPalette(palette);
    color_picker->setFixedSize(350, 50);
    // connect(color_picker, SIGNAL(colorSelected(const QColor &)), toyview, SLOT(setPaintColor(const QColor &)));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete color_picker;
}

void MainWindow::on_actionAdd_Viewer_triggered()
{
    SubViewer *subviewer = new SubViewer(ui->mdiArea, this, viewer);
    EasyGL *viewer2 = subviewer->viewer;
    // connect removeViewer event among EasyGL and ToyView
    connect(viewer2, &EasyGL::contextIsDestroyed,
            this, [this, viewer2]()
            {
                toyview->removeViewer(viewer2);
                Q_DEBUG << "EasyGLPool().size()=" << EasyGL::EasyGLPool().size();
            });

    // setupviewer
    subviewer->show();
    ui->mdiArea->tileSubWindows();
    QPoint pos = viewer_window->pos();
    QSize size = viewer_window->size();

    viewer_window->move(subviewer->pos());
    viewer_window->resize(subviewer->size());
    subviewer->move(pos);
    subviewer->resize(size);
}

void MainWindow::on_actionSplit_Viewer_triggered()
{
    SubViewer *subviewer = new SubViewer(ui->mdiArea, this, viewer);
    EasyGL *viewer2 = subviewer->viewer;
    connect(viewer2, &EasyGL::contextIsDestroyed,
            this, [this, viewer2]()
            {
                toyview->removeViewer(viewer2);
                Q_DEBUG << "EasyGLPool().size()=" << EasyGL::EasyGLPool().size();
            });
    viewer2->is2DMode = false;
    // ToDO: setupviewer
    subviewer->show();
    ui->mdiArea->tileSubWindows();
    QPoint pos = viewer_window->pos();
    QSize size = viewer_window->size();

    // Q_DEBUG << " viewer_window" << viewer_window->viewer << " window pos=" << subviewer->pos() << " size=" << subviewer->size();
    viewer_window->move(subviewer->pos());
    viewer_window->resize(subviewer->size());
    subviewer->move(pos);
    subviewer->resize(size);
}

void MainWindow::on_actionSplit_to_Four_Viewers_triggered()
{
}

void MainWindow::on_actionRearange_Viewer_triggered()
{
    if (ui->mdiArea->subWindowList().size() == 1)
        ui->mdiArea->subWindowList().first()->showMaximized();
    else
    {
        ui->mdiArea->tileSubWindows();
        QMdiSubWindow *subviewer = qobject_cast<QMdiSubWindow *>(
            ui->mdiArea->childAt(ui->mdiArea->pos()));
        if (!subviewer) //should not happen but better safe than sorry
        {
            return;
        }
        QPoint pos = viewer_window->pos();
        QSize size = viewer_window->size();
        viewer_window->move(subviewer->pos());
        viewer_window->resize(subviewer->size());
        subviewer->move(pos);
        subviewer->resize(size);
    }
}

int SubViewer::idx = 0;

SubViewer::SubViewer(QWidget *parent, MainWindow *mw, EasyGL *mainviewer)
    : QMdiSubWindow(parent),
      mw(mw),
      is_main(false)
{
    if (mainviewer)
    {

        viewer = new EasyGL(this, mainviewer);
    }
    else
    {
        viewer = new EasyGL(this); // Add main viewer
        is_main = true;
    }
    setWidget(viewer);

    if (mainviewer)
    {
        setAttribute(Qt::WA_DeleteOnClose);
    }
    setObjectName(QString("SubViewer%1").arg(idx));
    ++idx;
    Q_DEBUG << " SubViewer::SubViewer" << this << " viewer=" << viewer;
}

SubViewer::~SubViewer()
{
    Q_DEBUG << "~SubViewer " << this;
    --idx;
    delete viewer;
}

void SubViewer::closeEvent(QCloseEvent *closeEvent)
{
    if (is_main)
    {
        QMessageBox::information(mw, "", "This is the main viewer. It cannot be closed.");
        closeEvent->ignore();
    }
    else
        QWidget::closeEvent(closeEvent);
}

void SubViewer::changeEvent(QEvent *event)
{
    QMdiSubWindow::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange)
    {
        if (isMaximized())
        {
            setWindowFlags(
                Qt::SubWindow | Qt::CustomizeWindowHint | Qt::WindowMaximizeButtonHint
                //| Qt::WindowSystemMenuHint
                | Qt::WindowTitleHint);
            viewer->update();
        }
        else
        {
            setWindowFlags(
                Qt::SubWindow | Qt::CustomizeWindowHint | Qt::WindowMaximizeButtonHint | Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
            // Q_DEBUG << " " << EasyGL::EasyGLPool().size();
            for (auto v : EasyGL::EasyGLPool())
            {
                if (v == nullptr)
                    continue;
                // Q_DEBUG << " " << v;
                v->update();
            }
        }
    }
}
