#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMdiSubWindow>

#include "Three.h"
#include "color_palette_widget.h"

class ToyView;
class EasyGL;
class SubViewer;

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow, public Three
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private Q_SLOTS:
    void on_actionAdd_Viewer_triggered();
    void on_actionRearange_Viewer_triggered();
    void on_actionSplit_Viewer_triggered();
    void on_actionSplit_to_Four_Viewers_triggered();

public:
    Ui::MainWindow *ui;
    color_widgets::Swatch *color_picker;
    SubViewer *viewer_window;
    EasyGL *viewer;
    ToyView *toyview;
};

struct SubViewer : QMdiSubWindow
{
    Q_OBJECT
public:
    MainWindow *mw;
    EasyGL *viewer;
    SubViewer(QWidget *parent, MainWindow *mw, EasyGL *mainviewer);
    ~SubViewer();
    void closeEvent(QCloseEvent *closeEvent) Q_DECL_OVERRIDE;
    void changeEvent(QEvent *event) Q_DECL_OVERRIDE;

private:
    bool is_main;
    static int idx;
};

#endif // MAINWINDOW_H
