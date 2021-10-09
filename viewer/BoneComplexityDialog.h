#pragma once

#include <QDialog>
namespace Ui
{
    class BoneComplexityDialog;
}

class EasyGL;
class ToyView;
class XToy;

class BoneComplexityDialog : public QDialog
{
    Q_OBJECT
public:
    BoneComplexityDialog(QWidget *parent = nullptr);

private:
    void updateGlobalThresh();
    void updateMergeThresh();
    void updatePruneThresh();
    void updateCollapseThresh();

private Q_SLOTS:
    void skelTypeChanged(int index);
    void sliderBCValueChanged(int value);
    void sliderMergeValueChanged(int value);
    void sliderPruneValueChanged(int value);
    void sliderCollapseValueChanged(int value);

private:
    Ui::BoneComplexityDialog *ui;
    EasyGL *gl;
    ToyView *m_curView;
    XToy *m_toy;
};