#pragma once

#include <QDialog>
#include <vector>

namespace Ui
{
    class SetInfoDialog;
}

class EasyGL;
class ToyView;
class XSubToy;
class SetInfoDialog : public QDialog
{
    Q_OBJECT
public:
    enum SubToyOp
    {
        SubToyOP_ADD,
        SubToyOP_ADJUST_THICK,
        SubToyOP_SCALE
    };
    SetInfoDialog(QWidget *parent = nullptr);

private Q_SLOTS:
    void scaleSliderValueChanged(int value);
    void thickSliderValueChanged(int value);
    void depthSliderValueChanged(int value);

private:
    Ui::SetInfoDialog *ui;
    EasyGL *gl;
    ToyView *m_curView;
    XSubToy *subToy;
    std::vector<int> layers;
    int lastSclValue;
    double scl = 1.0;
    int op_mode = SubToyOP_ADD;
};