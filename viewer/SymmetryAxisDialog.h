#pragma once

#include <QDialog>
#include <vector>

namespace Ui
{
    class SymmetryAxisDialog;
}

class EasyGL;
class ToyView;
class SymmetryAxisDialog : public QDialog
{
    Q_OBJECT
public:
    SymmetryAxisDialog(QWidget *parent = nullptr);

private:
    Ui::SymmetryAxisDialog *ui;
    EasyGL *gl;
    ToyView *m_curView;
};