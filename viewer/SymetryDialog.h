#pragma once

#include <QDialog>
#include <vector>

namespace Ui
{
    class SymetryDialog;
}

class EasyGL;
class ToyView;
class SymetryDialog : public QDialog
{
    Q_OBJECT
public:
    SymetryDialog(QWidget *parent = nullptr);

private:
    Ui::SymetryDialog *ui;
    EasyGL *gl;
    ToyView *m_curView;
};