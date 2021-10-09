#include <QPushButton>

#include <core/common.h>

#include "SymmetryAxisDialog.h"
#include "ui_SymmetryAxisDialog.h"
#include "EasyGL.h"
#include "mainwindow.h"

SymmetryAxisDialog::SymmetryAxisDialog(QWidget *parent) : QDialog(parent),
                                                          ui(new Ui::SymmetryAxisDialog)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle("Draw Sketchline Symmetrically");

    gl = qobject_cast<EasyGL *>(parent);
    m_curView = gl->m_curView;

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Ok), &QPushButton::clicked,
            [this]()
            {
                if (ui->radioBtnX->isChecked())
                {
                    M_DEBUG << "SYM_AXIS_X";
                    m_curView->m_sym_sketch.set_sym_axis(SYM_AXIS_X);
                }
                else if (ui->radioBtnY->isChecked())
                {
                    M_DEBUG << "SYM_AXIS_X";
                    m_curView->m_sym_sketch.set_sym_axis(SYM_AXIS_Y);
                }
                this->close();
            });

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Cancel), &QPushButton::clicked,
            [this]()
            {
                this->close();
            });
}