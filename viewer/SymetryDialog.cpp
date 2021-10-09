#include <QPushButton>

#include <core/common.h>

#include "SymetryDialog.h"
#include "ui_SymetryDialog.h"
#include "EasyGL.h"
#include "mainwindow.h"

SymetryDialog::SymetryDialog(QWidget *parent) : QDialog(parent),
                                                ui(new Ui::SymetryDialog)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose);

    gl = qobject_cast<EasyGL *>(parent);
    m_curView = gl->m_curView;

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Ok), &QPushButton::clicked,
            [this]()
            {
                SymetryPlane symetry_plane;
                if (ui->radioBtnXY->isChecked())
                {
                    qDebug() << __FILE__ << " " << __LINE__ << " SYM_XY_PLANE";
                    symetry_plane = SYM_XY_PLANE;
                }
                else if (ui->radioBtnYZ->isChecked())
                {
                    qDebug() << __FILE__ << " " << __LINE__ << " SYM_YZ_PLANE";
                    symetry_plane = SYM_YZ_PLANE;
                }

                m_curView->add_symetry_toy(gl, symetry_plane, ui->checkBoxWstParent->isChecked());
                this->close();
            });

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Cancel), &QPushButton::clicked,
            [this]()
            {
                this->close();
            });
}