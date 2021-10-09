#include <QPushButton>

#include "SetInfoDialog.h"
#include "ui_SetInfoDialog.h"
#include "EasyGL.h"
#include "mainwindow.h"

SetInfoDialog::SetInfoDialog(QWidget *parent) : QDialog(parent),
                                                ui(new Ui::SetInfoDialog)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose);

    gl = qobject_cast<EasyGL *>(parent);
    m_curView = gl->m_curView;
    subToy = m_curView->getCurToy();

    SubViewer *subviewer = qobject_cast<SubViewer *>(gl->parentWidget());
    if (subviewer)
    {
        //获取父窗口geometry
        QRect rect = subviewer->geometry();
        //计算显示原点
        this->move(rect.x() + gl->pos().x() + gl->width() - this->width() / 2, rect.y() + gl->pos().y() + 70 + this->height() / 2);
    }

    // Layers/Z depth
    int defaultLayerIdx = 0;
    int i = 0;
    layers.resize((gl->m_maxLayer - gl->m_minLayer) + 1);
    for (int lIdx = gl->m_minLayer; lIdx <= gl->m_maxLayer; lIdx++)
    {
        ui->comboBoxToyLayer->addItem(QString("%1").arg(lIdx));
        if (subToy)
        {
            if (lIdx == subToy->m_layerID)
            {
                defaultLayerIdx = i;
            }
        }

        layers[i] = lIdx;
        i++;
    }
    ui->comboBoxToyLayer->setCurrentIndex(defaultLayerIdx);

    // thickness
    if (subToy)
    {
        // Q_DEBUG << " m_layerThick=" << subToy->m_layerThick;
        int thickCurValue = subToy->m_layerThick * 100;
        int thickMaxValue = thickCurValue * 5;
        ui->sliderThick->setMaximum(thickMaxValue);
        ui->labelThickMax->setText(QString::number(thickMaxValue / 100.0));
        ui->sliderThick->setValue(thickCurValue);
    }
    else
    {
        ui->sliderThick->setValue(100); // initial case [0, 500]
    }

    // scale
    lastSclValue = 500;
    ui->sliderScale->setValue(500); // initial case [0, 500]

    connect(ui->sliderDepth, &QSlider::valueChanged, this, &SetInfoDialog::depthSliderValueChanged);
    connect(ui->sliderThick, &QSlider::valueChanged, this, &SetInfoDialog::thickSliderValueChanged);
    connect(ui->sliderScale, &QSlider::valueChanged, this, &SetInfoDialog::scaleSliderValueChanged);

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Ok), &QPushButton::clicked,
            [this]()
            {
                if (subToy->m_curMesh3D.number_of_vertices() == 0)
                {
                    // lThick: [0, 5]
                    double lThick = ui->sliderThick->value() / 100.0;
                    int layer = this->layers[ui->comboBoxToyLayer->currentIndex()];
                    bool autoSkel = ui->checkBoxAutoSkel->isChecked();

                    gl->m_curLayer = layer;
                    // M_DEBUG << "gl=" << gl << " gl->m_curLayer=" << gl->m_curLayer << endl;
                    if (subToy)
                    {
                        subToy->m_layerID = layer;
                        subToy->m_autoSkel = autoSkel;
                        subToy->m_layerThick = lThick;
                        // M_DEBUG << " m_layerThick=" << subToy->m_layerThick
                        //         << " m_layerID=" << subToy->m_layerID
                        //         << " m_autoSkel=" << subToy->m_autoSkel << endl;
                    }
                    m_curView->addMeshes(gl); // ToDO: recalculate skeleton after changing submesh3d thickness, layerid,
                }
                m_curView->save("output");
                for (EasyGL *v : EasyGL::EasyGLPool())
                {
                    if (op_mode == SubToyOP_ADD)
                        v->m_opMode = OPMODE_DRAW;
                    v->m_isDrawPlane = false;
                    v->repaint();
                }
                this->close();
            });

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Cancel), &QPushButton::clicked,
            [this]()
            {
                m_curView->delete_subContour();
                gl->m_opMode = OPMODE_DRAW;
                gl->m_isDrawPlane = false;
                this->close();
                for (EasyGL *v : EasyGL::EasyGLPool())
                {
                    v->m_opMode = OPMODE_DRAW;
                    v->m_isDrawPlane = false;
                    v->repaint();
                }
            });
}

void SetInfoDialog::thickSliderValueChanged(int value)
{
    if (subToy && subToy->m_curMesh3D.number_of_vertices() > 0)
    {
        // Q_DEBUG << "thick value=" << value;
        subToy->m_layerThick = value / 100.0;
        subToy->InflateLaplaceFrom3DPlane(subToy->m_layerThick);
        m_curView->computeSubMesh3D(subToy->m_toyID);
        for (auto v : EasyGL::EasyGLPool())
        {
            m_curView->updateSubMesh3D(subToy->m_toyID, v);
            v->repaint();
        }
    }
    op_mode = SubToyOP_ADJUST_THICK;
}

void SetInfoDialog::scaleSliderValueChanged(int value)
{
    Q_DEBUG << "scale value=" << value;
    if (subToy && subToy->m_curMesh3D.number_of_vertices() > 0)
    {
        int value_change = (value - lastSclValue);
        if (value_change > 0)
        {
            scl = 1.01;
        }
        else
        {
            scl = 0.99;
        }

        Q_DEBUG << "scale value_change=" << value_change << " scl=" << scl;
        if (scl > 0.9 && scl < 1.1)
        {
            m_curView->m_toy->scale_subtoy(scl);
            m_curView->computeSubContour(subToy->m_toyID);
            m_curView->computeSubMeshFine(subToy->m_toyID);
            m_curView->computeSubMesh3D(subToy->m_toyID);
            for (auto v : EasyGL::EasyGLPool())
            {
                m_curView->update_subContours(subToy->m_toyID, v);
                m_curView->updateSubMeshFine(subToy->m_toyID, v);
                m_curView->updateSubMesh3D(subToy->m_toyID, v);
                m_curView->update_skel(v);
                m_curView->update_restSkel(v);
                v->repaint();
            }
            lastSclValue = value;
        }
        else
        {
            scl = 1.0;
        }
    }

    op_mode = SubToyOP_SCALE;
}

void SetInfoDialog::depthSliderValueChanged(int value)
{
    subToy->SetLayerDepth(value);
    int axis = (ui->radioBtnDepthX->isChecked() ? COORD_AXIS_X : (ui->radioBtnDepthY->isChecked() ? COORD_AXIS_Y : COORD_AXIS_Z));
    subToy->SetLayerAxis(axis);
    m_curView->set_plane(subToy->GetLayerPlane());
}
