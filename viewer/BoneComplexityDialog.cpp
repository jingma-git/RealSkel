#include <QPushButton>
#include <QComboBox>
#include "BoneComplexityDialog.h"
#include "ui_BoneComplexityDialog.h"
#include "EasyGL.h"
#include "mainwindow.h"

BoneComplexityDialog::BoneComplexityDialog(QWidget *parent) : QDialog(parent),
                                                              ui(new Ui::BoneComplexityDialog)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose);

    gl = qobject_cast<EasyGL *>(parent);
    m_curView = gl->m_curView;
    m_toy = m_curView->m_toy;

    SubViewer *subviewer = qobject_cast<SubViewer *>(gl->parentWidget());
    if (subviewer)
    {
        //获取父窗口geometry
        QRect rect = subviewer->geometry();
        //计算显示原点
        this->move(rect.x() + gl->pos().x() + gl->width() - this->width() / 2, rect.y() + gl->pos().y() + 70 + this->height() / 2);
    }

    ui->comboBoxSkelType->addItem(QString("polygon offsetting"));
    ui->comboBoxSkelType->addItem(QString("symmetry line"));
    connect(ui->comboBoxSkelType, SIGNAL(currentIndexChanged(int)), this, SLOT(skelTypeChanged(int)));

    // Branch Level complexity control
    connect(ui->sliderBC, &QSlider::valueChanged, this, &BoneComplexityDialog::sliderBCValueChanged);

    // Part & Global Level complexity control
    ui->sliderMerge->setValue(m_toy->m_merge_thresh);
    ui->sliderPrune->setValue(m_toy->m_prune_thresh);
    ui->sliderCollapse->setValue(m_toy->m_collapse_thresh);
    updateGlobalThresh();

    connect(ui->sliderMerge, &QSlider::valueChanged, this, &BoneComplexityDialog::sliderMergeValueChanged);
    connect(ui->sliderPrune, &QSlider::valueChanged, this, &BoneComplexityDialog::sliderPruneValueChanged);
    connect(ui->sliderCollapse, &QSlider::valueChanged, this, &BoneComplexityDialog::sliderCollapseValueChanged);

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Ok), &QPushButton::clicked,
            [this]()
            {
                this->close();
            });

    connect(ui->buttonBox->button(QDialogButtonBox::StandardButton::Cancel), &QPushButton::clicked,
            [this]()
            {
                this->close();
            });
}

void BoneComplexityDialog::updateGlobalThresh()
{
    updateMergeThresh();
    updatePruneThresh();
    updateCollapseThresh();
}

void BoneComplexityDialog::updateMergeThresh()
{
    int mergeThresh = m_toy->avg_bone_len();
    ui->labelMerge->setText(QString::number(mergeThresh));
    ui->sliderMerge->setMaximum(mergeThresh);
}

void BoneComplexityDialog::updatePruneThresh()
{
    int pruneThresh = m_toy->max_jt_edge_len() + 10;
    ui->labelPrune->setText(QString::number(pruneThresh));
    ui->sliderPrune->setMaximum(pruneThresh);
}

void BoneComplexityDialog::updateCollapseThresh()
{
    int collapseThresh = m_toy->max_bone_len();
    ui->labelCollapse->setText(QString::number(collapseThresh));
    ui->sliderCollapse->setMaximum(collapseThresh);
}

void BoneComplexityDialog::skelTypeChanged(int index)
{
    M_DEBUG << "change skel type " << index << endl;
    XSubToy *subToy = m_toy->GetCurToy();
    subToy->Skeletonize(index);
    m_toy->global_skel_optim();
    updateGlobalThresh();

    for (auto v : EasyGL::EasyGLPool())
    {
        m_curView->update_skel(v);
        m_curView->update_restSkel(v);
        v->repaint();
    }
}

void BoneComplexityDialog::sliderBCValueChanged(int value)
{
    Q_DEBUG << " Change Bone Complexity value=" << value;
    m_toy->change_bone_complexity(value);
    updateGlobalThresh();
    for (auto v : EasyGL::EasyGLPool())
    {
        m_curView->update_skel(v);
        m_curView->update_restSkel(v);
        v->repaint();
    }
}

void BoneComplexityDialog::sliderMergeValueChanged(int value)
{
    m_toy->merge_junc_nodes(value);
    updatePruneThresh();
    for (auto v : EasyGL::EasyGLPool())
    {
        m_curView->update_skel(v);
        m_curView->update_restSkel(v);
        v->repaint();
    }
}

void BoneComplexityDialog::sliderPruneValueChanged(int value)
{
    m_toy->prune_jt_edges(value);
    updateCollapseThresh();
    for (auto v : EasyGL::EasyGLPool())
    {
        m_curView->update_skel(v);
        m_curView->update_restSkel(v);
        v->repaint();
    }
}

void BoneComplexityDialog::sliderCollapseValueChanged(int value)
{
    m_toy->collapse_short_edges(value);
    for (auto v : EasyGL::EasyGLPool())
    {
        m_curView->update_skel(v);
        m_curView->update_restSkel(v);
        v->repaint();
    }
}