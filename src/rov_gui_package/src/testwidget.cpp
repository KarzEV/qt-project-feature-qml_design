#include <rov_gui_package/testwidget.h>
#include "ui_testwidget.h"

#include <rov_gui_package/utils.h>

static float LINEAR_COEFFICIENT = 1 / 20.0;

TestWidget::TestWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TestWidget)
{
    ui->setupUi(this);

    setup_layout_();

    pose_.setX(ui->X_Slider->value());
    pose_.setY(ui->Y_Slider->value());
    pose_.setZ(ui->Z_Slider->value());

    angles_.cource = ui->Cource_Slider->value();
    angles_.roll = ui->Roll_Slider->value();
    angles_.pitch = ui->Pitch_Slider->value();
}

TestWidget::~TestWidget()
{
    delete ui;
}

void TestWidget::setup_layout_()
{
    ui->X_Layout->setAlignment(ui->X_Slider, Qt::AlignHCenter);
    ui->X_Layout->setAlignment(ui->X_Label, Qt::AlignHCenter);

    ui->Y_Layout->setAlignment(ui->Y_Slider, Qt::AlignHCenter);
    ui->Y_Layout->setAlignment(ui->Y_Label, Qt::AlignHCenter);

    ui->Z_Layout->setAlignment(ui->Z_Slider, Qt::AlignHCenter);
    ui->Z_Layout->setAlignment(ui->Z_Label, Qt::AlignHCenter);

    ui->Cource_Layout->setAlignment(ui->Cource_Slider, Qt::AlignHCenter);
    ui->Cource_Layout->setAlignment(ui->Cource_Label, Qt::AlignHCenter);

    ui->Pitch_Layout->setAlignment(ui->Pitch_Slider, Qt::AlignHCenter);
    ui->Pitch_Layout->setAlignment(ui->Pitch_Label, Qt::AlignHCenter);

    ui->Roll_Layout->setAlignment(ui->Roll_Slider, Qt::AlignHCenter);
    ui->Roll_Layout->setAlignment(ui->Roll_Label, Qt::AlignHCenter);
}

void TestWidget::on_X_Slider_valueChanged(int value)
{
    pose_.setX(value * LINEAR_COEFFICIENT);
    send_new_vehicle_pose_();
}


void TestWidget::on_Y_Slider_valueChanged(int value)
{
    pose_.setY(value * LINEAR_COEFFICIENT);
    send_new_vehicle_pose_();
}


void TestWidget::on_Z_Slider_valueChanged(int value)
{
    pose_.setZ(value * LINEAR_COEFFICIENT);
    send_new_vehicle_pose_();
}


void TestWidget::on_Cource_Slider_valueChanged(int value)
{
    angles_.cource = value;
    send_new_vehicle_pose_();
}


void TestWidget::on_Roll_Slider_valueChanged(int value)
{
    angles_.roll = value;
    send_new_vehicle_pose_();
}


void TestWidget::on_Pitch_Slider_valueChanged(int value)
{
    angles_.pitch = value;
    send_new_vehicle_pose_();
}

void TestWidget::send_new_vehicle_pose_()
{
    auto orientation = convert_to_quaternion(angles_);
    emit change_vehicle_pose(pose_, orientation, angles_);
}
