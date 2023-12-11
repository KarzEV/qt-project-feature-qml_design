#include <rov_gui_package/vehiclestatetable.h>
#include "ui_vehiclestatetable.h"

#include <QString>

VehicleStateTable::VehicleStateTable(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::vehicle_state_table)
{
    ui->setupUi(this);
}

VehicleStateTable::~VehicleStateTable()
{
    delete ui;
}

void VehicleStateTable::set_vehicle_state(const QVector3D &pose, const EulerKrylovAngles& angles)
{
    ui->x_edit->setText(QString::number(pose.x()));
    ui->y_edit->setText(QString::number(pose.y()));
    ui->z_edit->setText(QString::number(pose.z()));

    ui->course_edit->setText(QString::number(angles.cource));
    ui->pitch_edit->setText(QString::number(angles.pitch));
    ui->roll_edit->setText(QString::number(angles.roll));
}
