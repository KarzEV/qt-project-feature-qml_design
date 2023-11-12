#include "vehiclestatetable.h"
#include "ui_vehiclestatetable.h"

#include <QString>

#include "utils.h"

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

void VehicleStateTable::set_vehicle_state(const QVector3D &pose, const QQuaternion &quaternion)
{
    ui->x_edit->setText(QString::number(pose.x()));
    ui->y_edit->setText(QString::number(pose.y()));
    ui->z_edit->setText(QString::number(pose.z()));

    auto angles = convert_to_angles(quaternion);

    ui->course_edit->setText(QString::number(angles.cource));
    ui->trim_edit->setText(QString::number(angles.trim));
    ui->roll_edit->setText(QString::number(angles.roll));
}
