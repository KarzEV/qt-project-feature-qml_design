#pragma once

#include <QVector>
#include <QVector3D>
#include <QString>
#include <QQuaternion>

#include "types.h"

QVector<QVector3D> parse_CSV(const QString& path);
QVector<QVector3D> parse_OBJ(const QString& path);

EulerKrylovAngles convert_to_angles(const QQuaternion &quaternion);
