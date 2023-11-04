#pragma once

#include <QVector>
#include <QVector3D>
#include <QString>

QVector<QVector3D> parse_CSV(const QString& path);
QVector<QVector3D> parse_OBJ(const QString& path);
