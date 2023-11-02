#pragma once

#include <QVector3D>
#include <QQuaternion>

#include "basedrawler.h"

class ModelDrawler: public BaseDrawler {
public:
    ModelDrawler(): BaseDrawler()
    {}

    void set_position(const QVector3D& pose, const QQuaternion& quaternion);
    void draw() override;
};
