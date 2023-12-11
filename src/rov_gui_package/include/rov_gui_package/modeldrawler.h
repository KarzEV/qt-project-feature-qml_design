#pragma once

#include <QVector3D>
#include <QQuaternion>
#include <QString>

#include "basedrawler.h"
#include "types.h"

class ModelDrawler: public BaseDrawler {
public:
    ModelDrawler(const GL_Color& color, const QString& path): BaseDrawler(),
        model_color_(color), path_(path)
    {}

    void set_position(const QVector3D& pose, const QQuaternion& quaternion);
    void draw() override;

private:
    GL_Color model_color_;
    QString path_;

    void setup_model_data_();
    void draw_axis_();
};
