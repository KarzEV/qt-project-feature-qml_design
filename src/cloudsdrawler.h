#pragma once

#include <QVector>
#include <QVector3D>
#include <QColor>

#include "basedrawler.h"

struct GL_Color {
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
};

class CloudsDrawler: public BaseDrawler {
public:
    CloudsDrawler(const GL_Color& color);

    void set_points(const QVector<QVector3D>& points);

private:
    GL_Color cloud_color_;

    void draw_() override;
};
