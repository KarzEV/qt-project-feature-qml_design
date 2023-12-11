#pragma once

#include <QVector>
#include <QVector3D>

#include "basedrawler.h"
#include "types.h"

class CloudsDrawler: public BaseDrawler {
public:
    CloudsDrawler(const GL_Color& color);

    void set_points(const QVector<QVector3D>& points);
    void draw() override;

private:
    GL_Color cloud_color_;

    void initialise_buffer_();
};
