#pragma once

#include <QVector3D>
#include "basedrawler.h"

class ModelDrawler: public BaseDrawler {
public:
    ModelDrawler(): BaseDrawler()
    {}

    void set_position(const QVector3D& pose);

private:
    void draw_() override;
};
