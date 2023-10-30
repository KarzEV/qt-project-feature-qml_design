#pragma once

#include <memory>

#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QVector>
#include <QVector3D>
#include <QVector2D>
#include <QMouseEvent>
#include <QQuaternion>

#include "cloudsdrawler.h"
#include "modeldrawler.h"

class CloudsWisualizer : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit CloudsWisualizer(QWidget *parent = nullptr);
    ~CloudsWisualizer() = default;

public slots:
    void draw_map(const QVector<QVector3D>& points);
    void draw_sonar_data(const QVector<QVector3D>& points);
    void draw_vehicle(const QVector3D& pose);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    std::unique_ptr<CloudsDrawler> map_drawler_;
    std::unique_ptr<CloudsDrawler> sonar_drawler_;
    std::unique_ptr<ModelDrawler> vehicle_drawler_;

    QVector2D prev_mouse_position_;
    QQuaternion view_rotation_;
};
