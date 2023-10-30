#include "cloudswisualizer.h"

#include <math.h>

namespace  {
constexpr float ROTATE_COEF = 0.5;

GL_Color MAP_COLOR = {0.5, 0.27, 0.07};
GL_Color SONAT_COLOR = {0.5, 0.27, 0.07};

void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar )
{
    GLdouble fW = 0.0;
    GLdouble fH = 0.0;

    fH = tan( fovY / 360 * M_PI ) * zNear;
    fW = fH * aspect;

    glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}
} // end namespace

CloudsWisualizer::CloudsWisualizer(QWidget *parent): QOpenGLWidget(parent)
{
    vehicle_drawler_ = std::make_unique<ModelDrawler>();
    map_drawler_ = std::make_unique<CloudsDrawler>(MAP_COLOR);
    sonar_drawler_ = std::make_unique<CloudsDrawler>(SONAT_COLOR);
}

void CloudsWisualizer::draw_map(const QVector<QVector3D> &points)
{
    map_drawler_->set_points(points);

    update();
}

void CloudsWisualizer::draw_sonar_data(const QVector<QVector3D> &points)
{
    sonar_drawler_->set_points(points);

    update();
}

void CloudsWisualizer::draw_vehicle(const QVector3D &pose)
{
    vehicle_drawler_->set_position(pose);

    update();
}

void CloudsWisualizer::initializeGL()
{
    glClearColor(0.0f, 0.6f, 0.8f, 1.0f);

    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnableClientState(GL_VERTEX_ARRAY);
}

void CloudsWisualizer::resizeGL(int w, int h)
{
    float aspect = w / (float)h;
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
    perspectiveGL(45, aspect, 0.1f, 10.0f);

    glViewport(0.0, 0.0, (GLint)w, (GLint)h);
}

void CloudsWisualizer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 view_matrix;
    view_matrix.setToIdentity();
    view_matrix.translate(0.0, 0.0, -0.5);
    view_matrix.rotate(view_rotation_);

    glLoadMatrixf(view_matrix.constData());

    map_drawler_->draw(context(), context()->surface());
    sonar_drawler_->draw(context(), context()->surface());
    vehicle_drawler_->draw(context(), context()->surface());
}

void CloudsWisualizer::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton) {
        prev_mouse_position_ = QVector2D(event->localPos());
    }
    event->accept();
}

void CloudsWisualizer::mouseMoveEvent(QMouseEvent *event)
{
    if(event->buttons() != Qt::LeftButton) {
        return;
    }

    QVector2D new_mouse_pose = QVector2D(event->localPos());
    QVector2D diff = new_mouse_pose - prev_mouse_position_;

    float angle = diff.length();

    QVector3D axis = QVector3D(diff.x(), diff.y(), 0.0) * ROTATE_COEF;

    view_rotation_ = QQuaternion::fromAxisAndAngle(axis, angle) * view_rotation_;

    update();
}
