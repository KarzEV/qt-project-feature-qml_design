#include "cloudswisualizer.h"

#include <math.h>

#include <QOpenGLContext>

namespace  {
constexpr float ROTATE_COEF = 0.1;

GL_Color MAP_COLOR = {1.0, 0.0, 0.0};
GL_Color SONAR_COLOR = {0.0, 1.0, 0.0};

void perspectiveGL(GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
    GLdouble fH = tan( fovY / 360 * M_PI ) * zNear;;
    GLdouble fW = fH * aspect;

    glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}

} // end namespace

CloudsWisualizer::CloudsWisualizer(QWidget *parent): QOpenGLWidget(parent), map_drawler_(MAP_COLOR), sonar_drawler_(SONAR_COLOR)
{
}

void CloudsWisualizer::draw_map(const QVector<QVector3D> &points)
{
    map_drawler_.set_points(points);
    update();
}

void CloudsWisualizer::draw_sonar_data(const QVector<QVector3D> &points)
{
    sonar_drawler_.set_points(points);
    update();
}

void CloudsWisualizer::draw_vehicle(const QVector3D &pose, const QQuaternion& quaternion)
{
    vehicle_drawler_.set_position(pose, quaternion);
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
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = w / (float)h;
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

    glMatrixMode(GL_MODELVIEW);

    glLoadMatrixf(view_matrix.constData());

    map_drawler_.draw();
    sonar_drawler_.draw();
    vehicle_drawler_.draw();
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
    prev_mouse_position_ = new_mouse_pose;

    float angle = diff.length() * ROTATE_COEF;

    QVector3D axis = QVector3D(diff.y(), diff.x(), 0.0);

    view_rotation_ = QQuaternion::fromAxisAndAngle(axis, angle) * view_rotation_;

    update();
}
