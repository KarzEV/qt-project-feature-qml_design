#include "cloudsvisualizer.h"

#include <math.h>

#include <QOpenGLContext>

namespace  {

GL_Color MAP_COLOR = {1.0, 0.0, 0.0};
GL_Color SONAR_COLOR = {0.0, 1.0, 0.0};
GL_Color VEHICLE_COLOR = {1.0, 0.5, 0.0};

void perspectiveGL(GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
    GLdouble fH = tan( fovY / 360 * M_PI ) * zNear;;
    GLdouble fW = fH * aspect;

    glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}

} // end namespace

CloudsVisualizer::CloudsVisualizer(const QString& vehicle_model_path, QWidget *parent): QOpenGLWidget(parent),
    map_drawler_(MAP_COLOR), sonar_drawler_(SONAR_COLOR), vehicle_drawler_(VEHICLE_COLOR, vehicle_model_path)
{
    reset_view_matrix_();
}

void CloudsVisualizer::draw_map(const QVector<QVector3D> &points)
{
    map_drawler_.set_points(points);
    update();
}

void CloudsVisualizer::draw_sonar_data(const QVector<QVector3D> &points)
{
    sonar_drawler_.set_points(points);
    update();
}

void CloudsVisualizer::draw_vehicle(const QVector3D &pose, const QQuaternion& quaternion)
{
    vehicle_drawler_.set_position(pose, quaternion);
    update();
}

void CloudsVisualizer::initializeGL()
{
    glClearColor(0.0f, 0.6f, 0.8f, 1.0f);

    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnableClientState(GL_VERTEX_ARRAY);
}

void CloudsVisualizer::resizeGL(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = w / (float)h;
    perspectiveGL(45, aspect, 0.1f, 100.0f);

    glViewport(0.0, 0.0, (GLint)w, (GLint)h);
}

void CloudsVisualizer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadMatrixf(view_matrix_.constData());

    map_drawler_.draw();
    sonar_drawler_.draw();
    vehicle_drawler_.draw();
}

void CloudsVisualizer::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton || event->buttons() == Qt::MiddleButton) {
        prev_mouse_position_ = QVector2D(event->localPos());
    }
    event->accept();
}

void CloudsVisualizer::mouseMoveEvent(QMouseEvent *event)
{
    switch (event->buttons()) {
        case Qt::LeftButton:
            rotate_camera_(event);
        break;
        case Qt::MiddleButton:
            translate_camera_(event);
        break;
        default:
        return;
    }
    event->accept();

    update();
}

void CloudsVisualizer::wheelEvent(QWheelEvent *pe)
{
    if(pe->delta() > 0) {
        view_matrix_.scale(1.1);
    } else if(pe->delta() < 0){
        view_matrix_.scale(1 / 1.1);
    }
    pe->accept();

    update();
}

void CloudsVisualizer::keyPressEvent(QKeyEvent *pe)
{
    switch (pe->key())
    {
        case Qt::Key_Escape:
            reset_view_matrix_();
        break;
        default:
        return;
    }
    pe->accept();

    update();
}

void CloudsVisualizer::rotate_camera_(QMouseEvent *event)
{
    constexpr float ROTATE_COEF = 0.1;
    QVector2D diff = processing_mouse_move_(event);

    float angle = diff.length() * ROTATE_COEF;
    QVector3D axis = QVector3D(diff.y(), diff.x(), 0.0);

    view_matrix_.rotate(QQuaternion::fromAxisAndAngle(axis, angle));
}

void CloudsVisualizer::translate_camera_(QMouseEvent *event)
{
    constexpr float TRANSLATE_COEF = 0.0001;
    QVector2D diff = processing_mouse_move_(event) * TRANSLATE_COEF;
    diff.setY(-diff.y());
    QVector3D transformed_diff = view_matrix_.inverted().mapVector(diff.toVector3D());

    view_matrix_.translate(transformed_diff.x(), transformed_diff.y(), transformed_diff.z());
}

QVector2D CloudsVisualizer::processing_mouse_move_(QMouseEvent *event)
{
    QVector2D new_mouse_pose = QVector2D(event->localPos());
    QVector2D diff = new_mouse_pose - prev_mouse_position_;
    prev_mouse_position_ = new_mouse_pose;

    return diff;
}

void CloudsVisualizer::reset_view_matrix_()
{
    view_matrix_.setToIdentity();
    view_matrix_.translate(0.0, 0.0, -0.5);
}
