#include "cloudsdrawler.h"

CloudsDrawler::CloudsDrawler(const GL_Color& color):
    BaseDrawler(), cloud_color_(color)
{
    vertex_buffer_.create();
    vertex_buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
}

void CloudsDrawler::set_points(const QVector<QVector3D> &points)
{
    vertex_buffer_.bind();
    vertex_buffer_.allocate(points.constData(), points.size() * sizeof(QVector3D));
    vertex_buffer_.release();
}

void CloudsDrawler::draw_()
{
    vertex_buffer_.bind();

    if(vertex_buffer_.size() > 0) {
        glColor3d(cloud_color_.r, cloud_color_.g, cloud_color_.b);
        glVertexPointer(3, GL_FLOAT, 0, vertex_buffer_.map(QOpenGLBuffer::Access::ReadOnly));
        glDrawArrays(GL_POINTS, 0, vertex_buffer_.size() / sizeof(QVector3D));
        vertex_buffer_.unmap();
    }

    vertex_buffer_.release();
}
