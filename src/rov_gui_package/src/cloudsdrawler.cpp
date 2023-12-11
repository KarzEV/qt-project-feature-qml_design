#include <rov_gui_package/cloudsdrawler.h>

#include <stdexcept>

CloudsDrawler::CloudsDrawler(const GL_Color& color):
    BaseDrawler(), cloud_color_(color)
{
}

void CloudsDrawler::set_points(const QVector<QVector3D> &points)
{
    check_contexst_();

    if(!vertex_buffer_.isCreated()) {
        initialise_vertex_buffer_();
    }

    if(!vertex_buffer_.bind()) {
        throw std::runtime_error("CloudsDrawler: buffer not binded");
    }

    vertex_buffer_.allocate(points.constData(), points.size() * sizeof(QVector3D));
    vertex_buffer_.release();
}

void CloudsDrawler::draw()
{
    if(!vertex_buffer_.isCreated() ) {
        return;
    }

    check_contexst_();
    if(!vertex_buffer_.bind()) {
        throw std::runtime_error("CloudsDrawler: buffer not binded in draw");
    }


    if(vertex_buffer_.size() > 0) {
        glEnable(GL_PROGRAM_POINT_SIZE);

        glPointSize(10);
        glColor3d(cloud_color_.r, cloud_color_.g, cloud_color_.b);

        glVertexPointer(3, GL_FLOAT, 0, 0);
        glDrawArrays(GL_POINTS, 0, vertex_buffer_.size() / sizeof(QVector3D));

        glDisable(GL_PROGRAM_POINT_SIZE);
    }

    vertex_buffer_.release();
}
