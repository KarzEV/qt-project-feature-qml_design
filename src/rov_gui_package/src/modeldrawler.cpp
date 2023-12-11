#include <rov_gui_package/modeldrawler.h>

#include <rov_gui_package/utils.h>

void ModelDrawler::set_position(const QVector3D &pose, const QQuaternion& quaternion)
{
    model_matrix_.setToIdentity();

    model_matrix_.translate(pose);
    model_matrix_.rotate(quaternion);
}

void ModelDrawler::draw()
{
    if(path_.isEmpty()) {
        return;
    }

    check_contexst_();
    if(!vertex_buffer_.isCreated()) {
        setup_model_data_();
    }

    if(!vertex_buffer_.bind()) {
        throw std::runtime_error("ModelDrawler: buffer not binded in draw");
    }

    if(vertex_buffer_.size() > 0) {
        glColor3d(model_color_.r, model_color_.g, model_color_.b);

        glVertexPointer(3, GL_FLOAT, 0, 0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();

        float gl_transform[16];
        for(int i = 0; i < 4; ++i) {
           for(int j = 0; j < 4; ++j) {
               gl_transform[i*4 + j] = model_matrix_.transposed()(i, j);
           }
        }

        glMultMatrixf(gl_transform);
        glDrawArrays(GL_TRIANGLES, 0, vertex_buffer_.size() / sizeof(QVector3D));
        draw_axis_();

        glPopMatrix();
    }

    vertex_buffer_.release();
}

void ModelDrawler::setup_model_data_()
{
    initialise_vertex_buffer_();

    if(!vertex_buffer_.bind()) {
        throw std::runtime_error("ModelDrawler: buffer not binded");
    }

    auto vertexses = parse_OBJ(path_);
    vertex_buffer_.allocate(vertexses.constData(), vertexses.size() * sizeof(QVector3D));
    vertex_buffer_.release();
}

void ModelDrawler::draw_axis_()
{
    glLineWidth(3.0f);
    glBegin(GL_LINES);
       glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
       glVertex3f(1.5f,  0.0f,  0.0f);
       glVertex3f(0.0f,  0.0f,  0.0f);
       glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
       glVertex3f(0.0f,  1.5f,  0.0f);
       glVertex3f(0.0f, 0.0f,  0.0f);
       glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
       glVertex3f(0.0f,  0.0f,  1.5f);
       glVertex3f(0.0f,  0.0f, 0.0f);
    glEnd();
}
