#include "basedrawler.h"

#include <QOpenGLContext>

BaseDrawler::BaseDrawler(): index_buffer_(QOpenGLBuffer::IndexBuffer)
{
    model_matrix_.setToIdentity();
}

BaseDrawler::~BaseDrawler()
{
    if(index_buffer_.isCreated()) {
        index_buffer_.destroy();
    }

    if(vertex_buffer_.isCreated()) {
        vertex_buffer_.destroy();
    }
}

void BaseDrawler::set_model_matrix(const QMatrix4x4 &model_matrix) {
    model_matrix_ = model_matrix;
}

void BaseDrawler::check_contexst_() const
{
    if(!QOpenGLContext::currentContext()->isValid() || QOpenGLContext::currentContext()->isOpenGLES()) {
        throw std::runtime_error("BaseDrawler: contexst not set");
    }
}

void BaseDrawler::initialise_vertex_buffer_()
{
    if(!vertex_buffer_.create()) {
        throw std::runtime_error("BaseDrawler: buffer not created");
    }
    vertex_buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
}
