#include "basedrawler.h"

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
