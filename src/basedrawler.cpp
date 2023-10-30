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

void BaseDrawler::draw(QOpenGLContext* parent_context, QSurface* parent_surface)
{
    set_contexst_(parent_context, parent_surface);
    draw_();
    release_contexst_(parent_context);
}

void BaseDrawler::set_contexst_(QOpenGLContext* parent_context, QSurface* parent_surface)
{
    parent_context->makeCurrent(parent_surface);
}

void BaseDrawler::release_contexst_(QOpenGLContext* parent_context)
{
    parent_context->doneCurrent();
}
