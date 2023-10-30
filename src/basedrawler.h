#pragma once

#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QOpenGLContext>
#include <QSurface>

class BaseDrawler
{
public:
    BaseDrawler();
    ~BaseDrawler();

    void set_model_matrix(const QMatrix4x4& model_matrix);
    virtual void draw(QOpenGLContext* parent_context, QSurface* parent_surface);

protected:
    QOpenGLBuffer index_buffer_;
    QOpenGLBuffer vertex_buffer_;

    QMatrix4x4 model_matrix_;

    virtual void draw_() = 0;

private:
    void set_contexst_(QOpenGLContext* parent_context, QSurface* parent_surface);
    void release_contexst_(QOpenGLContext* parent_context);
};
