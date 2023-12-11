#pragma once

#include <QOpenGLBuffer>
#include <QMatrix4x4>

class BaseDrawler
{
public:
    BaseDrawler();
    ~BaseDrawler();

    void set_model_matrix(const QMatrix4x4& model_matrix);
    virtual void draw() = 0;

protected:
    QOpenGLBuffer index_buffer_;
    QOpenGLBuffer vertex_buffer_;

    QMatrix4x4 model_matrix_;

    void check_contexst_() const;
    void initialise_vertex_buffer_();
};
