#pragma once

#include <QWidget>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QKeyEvent>

class Compass: public QWidget
{
    Q_OBJECT

public:
    Compass(QWidget *parent = 0);
    ~Compass() = default;

    void set_data(double course, double alt, double h);
    void set_course(double course);
    void set_alt(double alt);
    void set_H(double h);

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void keyPressEvent(QKeyEvent *event);

protected:
    int m_sizeMin_ = 220;
    int m_sizeMax_ = 600;
    int m_size_ = 0;
    int m_offset_ = 2;

    double m_course_ = 0.0;
    double m_alt_ = 0.0;
    double m_h_ = 0.0;
};
