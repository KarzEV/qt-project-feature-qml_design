#pragma once

#include <QWidget>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QKeyEvent>

class AttitudeIndicator: public QWidget
{
    Q_OBJECT

public:
    AttitudeIndicator(QWidget *parent = 0);
    ~AttitudeIndicator() = default;

    void set_data(double roll, double pitch);
    void set_roll(double roll);
    void set_pitch(double pitch);

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void keyPressEvent(QKeyEvent *event);

protected:
    int m_sizeMin_ = 220;
    int m_sizeMax_ = 600;
    int m_size_ = 0;
    int m_offset_ = 2;

    double m_roll_ = 0.0;
    double m_pitch_ = 0.0;
};
