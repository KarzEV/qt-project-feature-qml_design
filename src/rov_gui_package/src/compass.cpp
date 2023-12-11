#include <rov_gui_package/compass.h>

#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QColor>

Compass::Compass(QWidget *parent)
    : QWidget(parent)
{
    m_size_ = m_sizeMin_ - 2 * m_offset_;

    setMinimumSize(m_sizeMin_, m_sizeMin_);
    setMaximumSize(m_sizeMax_, m_sizeMax_);
    resize(m_sizeMin_, m_sizeMin_);

    setFocusPolicy(Qt::NoFocus);
}

void Compass::set_data(double course, double alt, double h)
{
    m_course_ = course;
    m_alt_ = alt;
    m_h_   = h;

    if(m_course_ < 0.0) {
       m_course_ = 360 + m_course_;
    }
    if(m_course_ > 360.0) {
       m_course_ = m_course_ - 360;
    }

    update();
}

void Compass::set_course(double course)
{
    m_course_  = course;

    if(m_course_ < 0.0) {
       m_course_ = 360 + m_course_;
    }
    if(m_course_ > 360.0) {
       m_course_ = m_course_ - 360;
    }

    update();
}

void Compass::set_alt(double alt)
{
    m_alt_ = alt;

    update();
}

void Compass::set_H(double h)
{
    m_h_ = h;

    update();
}

void Compass::resizeEvent(QResizeEvent *event)
{
    m_size_ = qMin(width(),height()) - 2*m_offset_;
}

void Compass::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    QBrush bgGround(QColor(48, 172, 220));

    QPen whitePen(Qt::white);
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    QPen bluePen(Qt::blue);
    QPen greenPen(Qt::green);

    whitePen.setWidth(1);
    blackPen.setWidth(2);
    redPen.setWidth(2);
    bluePen.setWidth(2);
    greenPen.setWidth(2);

    painter.setRenderHint(QPainter::Antialiasing);

    painter.translate(width() / 2, height() / 2);

    {
        painter.setPen(blackPen);
        painter.setBrush(bgGround);

        painter.drawEllipse(-m_size_ / 2, -m_size_ / 2, m_size_, m_size_);
    }

    {
        int ncourseLines = 36;
        float rotAng = 360.0 / ncourseLines;
        int courseLineLeng = m_size_ / 25;
        double fx1 = 0.0;
        double fy1 = 0.0;
        double fx2 = 0.0;
        double fy2 = 0.0;
        int fontSize = 8;
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);

        for(int i = 0; i < ncourseLines; i++) {

            if(i == 0) {
                s = "N";
                painter.setPen(bluePen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if (i == 9) {
                s = "W";
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if (i == 18) {
                s = "S";
                painter.setPen(redPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if (i == 27) {
                s = "E";
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else {
                s = QString("%1").arg(i*rotAng);
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize));
            }

            fx1 = 0;
            fy1 = -m_size_ / 2 + m_offset_;
            fx2 = 0;

            if(i % 3 == 0) {
                fy2 = fy1 + courseLineLeng;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + courseLineLeng+4;
                painter.drawText(QRectF(-50, fy2, 100, fontSize+2),
                                 Qt::AlignCenter, s);
            } else {
                fy2 = fy1 + courseLineLeng/2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(-rotAng);
        }
    }

    {
        int arrowWidth = m_size_ / 5;

        double fx1 = 0.0;
        double fy1 = -m_size_ / 2 + m_offset_ + m_size_ / 25 + 15;
        double fx2 = -arrowWidth / 2;
        double fy2 = 0.0;
        double fx3 = arrowWidth / 2;
        double fy3 = 0.0;

        painter.setPen(Qt::NoPen);

        painter.setBrush(QBrush(Qt::blue));
        QPointF pointsN[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsN, 3);

        fx1 = 0.0;
        fy1 = m_size_ / 2 - m_offset_ - m_size_ / 25 - 15;
        fx2 = -arrowWidth / 2;
        fy2 = 0.0;
        fx3 = arrowWidth / 2;
        fy3 = 0.0;

        painter.setBrush(QBrush(Qt::red));
        QPointF pointsS[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsS, 3);
    }

    {
        int courseMarkerSize = m_size_ / 12;

        painter.rotate(-m_course_);
        painter.setBrush(QBrush(QColor(0xFF, 0x00, 0x00, 0xE0)));

        double fx1 = 0.0;
        double fy1 = -m_size_ / 2 + m_offset_;
        double fx2 = fx1 - courseMarkerSize / 2;
        double fy2 = fy1 + courseMarkerSize;
        double fx3 = fx1 + courseMarkerSize / 2;
        double fy3 = fy1 + courseMarkerSize;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);

        painter.rotate(m_course_);
    }

    {
        int altFontSize = 8;
        QString s;
        char buf[200];

        int w  = 150;
        int h  = 2 * (altFontSize + 12);
        int fx = -w / 2;
        int fy = -h / 2;

        blackPen.setWidth(2);
        painter.setPen(blackPen);
        painter.setBrush(QBrush(Qt::white));
        painter.setFont(QFont("", altFontSize));

        painter.drawRoundedRect(fx, fy, w, h, 6, 6);

        painter.setPen(bluePen);
        sprintf(buf, "ALT: %6.1f m", m_alt_);
        s = buf;
        painter.drawText(QRectF(fx, fy+2, w, h/2), Qt::AlignCenter, s);

        sprintf(buf, "H: %6.1f m", m_h_);
        s = buf;
        painter.drawText(QRectF(fx, fy+h/2, w, h/2), Qt::AlignCenter, s);
    }
}

void Compass::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Left:
        m_course_ -= 1.0;
        break;
    case Qt::Key_Right:
        m_course_ += 1.0;
        break;
    case Qt::Key_Down:
        m_alt_ -= 1.0;
        break;
    case Qt::Key_Up:
        m_alt_ += 1.0;
        break;
    case Qt::Key_W:
        m_h_ += 1.0;
        break;
    case Qt::Key_S:
        m_h_ -= 1.0;
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }

    update();
}

