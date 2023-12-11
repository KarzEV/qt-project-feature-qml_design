#include <rov_gui_package/attitudeindicator.h>

#include <math.h>

#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QColor>

AttitudeIndicator::AttitudeIndicator(QWidget *parent)
    : QWidget(parent)
{
    m_size_ = m_sizeMin_ - 2 * m_offset_;

    setMinimumSize(m_sizeMin_, m_sizeMin_);
    setMaximumSize(m_sizeMax_, m_sizeMax_);
    resize(m_sizeMin_, m_sizeMin_);

    setFocusPolicy(Qt::NoFocus);
}

void AttitudeIndicator::set_data(double roll, double pitch)
{
    m_roll_ = roll;
    m_pitch_ = pitch;

    if(m_roll_ < -180.0) {
        m_roll_ = -180.0;
    }
    if(m_roll_ > 180.0) {
        m_roll_ = 180.0;
    }
    if(m_pitch_ < -90.0) {
        m_pitch_ = -90.0;
    }
    if(m_pitch_ > 90.0) {
        m_pitch_ = 90.0;
    }

    update();
}

void AttitudeIndicator::set_roll(double roll)
{
    m_roll_ = roll;

    if(m_roll_ < -180.0) {
        m_roll_ = -180.0;
    }
    if(m_roll_ > 180.0) {
        m_roll_ =  180.0;
    }

    update();
}

void AttitudeIndicator::set_pitch(double pitch)
{
    m_pitch_ = pitch;

    if(m_pitch_ < -90.0) {
        m_pitch_ = -90.0;
    }
    if(m_pitch_ > 90.0) {
        m_pitch_ = 90.0;
    }

    update();
}

void AttitudeIndicator::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    QBrush bgSky(QColor(48, 172, 220));
    QBrush bgGround(Qt::green);

    QPen whitePen(Qt::white);
    QPen blackPen(Qt::black);
    QPen pitchPen(Qt::white);
    QPen pitchZero(Qt::yellow);

    whitePen.setWidth(2);
    blackPen.setWidth(2);
    pitchZero.setWidth(3);

    painter.setRenderHint(QPainter::Antialiasing);

    painter.translate(width() / 2, height() / 2);
    painter.rotate(m_roll_);

    double pitch_tem = -m_pitch_;

    {
        int y_min = m_size_ / 2 * -40.0 / 45.0;
        int y_max = m_size_ / 2 * 40.0 / 45.0;

        int y = m_size_ / 2 * pitch_tem / 45.0;
        if(y < y_min) {
            y = y_min;
        }
        if(y > y_max) {
            y = y_max;
        }

        int x = sqrt(m_size_ * m_size_ / 4 - y * y);
        qreal gr = atan((double)(y) / x);
        gr = gr * 180.0 / M_PI;

        painter.setPen(blackPen);
        painter.setBrush(bgSky);
        painter.drawChord(-m_size_ / 2, -m_size_ / 2, m_size_, m_size_,
                          gr * 16, (180 - 2 * gr) * 16);

        painter.setBrush(bgGround);
        painter.drawChord(-m_size_ / 2, -m_size_ / 2, m_size_, m_size_,
                          gr * 16, -(180 + 2 * gr) * 16);
    }

    QRegion maskRegion(-m_size_ / 2, -m_size_ / 2, m_size_, m_size_, QRegion::Ellipse);
    painter.setClipRegion(maskRegion);

    {
        int x = 0;
        int y = 0;
        int x1 = 0;
        int y1 = 0;
        int textWidth = 0;
        double p = 0.0;
        double r = 0.0;
        int ll = m_size_ / 8, l;

        int fontSize = 8;
        QString s;

        pitchPen.setWidth(2);
        painter.setFont(QFont("", fontSize));

        for(int i = -9; i <= 9; i++) {
            p = i * 10;

            s = QString("%1").arg(-p);

            if(i % 3 == 0) {
                l = ll;
            } else {
                l = ll/2;
            }

            if(i == 0) {
                painter.setPen(pitchZero);
                l = l * 1.8;
            } else {
                painter.setPen(pitchPen);
            }

            y = m_size_ / 2 * p / 45.0 - m_size_ / 2 * pitch_tem / 45.0;
            x = l;

            r = sqrt(x * x + y * y);
            if(r > m_size_ / 2) {
                continue;
            }

            painter.drawLine(QPointF(-l, 1.0 * y), QPointF(l, 1.0 * y));

            textWidth = 100;

            if(i % 3 == 0 && i != 0) {
                painter.setPen(QPen(Qt::white));

                x1 = -x - 2 - textWidth;
                y1 = y - fontSize / 2 - 1;
                painter.drawText(QRectF(x1, y1, textWidth, fontSize + 2),
                                 Qt::AlignRight | Qt::AlignVCenter, s);
            }
        }

        int markerSize = m_size_ / 20;

        painter.setBrush(QBrush(Qt::red));
        painter.setPen(Qt::NoPen);

        float fx1 = markerSize;
        float fy1 = 0.0;
        float fx2 = fx1 + markerSize;
        float fy2 = -markerSize / 2;
        float fx3 = fx1 + markerSize;
        float fy3 = markerSize / 2;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);

        QPointF points2[3] = {
            QPointF(-fx1, fy1),
            QPointF(-fx2, fy2),
            QPointF(-fx3, fy3)
        };
        painter.drawPolygon(points2, 3);
    }

    {
        int nRollLines = 36;
        float rotAng = 360.0 / nRollLines;
        int rollLineLeng = m_size_ / 25;
        double fx1 = 0.0;
        double fy1 = 0.0;
        double fx2 = 0.0;
        double fy2 = 0.0;
        int fontSize = 8;
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);
        painter.setFont(QFont("", fontSize));

        for(int i = 0; i < nRollLines; i++) {
            if( i < nRollLines/2 ) {
                s = QString("%1").arg(-i*rotAng);
            } else {
                s = QString("%1").arg(360-i*rotAng);
            }

            fx1 = 0.0;
            fy1 = -m_size_ / 2 + m_offset_;
            fx2 = 0.0;

            if(i % 3 == 0) {
                fy2 = fy1 + rollLineLeng;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + rollLineLeng + 2;
                painter.drawText(QRectF(-50, fy2, 100, fontSize+2),
                                 Qt::AlignCenter, s);
            } else {
                fy2 = fy1 + rollLineLeng / 2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(rotAng);
        }
    }

    {
        int rollMarkerSize = m_size_ / 25;

        painter.rotate(-m_roll_);
        painter.setBrush(QBrush(Qt::black));

        double fx1 = 0.0;
        double fy1 = -m_size_ / 2 + m_offset_;
        double fx2 = fx1 - rollMarkerSize / 2;
        double fy2 = fy1 + rollMarkerSize;
        double fx3 = fx1 + rollMarkerSize / 2;
        double fy3 = fy1 + rollMarkerSize;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);
    }
}

void AttitudeIndicator::resizeEvent(QResizeEvent *event)
{
    m_size_ = qMin(width(), height()) - 2 * m_offset_;
}

void AttitudeIndicator::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Left:
        m_roll_ -= 1.0;
        break;
    case Qt::Key_Right:
        m_roll_ += 1.0;
        break;
    case Qt::Key_Down:
        if(m_pitch_>-90.)
            m_pitch_ -=1.0;
        break;
    case Qt::Key_Up:
        if(m_pitch_<90.)
            m_pitch_ +=1.0;
        break;
    default:
        QWidget::keyPressEvent(event);
        break;
    }

    update();
}
