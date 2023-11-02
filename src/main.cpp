#include <QApplication>
#include <QQmlApplicationEngine>
#include "cloudswisualizer.h"

static QVector<QVector3D> TEST_MAP_POINTS = {{0.5, 0.5, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.5, -1.0}, {0.5, 0.0, -2.0}, {0.0, 0.0, -1.0}};
static QVector<QVector3D> TEST_SONAR_POINTS = {{0.2, 0.2, -1.0}, {-0.2, -0.2, -1.0}};

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    CloudsWisualizer cloud_wis_;
    cloud_wis_.setWindowTitle("Cloud Tool");
    cloud_wis_.resize(500, 500);
    cloud_wis_.show();
    cloud_wis_.draw_map(TEST_MAP_POINTS);
    cloud_wis_.draw_sonar_data(TEST_SONAR_POINTS);

    return app.exec();
}

