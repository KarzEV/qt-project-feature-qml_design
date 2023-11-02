#include <QApplication>
#include <QQmlApplicationEngine>
#include "cloudsvisualizer.h"

void set_test_data(CloudsVisualizer& cloud_vis) {
    const QVector<QVector3D> TEST_MAP_POINTS = {{0.5, 0.5, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.5, -1.0}, {0.5, 0.0, -2.0}, {0.0, 0.0, -1.0}};
    const QVector<QVector3D> TEST_SONAR_POINTS = {{0.2, 0.2, -1.0}, {-0.2, -0.2, -1.0}};

    cloud_vis.draw_map(TEST_MAP_POINTS);
    cloud_vis.draw_sonar_data(TEST_SONAR_POINTS);
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    CloudsVisualizer cloud_vis;
    cloud_vis.setWindowTitle("Cloud Tool");
    cloud_vis.resize(500, 500);
    cloud_vis.show();
    set_test_data(cloud_vis);

    return app.exec();
}

