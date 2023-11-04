#include <QApplication>
#include <QQmlApplicationEngine>
#include "mainwindow.h"

void set_test_data(MainWindow& main_window) {
    const QVector<QVector3D> TEST_MAP_POINTS = {{0.5, 0.5, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.5, -1.0}, {0.5, 0.0, -2.0}, {0.0, 0.0, -1.0}};
    const QVector<QVector3D> TEST_SONAR_POINTS = {{0.2, 0.2, -1.0}, {-0.2, -0.2, -1.0}};

    main_window.draw_map(TEST_MAP_POINTS);
    main_window.draw_sonar_data(TEST_SONAR_POINTS);
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    MainWindow w;
    w.setWindowTitle("Cloud Tool");
    w.show();
    set_test_data(w);

    return app.exec();
}

