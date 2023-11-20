#include <QApplication>
#include <QQmlApplicationEngine>

#include "mainwindow.h"
#include "utils.h"

void set_test_data(MainWindow& main_window) {
    const QVector<QVector3D> TEST_MAP_POINTS = parse_CSV(":/resources/test_map.csv");
    const QVector<QVector3D> TEST_SONAR_POINTS = parse_CSV(":/resources/test_sonar.csv");

    main_window.draw_map(TEST_MAP_POINTS);
    main_window.draw_sonar_data(TEST_SONAR_POINTS);
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    MainWindow w(":/resources/submarine.obj");
    w.setWindowTitle("Cloud Tool");
    w.show();
    set_test_data(w);

    return app.exec();
}

