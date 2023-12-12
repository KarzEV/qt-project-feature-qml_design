#include <QApplication>
#include <QQmlApplicationEngine>

#include <rov_gui_package/shell.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ros::init(argc, argv, "rov_gui_node");

    RosGuiShell ros_shell;
    ros_shell.init();
    ros_shell.show();

    return app.exec();
}

