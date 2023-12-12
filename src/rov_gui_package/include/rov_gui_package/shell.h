#include <QTimer>
#include <QObject>

#include <ros/ros.h>

#include <rov_gui_package/mainwindow.h>

#include <rov_msgs_package/SonarData.h>
#include <rov_msgs_package/VehiclePose.h>
#include <rov_msgs_package/CloudMap.h>

class RosGuiShell: public QObject
{
    Q_OBJECT

public:
  RosGuiShell(): main_window_(":/resources/submarine.obj"), timer_(this) {}

  void init();
  void show();

private slots:
  void sipn_once();

private:
  MainWindow main_window_;

  bool debug_load_ = true;

  QTimer timer_;

  ros::Subscriber vehicle_pose_sub_;
  ros::Subscriber sonar_data_sub_;

  ros::ServiceServer map_srv_;

  void set_test_data_();
  void vehicle_pose_callback_(rov_msgs_package::VehiclePoseConstPtr pose_ptr);
  void sonar_callback_(rov_msgs_package::SonarDataConstPtr sonar_points_ptr);
  bool set_map_callback_(rov_msgs_package::CloudMapRequest& req, rov_msgs_package::CloudMapResponse& res);
};
