#include <rov_gui_package/shell.h>

#include <QApplication>

#include <rov_gui_package/utils.h>

namespace {

QVector<QVector3D> convert_cloud(const std::vector<geometry_msgs::Point32>& ros_points) {
  QVector<QVector3D> points;

  for(const auto& point: ros_points) {
    points.push_back({point.x, point.y, point.z});
  }

  return points;
}
} // end namespace

void RosGuiShell::init() {
  main_window_.setWindowTitle("Cloud Tool");
  connect(&timer_, &QTimer::timeout, this, &RosGuiShell::sipn_once);

  ros::NodeHandle nh;

  nh.param<bool>("debug_load", debug_load_, true);

  vehicle_pose_sub_ = nh.subscribe("/vehicle/pose", 1, &RosGuiShell::vehicle_pose_callback_, this);
  sonar_data_sub_ = nh.subscribe("/sensors/sonar", 1, &RosGuiShell::sonar_callback_, this);

  map_srv_ = nh.advertiseService("/map", &RosGuiShell::set_map_callback_, this);

  timer_.start(100);
}

void RosGuiShell::show() {
  main_window_.show();

  if(debug_load_) {
    set_test_data_();
  }
}

void RosGuiShell::sipn_once() {
  if(ros::ok()){
    ros::spinOnce();
  } else {
    QApplication::quit();
  }
}

void RosGuiShell::set_test_data_() {
    const QVector<QVector3D> TEST_MAP_POINTS = parse_CSV(":/resources/test_map.csv");
    const QVector<QVector3D> TEST_SONAR_POINTS = parse_CSV(":/resources/test_sonar.csv");
    const QVector3D pose = {0.0, -5.0, 0.0};

    main_window_.draw_map(TEST_MAP_POINTS);
    main_window_.draw_sonar_data(TEST_SONAR_POINTS);
    main_window_.draw_vehicle(pose);
}

void RosGuiShell::vehicle_pose_callback_(rov_msgs_package::VehiclePoseConstPtr pose_ptr) {
  const QVector3D pose = {pose_ptr->vehicle_point.x,
                          pose_ptr->vehicle_point.y,
                          pose_ptr->vehicle_point.z};
  const QQuaternion quaternion = {pose_ptr->vehicle_orientation.w,
                                  pose_ptr->vehicle_orientation.x,
                                  pose_ptr->vehicle_orientation.y,
                                  pose_ptr->vehicle_orientation.z};

  main_window_.draw_vehicle(pose, quaternion);
}

void RosGuiShell::sonar_callback_(rov_msgs_package::SonarDataConstPtr sonar_points_ptr) {
  main_window_.draw_sonar_data(convert_cloud(sonar_points_ptr->points));
}

bool RosGuiShell::set_map_callback_(rov_msgs_package::CloudMapRequest& req, rov_msgs_package::CloudMapResponse& res) {
  main_window_.draw_map(convert_cloud(req.points));
}
