#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <hydroacoustic_localization/utils.h>
#include <hydroacoustic_localization/types.h>
#include <hydroacoustic_localization/transform.h>
#include <hydroacoustic_localization/scan_mather_manager.h>

#include <rov_msgs_package/VehiclePose.h>

namespace rov_localization {

namespace {

inline Transform from_ros(rov_msgs_package::VehiclePoseConstPtr ros_pose_ptr) {
    Transform res;
    res.translation.x() = ros_pose_ptr->vehicle_point.x;
    res.translation.y() = ros_pose_ptr->vehicle_point.y;
    res.translation.z() = ros_pose_ptr->vehicle_point.z;

    res.rotation.x() = ros_pose_ptr->vehicle_orientation.x;
    res.rotation.y() = ros_pose_ptr->vehicle_orientation.y;
    res.rotation.z() = ros_pose_ptr->vehicle_orientation.z;
    res.rotation.w() = ros_pose_ptr->vehicle_orientation.w;

    return res;
}

inline rov_msgs_package::VehiclePose to_ros(Transform transform) {
    rov_msgs_package::VehiclePose ros_pose;

    ros_pose.vehicle_point.x = transform.translation.x();
    ros_pose.vehicle_point.y = transform.translation.y();
    ros_pose.vehicle_point.z = transform.translation.z();

    ros_pose.vehicle_orientation.x = transform.rotation.x();
    ros_pose.vehicle_orientation.y = transform.rotation.y();
    ros_pose.vehicle_orientation.z = transform.rotation.z();
    ros_pose.vehicle_orientation.w = transform.rotation.w();

    return ros_pose;
}

inline auto from_ros(const sensor_msgs::PointCloud2& scan)
{
    std::shared_ptr<PointCloud> point_cloud = std::make_shared<PointCloud>();
    pcl::PCLPointCloud2 temporary_point_cloud;
    temporary_point_cloud.header.seq = scan.header.seq;
    temporary_point_cloud.header.frame_id = scan.header.frame_id;
    temporary_point_cloud.header.stamp = scan.header.stamp.toNSec() / 1000ull;
    temporary_point_cloud.height = scan.height;
    temporary_point_cloud.width = scan.width;
    temporary_point_cloud.fields.resize(scan.fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = scan.fields.begin();
    for (uint64_t i = 0; it != scan.fields.end(); ++it, ++i) {
        temporary_point_cloud.fields[i].name = it->name;
        temporary_point_cloud.fields[i].offset = it->offset;
        temporary_point_cloud.fields[i].datatype = it->datatype;
        temporary_point_cloud.fields[i].count = it->count;
    }
    temporary_point_cloud.is_bigendian = scan.is_bigendian;
    temporary_point_cloud.point_step = scan.point_step;
    temporary_point_cloud.row_step = scan.row_step;
    temporary_point_cloud.is_dense = scan.is_dense;
    temporary_point_cloud.data = scan.data;
    pcl::fromPCLPointCloud2(temporary_point_cloud, *point_cloud);

    return point_cloud;
}
} // end namespace

class HydroacousticLocalizationNode {
public:

    HydroacousticLocalizationNode() {
        ros::NodeHandle nh;

        auto package_path = ros::package::getPath(ROS_PACKAGE_NAME);
        auto config_path = package_path + "/config/localization_config.yaml";
        auto map_path = package_path + "/map/map.pcd";
        auto params = load_config(config_path);
        auto map = load_map(map_path);

        mather_manager_ = std::make_unique<ScanMatherManager>(params);
        mather_manager_->set_map(map);

        prior_pose_sub_ = nh.subscribe("/localization/prior_pose", 1,
                                       &HydroacousticLocalizationNode::prior_pose_callback_, this);
        prior_pose_sub_ = nh.subscribe("/sensors/hydroacoustic", 1,
                                       &HydroacousticLocalizationNode::scan_callback_, this);
        localization_pub_ = nh.advertise<rov_msgs_package::VehiclePose>("/localization/ndt_localization", 1);

    }

    void spin_once() {
        if (!set_new_scan_) {
            return;
        }

        try {
            localization_pub_.publish(to_ros(mather_manager_->step()));
        } catch (const std::exception &exp) {
            ROS_WARN(exp.what());
        }
    }

private:
    std::unique_ptr<ScanMatherManager> mather_manager_;
    bool set_new_scan_ = false;
    Transform prior_pose_;

    ros::Subscriber prior_pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher localization_pub_;

    void prior_pose_callback_(rov_msgs_package::VehiclePoseConstPtr pose_ptr) {
        prior_pose_ = from_ros(pose_ptr);
        mather_manager_->set_prior_pose(prior_pose_);
    }

    void scan_callback_(const sensor_msgs::PointCloud2& scan) {
        mather_manager_->set_scan(from_ros(scan));
    }
};
} // end namespace rov_localization

int main(int argc, char **argv) {
    ros::init(argc, argv, "parking_planner_node");
    rov_localization::HydroacousticLocalizationNode hydro_localization_node;
    ros::Rate rate(10);
    setvbuf(stdout, nullptr, _IOLBF, 4096);
    while (ros::ok()) {
        try {
            hydro_localization_node.spin_once();
            ros::spinOnce();
            rate.sleep();
            fflush(stdout);
        } catch (const std::exception &e) {
            ROS_ERROR(e.what());
            return 1;
        }
    }
    return 0;
}
