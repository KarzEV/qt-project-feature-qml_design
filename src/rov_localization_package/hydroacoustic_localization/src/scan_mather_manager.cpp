#include <hydroacoustic_localization/scan_mather_manager.h>

#include <exception>

namespace rov_localization {

ScanMatherManager::ScanMatherManager(const ManagerParams& manager_params):
        filter_enabled_(manager_params.filter_enabled),
        match_solver_(std::make_shared<MatherShell>(manager_params.mather_params)) {
    diagnostic_extractor_ = std::make_unique<DiagnosticExtractor>(match_solver_);

    if (filter_enabled_) {
        scan_filter_ = std::make_unique<VoxelGrid>();
        scan_filter_->setLeafSize(manager_params.filter_cell_size, manager_params.filter_cell_size,
                                  manager_params.filter_cell_size);
    }
}

void ScanMatherManager::set_map(const std::shared_ptr<PointCloud>& point_cloud) {
    match_solver_->setInputTarget(point_cloud);
    diagnostic_extractor_->set_map(point_cloud);
}

Transform ScanMatherManager::step() {
    set_scan_();
    return match_solver_->math_scan();
}

void ScanMatherManager::set_scan_() {
    if(filter_enabled_) {
        auto filtered_scan = std::make_shared<PointCloud>();
        scan_filter_->setInputCloud(lidar_scan_);
        scan_filter_->filter(*filtered_scan);
        match_solver_->setInputSource(std::move(filtered_scan));
    } else {
        match_solver_->setInputSource(lidar_scan_);
    }
}

} // end namespace rov_localization

