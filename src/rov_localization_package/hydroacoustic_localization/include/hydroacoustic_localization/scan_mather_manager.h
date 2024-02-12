#pragma once

#include <optional>
#include <memory>

#include "types.h"
#include "transform.h"
#include "diagnostic_extractor.h"
#include "mather_shell.h"

namespace rov_localization {

class ScanMatherManager {
public:
    ScanMatherManager() = delete;
    ~ScanMatherManager() = default;
    ScanMatherManager(const ScanMatherManager&) = delete;
    ScanMatherManager& operator=(const ScanMatherManager&) = delete;
    ScanMatherManager(ScanMatherManager&&) = delete;
    ScanMatherManager& operator=(ScanMatherManager&&) = delete;

    explicit ScanMatherManager(const ManagerParams& manager_params);

    inline void set_scan(std::shared_ptr<PointCloud> point_cloud) {
        lidar_scan_ = std::move(point_cloud);
    }

    void set_map(const std::shared_ptr<PointCloud>& point_cloud);

    inline void set_prior_pose(const Transform& prior_pose) {
        match_solver_->set_prior_pose(prior_pose);
    }

    [[nodiscard]] Transform step();
    [[nodiscard]] inline Metrics get_metrics() const {
        return diagnostic_extractor_->get_metrics();
    }

private:
    bool filter_enabled_ = false;
    std::shared_ptr<PointCloud> lidar_scan_;

    std::shared_ptr<MatherShell> match_solver_;
    std::unique_ptr<DiagnosticExtractor> diagnostic_extractor_;
    std::unique_ptr<VoxelGrid> scan_filter_;

    void set_scan_();
    [[nodiscard]] Transform math_scan_();
};
} // end namespace rov_localization
