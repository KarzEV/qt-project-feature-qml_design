#pragma once

#include "types.h"
#include "transform.h"

namespace rov_localization {

class MatherShell: public NDTSolver {
public:
    MatherShell() = delete;
    ~MatherShell() = default;
    MatherShell(const MatherShell&) = delete;
    MatherShell& operator=(const MatherShell&) = delete;
    MatherShell(MatherShell&&) = delete;
    MatherShell& operator=(MatherShell&&) = delete;
    explicit MatherShell(const MatherParams &params);

    inline void set_prior_pose(const Transform& prior_pose) {
        prior_pose_ = prior_pose;
    }

    [[nodiscard]] inline const Transform& get_prior_transform() const {return prior_pose_.value(); }
    [[nodiscard]] const Transform& get_current_transform() const {return last_pose_.value(); }
    [[nodiscard]] const PointCloud& get_aligned_scan() const {return *aligned_scan_;}

    [[nodiscard]] const Transform& math_scan();
    [[nodiscard]] inline bool is_transform_found() const {return hasConverged();}

private:
    std::shared_ptr<PointCloud> aligned_scan_;
    std::optional<Transform> prior_pose_;
    std::optional<Transform> last_pose_;
};
} // end namespace rov_localization
