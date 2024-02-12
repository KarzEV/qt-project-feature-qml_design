#include <hydroacoustic_localization/mather_shell.h>

namespace rov_localization {
MatherShell::MatherShell(const MatherParams &params) {
    setStepSize(params.optimization_step);
    setResolution(params.grid_cell_size);
    setTransformationEpsilon(params.optimization_treshold);
    setMaximumIterations(static_cast<int>(params.optimization_iterations));
}

const Transform& MatherShell::math_scan() {
    if(!prior_pose_) {
        throw std::runtime_error("ScanMatherManager: prior pose not set");
    }

    align(*aligned_scan_, prior_pose_->to_eigen());
    last_pose_ = Transform::from_eigen(getFinalTransformation());

    if(!is_transform_found()) {
        last_pose_ = prior_pose_;
    }

    return last_pose_.value();
}

} // end namespace rov_localization
