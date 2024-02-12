#pragma once

#include <Eigen/Geometry>

namespace rov_localization {

struct Transform {
    Eigen::Matrix4f to_eigen() const;
    static Transform from_eigen(const Eigen::Matrix4f& matrix);

    Transform inverse_transform() const;

    Eigen::Quaternionf rotation{Eigen::Quaternionf::Identity()};
    Eigen::Translation3f translation{Eigen::Translation3f::Identity()};
};

Transform operator*(const Transform& left, const Transform& right);
} // end namespace rov_localization
