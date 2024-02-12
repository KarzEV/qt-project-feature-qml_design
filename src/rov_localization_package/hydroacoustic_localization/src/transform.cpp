#include <hydroacoustic_localization/transform.h>

namespace rov_localization {

Eigen::Matrix4f Transform::to_eigen() const {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    Eigen::Affine3f transform_rotation = Eigen::Affine3f(rotation);
    matrix = (translation * transform_rotation).matrix();

    return matrix;
}

Transform Transform::from_eigen(const Eigen::Matrix4f &matrix) {
    Transform transform;

    transform.rotation = matrix.block<3, 3>(0, 0);
    transform.rotation = transform.rotation.normalized();

    transform.translation = Eigen::Translation3f(matrix.topRightCorner(3, 1));

    return transform;
}

Transform Transform::inverse_transform() const {
    Transform inverse_result;
    inverse_result.rotation = rotation.normalized().conjugate();
    inverse_result.translation = Eigen::Translation3f(
            (inverse_result.rotation * translation.inverse()).translation());

    return inverse_result;
}

Transform operator*(const Transform &left, const Transform &right) {
    Transform result_transform;
    result_transform.rotation = (left.rotation * right.rotation).normalized();
    result_transform.translation.vector() = (left.rotation * right.translation).translation();
    result_transform.translation.vector() += left.translation.vector();

    return result_transform;
}
} // end namespace rov_localization
