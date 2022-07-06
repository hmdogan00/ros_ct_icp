#include "types.h"

using ArrayVector3 = std::vector<Eigen::Matrix<double, 3, 1>, EIGEN_ALIGNED_ALLOCATOR<Eigen::Matrix<double, 3, 1>>>;
namespace slam {

    Eigen::Matrix<double, 3, 3> slam::SE3::Rotation() const {
        return quat.toRotationMatrix();
    }

    Eigen::Transform<double, 3, Eigen::Isometry> SE3::Isometry() const {
        return Eigen::Transform<double, 3, Eigen::Isometry>(Matrix().template block<3, 4>(0, 0));
    };

    SE3 SE3::Inverse() const {
        SE3 new_se3;
        new_se3.quat = quat.inverse();
        new_se3.tr = -(new_se3.quat * tr);
        return new_se3;
    }

    Eigen::Matrix<double, 4, 4> SE3::Matrix() const {
        Eigen::Matrix<double, 4, 4> mat = Eigen::Matrix<double, 4, 4>::Identity();
        mat.template block<3, 3>(0, 0) = quat.toRotationMatrix();
        mat.template block<3, 1>(0, 3) = tr;
        return mat;
    }

    SE3 SE3::operator*(const SE3& rhs) const {
        SE3 result;
        result.quat = quat * rhs.quat;
        result.quat.normalize();
        result.tr = quat.normalized() * rhs.tr + tr;
        return result;
    }

    Eigen::Matrix<double, 3, 1> SE3::operator*(const Eigen::Matrix<double, 3, 1>& point) const {
        Eigen::Matrix<double, 3, 1> result = quat.normalized() * point + tr;
        return result;
    }

    SE3 SE3::Interpolate(const SE3& other, double weight) const {
        SE3 result;
        result.quat = quat.slerp(weight, other.quat);
        result.tr = (double(1) - weight) * tr + weight * other.tr;
        return result;
    }

    const double& SE3::operator[](size_t param_idx) const {
        return const_cast<const double&>(const_cast<SE3&>(*this)[param_idx]);
    }

    double& SE3::operator[](size_t param_idx) {
        if (param_idx < 0 || param_idx > 6)
            throw std::range_error("SE3 only have 7 parameters, expects param idx in range [0, 6].");
        if (param_idx < 4)
            return quat.coeffs()[param_idx];
        return tr[param_idx - 4];
    }

    Eigen::Matrix<double, 7, 1> SE3::Parameters() const {
        Eigen::Matrix<double, 7, 1> vec;
        for (auto i(0); i < 7; ++i)
            vec(i) = operator[](i);
        return vec;
    }

    SE3 SE3::Random(double tr_scale, double quat_scale) {
        SE3 result;
        result.quat.coeffs() += Eigen::Vector4d::Random() * quat_scale;
        result.quat.normalize();
        result.tr = Eigen::Vector3d::Random() * tr_scale;
        return result;
    }

    Eigen::Matrix<double, 3, 1> Pose::ContinuousTransform(const Eigen::Matrix<double, 3, 1>& relative_point, const Pose& other_pose,
        double timestamp) const {
        Pose interpolated_pose = InterpolatePoseAlpha(other_pose, GetAlphaTimestamp(timestamp, other_pose));
        return interpolated_pose * relative_point;
    }

    Pose Pose::Inverse() const {
        Pose new_pose;
        new_pose.ref_frame_id = dest_frame_id;
        new_pose.ref_timestamp = dest_timestamp;
        new_pose.dest_frame_id = ref_frame_id;
        new_pose.dest_timestamp = ref_timestamp;
        new_pose.pose = pose.Inverse();
        return new_pose;
    }

    Pose Pose::InterpolatePoseAlpha(const Pose& other_pose, double alpha_timestamp, int new_dest_frame_id) const {
        CHECK(other_pose.ref_frame_id == ref_frame_id)
            << "Invalid operation: Cannot interpolate two frames not expressed in the same reference frame."
            << "ref_frame_id: " << ref_frame_id <<
            ", other_pose.ref_frame_id: " << other_pose.ref_frame_id
            << std::endl;
        Pose new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = dest_frame_id == other_pose.ref_frame_id ? dest_frame_id : new_dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = (double(1.0) - alpha_timestamp) * dest_timestamp +
            alpha_timestamp * other_pose.dest_timestamp;
        new_pose.pose = pose.Interpolate(other_pose.pose, alpha_timestamp);
        return new_pose;
    }

    Pose Pose::InterpolatePose(const Pose& other_pose, double timestamp, int new_dest_frame_id) const {
        // if (timestamp > other_pose.dest_timestamp) timestamp = std::fmod(timestamp, 10) / 10;
        CHECK(dest_timestamp <= timestamp && timestamp <= other_pose.dest_timestamp)
            << "The timestamp cannot be interpolated between the two poses" << std::endl;
        CHECK(other_pose.ref_frame_id == ref_frame_id)
            << "Invalid operation: Cannot interpolate two frames not expressed in the same reference frame."
            << "ref_frame_id: " << ref_frame_id <<
            ", other_pose.ref_frame_id: " << other_pose.ref_frame_id
            << std::endl;
        Pose new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = dest_frame_id == other_pose.ref_frame_id ? dest_frame_id : new_dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = timestamp;
        new_pose.pose = pose.Interpolate(other_pose.pose, GetAlphaTimestamp(timestamp, other_pose));
        return new_pose;
    }

    Eigen::Matrix<double, 4, 4> Pose::Matrix() const {
        Eigen::Matrix<double, 4, 4> Tr = Eigen::Matrix<double, 4, 4>::Identity();
        Tr.block<3, 3>(0, 0) = pose.quat.normalized().toRotationMatrix();
        Tr.block<3, 1>(0, 3) = pose.tr;
        return Tr;
    }

    Pose Pose::operator*(const Pose& rhs) const {
        CHECK(rhs.ref_frame_id == dest_frame_id)
            << "Invalid operation: Inconsistent reference frame for the Pose product. Got "
            << rhs.ref_frame_id << " and " << dest_frame_id << std::endl;
        CHECK(rhs.ref_timestamp == dest_timestamp)
            << "Invalid operation: Inconsistent reference timestamps for the Pose product";

        Pose new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = rhs.dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = rhs.dest_timestamp;
        new_pose.pose = pose * rhs.pose;
        return new_pose;
    }

    Eigen::Matrix<double, 3, 1> Pose::operator*(const Eigen::Matrix<double, 3, 1>& point) const {
        return pose * point;
    }

    Pose Pose::Identity() {
        Pose new_pose;
        new_pose.ref_frame_id = 0;
        new_pose.dest_frame_id = 0;
        new_pose.ref_timestamp = 0;
        new_pose.dest_timestamp = 0;
        return new_pose;
    }

    Pose Pose::Identity(double timestamp, int frame_id) {
        Pose identity;
        identity.pose = SE3();
        identity.ref_frame_id = frame_id;
        identity.dest_frame_id = frame_id;
        identity.dest_timestamp = timestamp;
        identity.ref_timestamp = timestamp;
        return identity;
    }

    Eigen::Transform<double, 3, Eigen::Isometry> Pose::Isometry() const {
        return pose.Isometry();
    }

    ArrayVector3 WorldPoints(const std::vector<pandar_ros::WPoint3D>& points) {
        ArrayVector3 data(points.size());
        for (auto i(0); i < points.size(); ++i) {
            data[i] = points[i].w_point;
        }
        return data;
    }

    ArrayVector3 RawPoints(const std::vector<pandar_ros::WPoint3D>& points) {
        ArrayVector3 data(points.size());
        for (auto i(0); i < points.size(); ++i) {
            Eigen::Vector3d vec(points[i].raw_point.x, points[i].raw_point.y, points[i].raw_point.z);
            data[i] = vec;
        }
        return data;
    }
}