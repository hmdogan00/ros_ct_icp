#ifndef CT_ICP_TYPES_HPP
#define CT_ICP_TYPES_HPP

#include <map>
#include <unordered_map>
#include <list>

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define _USE_MATH_DEFINES

#include <math.h>

namespace pandar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Point& operator=(const Point& other) {
            x = other.x;
            y = other.y;
            z = other.z;
            intensity = other.intensity;
            timestamp = other.timestamp;
            ring = other.ring;
            return *this;
        }
    };

    struct WPoint3D {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Point raw_point;
        Eigen::Vector3d w_point;
        double alpha_timestamp = 0.0;
        int frame_index = -1;

        WPoint3D() = default;

        WPoint3D& operator =(const Point& other) {
            raw_point = other;
            w_point = Eigen::Vector3d(raw_point.x, raw_point.y, raw_point.z);
            frame_index = -1;
            alpha_timestamp = 0.0;
            return *this;
        }
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

namespace slam {

    struct SE3 {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Quaternion<double> quat = Eigen::Quaternion<double>::Identity();
        Eigen::Matrix<double, 3, 1> tr = Eigen::Matrix<double, 3, 1>::Zero();

        SE3() = default;

        SE3(Eigen::Quaternion<double>&& quat, Eigen::Matrix<double, 3, 1>&& tr) : quat(quat.normalized()), tr(tr) {}

        SE3(const Eigen::Quaternion<double>& quat, const Eigen::Matrix<double, 3, 1>& tr) : quat(quat.normalized()), tr(tr) {}

        SE3 Inverse() const;
        inline Eigen::Matrix<double, 4, 4> Matrix() const;
        Eigen::Matrix<double, 3, 3> Rotation() const;
        inline Eigen::Transform<double, 3, Eigen::Isometry> Isometry() const;

        inline double& operator[](size_t param_idx);
        inline const double& operator[](size_t param_idx) const;

        //Right hand side matrix SE3 multiplication
        SE3 operator*(const SE3& rhs) const;

        // Given a raw 3D point 'x' captured from a LiDAR sensor at pose 'P'
        // coordinates of 'x' in the world frame is given by 'P * x'
        Eigen::Matrix<double, 3, 1> operator*(const Eigen::Matrix<double, 3, 1>& x) const;

        SE3 Interpolate(const SE3& other, double weight) const;

        inline Eigen::Matrix<double, 7, 1> Parameters() const;

        // Returns a random transformation
        static SE3 Random(double tr_scale = 1.0, double quat_scale = 1.0);
    };

    inline double AngularDistance(const Eigen::Matrix<double, 3, 3>& R1, const Eigen::Matrix<double, 3, 3>& R2) {
        double norm = ((R1 * R2.transpose()).trace() - 1.0) / 2.0;
        norm = ceres::fmax(ceres::fmin(norm, 1.0), -1.0);
        norm = ceres::acos(norm) * double(180.0 / M_PI);
        return norm;
    }

    inline double AngularDistance(const SE3& lhs, const SE3& rhs) {
        return AngularDistance(lhs.Rotation(), rhs.Rotation());
    }

    struct Pose {
        SE3 pose;
        double ref_timestamp = double(0);
        double dest_timestamp = double(-1);
        int ref_frame_id = 0;
        int dest_frame_id = -1;

        Pose() = default;

        explicit Pose(SE3&& _pose, double _timestamp = double(-1), int dest_frame_id = -1, double ref_timestamp = 0, int ref_frame_id = 0) :
            pose(std::move(_pose)), dest_timestamp(_timestamp), ref_timestamp(ref_timestamp), ref_frame_id(ref_frame_id), dest_frame_id(dest_frame_id) {}

        explicit Pose(const SE3& _pose, double _timestamp = -1.0, int dest_frame_id = -1, double ref_timestamp = 0, int ref_frame_id = 0) :
            pose(_pose), dest_timestamp(_timestamp), ref_timestamp(ref_timestamp), ref_frame_id(ref_frame_id), dest_frame_id(dest_frame_id) {}

        explicit Pose(Eigen::Quaternion<double>&& _quat, Eigen::Matrix<double, 3, 1>&& tr, double _timestamp = double(-1.0),
            int dest_frame_id = -1,
            double ref_timestamp = 0,
            int ref_frame_id = 0) : Pose(SE3(_quat, tr), _timestamp,
                dest_frame_id,
                ref_frame_id,
                ref_timestamp) {}

        double GetAlphaTimestamp(double mid_timestamp, const Pose& other) const {
            double min_timestamp = std::min(dest_timestamp, other.dest_timestamp);
            double max_timestamp = std::max(dest_timestamp, other.dest_timestamp);

            if (min_timestamp > mid_timestamp) return 0.0;
            if (max_timestamp < mid_timestamp) return 0.0;
            if (min_timestamp == max_timestamp) return 1.0;
            return (mid_timestamp - min_timestamp) / (max_timestamp - min_timestamp);
        }

        [[nodiscard]] Eigen::Matrix<double, 3, 1> ContinuousTransform(const Eigen::Matrix<double, 3, 1>& relative_point,
            const Pose& other_pose, double timestamp) const;

        [[nodiscard]] Pose InterpolatePoseAlpha(const Pose& other_pose, double alpha_timestamp,
            int new_dest_frame_id = -1) const;

        [[nodiscard]] Pose InterpolatePose(const Pose& other_pose, double timestamp,
            int new_dest_frame_id = -1) const;

        [[nodiscard]] Eigen::Matrix<double, 4, 4> Matrix() const;
        [[nodiscard]] Eigen::Transform<double, 3, Eigen::Isometry> Isometry() const;
        [[nodiscard]] Pose Inverse() const;
        Pose operator*(const Pose& rhs) const;
        Eigen::Matrix<double, 3, 1> operator*(const Eigen::Matrix<double, 3, 1>& point) const;

        Pose static Identity();
        Pose static Identity(double t, int frame_id);

        inline Eigen::Quaternion<double> &QuatRef() { return pose.quat; }

        inline const Eigen::Quaternion<double>& QuatConstRef() const { return pose.quat; }

        inline Eigen::Matrix<double, 3, 1>& TrRef() { return pose.tr; }

        inline const Eigen::Matrix<double, 3, 1>& TrConstRef() const { return pose.tr; }

        inline Eigen::Matrix<double, 3, 3> Rotation() const { return QuatConstRef().normalized().toRotationMatrix(); }

        double AngularDistance(const Pose& other) const {
            return slam::AngularDistance(pose, other.pose);
        }

        double LocationDistance(const Pose& other) const {
            return (TrConstRef() - other.TrConstRef()).norm();
        }
    };
}

namespace ct_icp {

    inline double AngularDistance(const Eigen::Matrix3d& rota,
        const Eigen::Matrix3d& rotb) {
        double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
        norm = std::acos(norm) * 180 / M_PI;
        return norm;
    }

    // A Trajectory Frame
    struct TrajectoryFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool success = true;
        double begin_timestamp = 0.0;
        double end_timestamp = 1.0;
        Eigen::Matrix3d begin_R;
        Eigen::Vector3d begin_t;
        Eigen::Matrix3d end_R;
        Eigen::Vector3d end_t;

        inline double EgoAngularDistance() const {
            return AngularDistance(begin_R, end_R);
        }

        double TranslationDistance(const TrajectoryFrame &other) {
            return (begin_t - other.begin_t).norm() + (end_t - other.end_t).norm();
        }

        double RotationDistance(const TrajectoryFrame &other) {
            return (begin_R * other.begin_R.inverse() - Eigen::Matrix3d::Identity()).norm() +
                   (end_R * other.end_R.inverse() - Eigen::Matrix3d::Identity()).norm();
        }

        TrajectoryFrame() = default;

        [[nodiscard]] inline Eigen::Matrix4d MidPose() const {
            Eigen::Matrix4d mid_pose = Eigen::Matrix4d::Identity();
            auto q_begin = Eigen::Quaterniond(begin_R);
            auto q_end = Eigen::Quaterniond(end_R);
            Eigen::Vector3d t_begin = begin_t;
            Eigen::Vector3d t_end = end_t;
            Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
            q.normalize();
            mid_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
            mid_pose.block<3, 1>(0, 3) = 0.5 * t_begin + 0.5 * t_end;
            return mid_pose;
        }
    };


    // Voxel
    // Note: Coordinates range is in [-32 768, 32 767]
    struct Voxel {

        Voxel() = default;

        Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

        bool operator==(const Voxel& vox) const { return x == vox.x && y == vox.y && z == vox.z; }

        inline bool operator<(const Voxel& vox) const {
            return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
        }

        inline static Voxel Coordinates(const Eigen::Vector3d& point, double voxel_size) {
            return { short(point.x() / voxel_size),
                    short(point.y() / voxel_size),
                    short(point.z() / voxel_size) };
        }

        short x;
        short y;
        short z;
    };

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ArrayVector3d;
    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> ArrayMatrix4d;
    typedef ArrayMatrix4d ArrayPoses;

    struct VoxelBlock {

        explicit VoxelBlock(int num_points = 20) : num_points_(num_points) { points.reserve(num_points); }

        ArrayVector3d points;

        bool IsFull() const { return num_points_ == points.size(); }

        void AddPoint(const Eigen::Vector3d& point) {
            CHECK(num_points_ >= points.size()) << "Voxel Is Full";
            points.push_back(point);
        }

        inline int NumPoints() const { return points.size(); }

        inline int Capacity() { return num_points_; }

        private:
        int num_points_;
    };

    typedef tsl::robin_map<Voxel, VoxelBlock> VoxelHashMap;


} // namespace Elastic_ICP

// Specialization of std::hash for our custom type Voxel
namespace std {


    template<>
    struct hash<ct_icp::Voxel> {
        std::size_t operator()(const ct_icp::Voxel& vox) const {
#ifdef CT_ICP_IS_WINDOWS
            const std::hash<int32_t> hasher;
            return ((hasher(vox.x) ^ (hasher(vox.y) << 1)) >> 1) ^ (hasher(vox.z) << 1) >> 1;
#else
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
#endif
        }
    };
}

#endif //CT_ICP_TYPES_HPP
