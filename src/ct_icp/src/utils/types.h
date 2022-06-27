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

        inline SE3 Inverse() const;
        inline Eigen::Matrix<double, 4, 4> Matrix() const;
        inline Eigen::Matrix<double, 3, 3> RotationMatrix() const;
        inline Eigen::Transform<double, 3, Eigen::Isometry> Isometry() const;

        inline double& operator[](size_t param_idx);
        inline const double& operator[](size_t param_idx) const;

        //Right hand side matrix SE3 multiplication
        inline SE3 operator*(const SE3& rhs) const;

        // Given a raw 3D point 'x' captured from a LiDAR sensor at pose 'P'
        // coordinates of 'x' in the world frame is given by 'P * x'
        inline Eigen::Matrix<double, 3, 1> operator*(const Eigen::Matrix<double, 3, 1>& x) const;

        SE3 Interpolate(const SE3& other, double weight) const;

        inline Eigen::Matrix<double, 7, 1> Parameters() const;

        // Returns a random transformation
        static SE3 Random(double tr_scale = 1.0, double quat_scale = 1.0);
    };

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

    };

    inline double AngularDistance(const Eigen::Matrix<double, 3, 3>& R1, const Eigen::Matrix<double, 3, 3>& R2) {
        double norm = ((R1 * R2.transpose()).trace() - 1.0) / 2.0;
        norm = ceres::fmax(ceres::fmin(norm, 1.0), -1.0);
        norm = ceres::acos(norm) * double(180.0 / M_PI);
        return norm;
    }

    inline double AngularDistance(const SE3 &lhs, const SE3 &rhs) {
        return AngularDistance(lhs.RotationMatrix(), rhs.RotationMatrix());
    }
}

namespace ct_icp {
    // A Point3D
    struct Point3D {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Eigen::Vector3d raw_pt; // Raw point read from the sensor
        Eigen::Vector3d pt; // Corrected point taking into account the motion of the sensor during frame acquisition
        double alpha_timestamp = 0.0; // Relative timestamp in the frame in [0.0, 1.0]
        double timestamp = 0.0; // The absolute timestamp (if applicable)
        int index_frame = -1; // The frame index

        Point3D() = default;
    };

    inline double AngularDistance(const Eigen::Matrix3d& rota,
        const Eigen::Matrix3d& rotb) {
        double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
        norm = std::acos(norm) * 180 / M_PI;
        return norm;
    }

    // A Trajectory Frame
    struct TrajectoryFrame {
        slam::Pose begin_pose, end_pose;

        inline double EgoAngularDistance() const {
            return slam::AngularDistance
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
