#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H

#include "../cost_functions/cost_func.h"
#include "../utils/types.h"

#include <iostream>
#include <string>
#include <list>
#include <pcl/point_cloud.h>

#define _USE_MATH_DEFINES

#include <fstream>
#include <vector>
#include <random>



namespace ct_icp {

    enum MOTION_COMPENSATION {
        NONE = 0,              // No compensation of the motion
        CONSTANT_VELOCITY = 1, // Initial distortion of the point cloud based on the estimated velocity
        ITERATIVE = 2,         // Iterative refinement after each ICP iteration
        CONTINUOUS = 3         // Continuous estimation of the pose
    };

    enum INITIALIZATION {
        INIT_NONE = 0,
        INIT_CONSTANT_VELOCITY = 1
    };

    enum CT_ICP_SOLVER {
        GN = 0,
        CERES = 1
    };

    enum LEAST_SQUARES {
        STANDARD = 0,
        CAUCHY = 1,
        HUBER = 2,
        TOLERANT = 3,
        TRUNCATED = 4
    };

    enum WEIGHTING_SCHEME {
        PLANARITY,      // Weighs residuals by their planarity coefficient
        NEIGHBORHOOD,   // Weighs residuals by the distance to their neares neighbors
        ALL          // Combines all weighting schemes with different coefficients
    };

    struct CTICPOptions {

        // The threshold on the voxel occupancy
        // To be considered in the neighbor search, a voxel must have at least threshold_voxel_occupancy points
        int threshold_voxel_occupancy = 1;

        int init_num_frames = 20; // The number of frames defining the initialization of the map

        double size_voxel_map = 1.0; //Max Voxel : -32767 to 32767 then 32km map for SIZE_VOXEL_MAP = 1m

        int num_iters_icp = 5; // The Maximum number of ICP iterations performed

        int min_number_neighbors = 20;

        int voxel_neighborhood = 1; // Visits the (3 * voxel_neighborhood)^3 neighboring voxels

        double power_planarity = 2.0; // The power of planarity defined in the weighting scheme

        // Whether to estimate the normal of the key point or the closest neighbor
        bool estimate_normal_from_neighborhood = true;

        int max_number_neighbors = 20;

        double max_dist_to_plane_ct_icp = 0.3; // The maximum distance point-to-plane (OLD Version of ICP)

        double threshold_orientation_norm = 0.0001; // Threshold on rotation (deg) for ICP's stopping criterion

        double threshold_translation_norm = 0.001; // Threshold on translation (deg) for ICP's stopping criterion

        bool point_to_plane_with_distortion = true; // Whether to distort the frames at each ICP iteration

        int max_num_residuals = -1; // The maximum number of keypoints used

        int min_num_residuals = 100; // Below this number, CT_ICP will crash

        ICP_DISTANCE distance = CT_POINT_TO_PLANE;

        int num_closest_neighbors = 1; // The number of closest neighbors considered as residuals

        // TODO : Add Trajectory Constraints Options
        double beta_location_consistency = 0.001; // Constraints on location

        double beta_constant_velocity = 0.001; // Constraint on velocity

        double beta_small_velocity = 0.0; // Constraint on the relative motion

        double beta_orientation_consistency = 0.0; // Constraint on the orientation consistency

        WEIGHTING_SCHEME weighting_scheme = ALL;

        double weight_alpha = 0.9;

        double weight_neighborhood = 0.1;

        CT_ICP_SOLVER solver = GN;

        /* ---------------------------------------------------------------------------------------------------------- */
        /* LEAST SQUARE OPTIMIZATION PARAMETERS                                                                       */

        LEAST_SQUARES loss_function = CAUCHY;

        int ls_max_num_iters = 1;

        int ls_num_threads = 16;

        double ls_sigma = 0.1; // The robust parameter (for Cauchy, Huber or truncated least square)

        double ls_tolerant_min_threshold = 0.05; // The Tolerant

        // Debug params
        bool debug_print = true; // Whether to output debug information to std::cout

    };

    struct OdometryOptions {

        /* Parameters for initialization of the map */
        double init_voxel_size = 0.2;

        double init_sample_voxel_size = 1.0;

        int init_num_frames = 20; // The number of frames defining the initialization of the map

        double voxel_size = 0.5;

        double sample_voxel_size = 1.5;

        double max_distance = 100.0; // The threshold on the voxel size to remove points from the map

        int max_num_points_in_voxel = 20; // The maximum number of points in a voxel

        double min_distance_points = 0.1; // The minimal distance between points in the map

        double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error

        // Whether to assess the quality of the registration,
        // And try a new registration with more conservative parameters in case of failure
        int robust_minimal_level = 0;
        bool robust_registration = false;
        double robust_full_voxel_threshold = 0.7;
        double robust_empty_voxel_threshold = 0.1;
        double robust_neighborhood_min_dist = 0.10; // The minimum relative distance to launch a robust neighborhood test
        double robust_neighborhood_min_orientation = 0.1; // The minimum relative orientation to launch a robust neighborhood
        // Threshold on the relative transform (all motion at 10Hz should be below this value)
        double robust_relative_trans_threshold = 1.0;
        bool robust_fail_early = false; // Stop iterations if the final assessment of the registration is unsucessful
        int robust_num_attempts = 6;
        int robust_num_attempts_when_rotation = 2;
        int robust_max_voxel_neighborhood = 3;
        double robust_threshold_ego_orientation = 3; // Angle in degrees
        double robust_threshold_relative_orientation = 3; // Angle in degrees

        CTICPOptions ct_icp_options;

        MOTION_COMPENSATION motion_compensation = CONTINUOUS;

        INITIALIZATION initialization = INIT_CONSTANT_VELOCITY;


        // Debug Parameters
        bool debug_print = true; // Whether to print debug information into the console

        bool debug_viz = false; // Whether to display the Local Map in a window

        bool log_to_file = false;

        std::string log_file_destination = "/tmp/ct_icp.log";
    };

    // Add Points To the Map
    void AddPointsToMap(VoxelHashMap& map, const std::vector<pandar_ros::WPoint3D>& points,
        double voxel_size, int max_num_points_in_voxel,
        double min_distance_points, int min_num_points = 0);

    // Add Points To the Map
    void AddPointsToMap(VoxelHashMap& map, const ArrayVector3d& points, double voxel_size,
        int max_num_points_in_voxel, double min_distance_points);

    // Remove voxels far from the given location
    void RemovePointsFarFromLocation(VoxelHashMap& map, const Eigen::Vector3d& location, double distance);

    // Extracts points of the local map into a PointCloud
    ArrayVector3d MapAsPointcloud(const VoxelHashMap& map);

    // Compute the size of a VoxelHashMap
    size_t MapSize(const VoxelHashMap &map);


    class Odometry {
        public:
        struct FrameInfo {
            int registered_fid = -1; // The index of the new frame (since the initial insertion of the frame)
            int frame_id = -1; // The frame index
            double begin_timestamp = -1., end_timestamp = -1.;
        };
        // The Output of a registration, including metrics,
        struct RegistrationSummary {

            TrajectoryFrame frame;

            int sample_size = 0; // The number of points sampled

            int number_of_residuals = 0; // The number of keypoints used for ICP registration

            int robust_level = 0;

            double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

            double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

            double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

            double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

            bool success = true; // Whether the registration was a success

            int number_of_attempts = 0; // The number of attempts at registering the new frame

            std::string error_message;

            std::vector<pandar_ros::WPoint3D> corrected_points; // Sampled points expressed in the initial frame

            std::vector<pandar_ros::WPoint3D> all_corrected_points; // Initial points expressed in the initial frame

            std::vector<pandar_ros::WPoint3D> keypoints; // Last Keypoints selected

        };

        explicit Odometry(const OdometryOptions& options);

        // Registers a new Frame to the Map
        RegistrationSummary RegisterFrame(const pcl::PointCloud<pandar_ros::Point>& frame, const std::vector<double>& timestamp_vector);

        // Registers a new Frame to the Map with an initial estimate
        RegistrationSummary RegisterFrameWithEstimate(const pcl::PointCloud<pandar_ros::Point>& frame,
            const TrajectoryFrame& initial_estimate);

        // Returns the currently registered trajectory
        [[nodiscard]] std::vector<TrajectoryFrame> Trajectory() const;

        // Returns the Aggregated PointCloud of the Local Map
        [[nodiscard]] ArrayVector3d GetLocalMap() const;

        // Num Points in the Map
        // Note: This requires a traversal of the whole map which is in O(n)
        [[nodiscard]] size_t MapSize() const;

        private:
        std::vector<TrajectoryFrame> trajectory_;
        VoxelHashMap voxel_map_;
        int registered_frames_ = 0;
        int robust_num_consecutive_failures_ = 0;
        bool suspect_registration_error_ = false;
        int next_robust_level_ = 0;
        OdometryOptions options_;
        std::ostream* log_out_ = nullptr;
        std::unique_ptr<std::ofstream> log_file_ = nullptr;

        // Initialize the Frame
        std::vector<pandar_ros::WPoint3D> InitializeFrame(const pcl::PointCloud<pandar_ros::Point>& const_frame,
            FrameInfo frame_info);

        // Registers a frame after the motion was initialized
        // When the Robust Registration profile is activated, it can call TryRegister
        // Multiple times changing the options in order to increase the chance of registration
        RegistrationSummary DoRegister(const pcl::PointCloud<pandar_ros::Point>& const_frame, FrameInfo frame_info);

        // Tries to register a frame given a set of options
        RegistrationSummary TryRegister(std::vector<pandar_ros::WPoint3D> &frame,
            FrameInfo frame_info, CTICPOptions& options,
            RegistrationSummary& registration_summary,
            double sample_voxel_size);

        // Insert a New Trajectory Frame, and initializes the motion for this new frame
        void InitializeMotion(FrameInfo frame_info, const TrajectoryFrame* initial_estimate = nullptr);

        // Try to insert Points to the map
        // Returns false if it fails
        bool AssessRegistration(const std::vector<pandar_ros::WPoint3D>& points, RegistrationSummary& summary) const;

    };

} // namespace ct_icp

#endif