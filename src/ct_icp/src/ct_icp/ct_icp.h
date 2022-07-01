#ifndef CT_ICP_CT_ICP_H
#define CT_ICP_CT_ICP_H

#include <iostream>
#include <string>

#define _USE_MATH_DEFINES

#include <vector>
#include <random>
#include "../utils/types.h"
#include "../cost_functions/cost_func.h"

namespace ct_icp {
    
    void sub_sample_frame(std::vector<pandar_ros::WPoint3D> &frame, double size_voxel);

    void grid_sampling(const std::vector<pandar_ros::WPoint3D> &frame, std::vector<pandar_ros::WPoint3D> &keypoints, double size_voxel_subsampling);

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

    struct ICPSummary {
        bool success = false; // Whether the registration succeeded
        int num_residuals_used = 0;
    };

    class CT_ICP_Registration {
        public:
            CTICPOptions &Options() { return options_; }

            ICPSummary Register(const VoxelHashMap &voxel_map,
                                std::vector<pandar_ros::WPoint3D> &keypoints,
                                TrajectoryFrame &t_frame,
                                const TrajectoryFrame *const previous_frame = nullptr);
            
            ICPSummary Register(const VoxelHashMap &voxel_map,
                                pcl::PointCloud<pandar_ros::WPoint3D> &keypoints,
                                TrajectoryFrame &t_frame,
                                const TrajectoryFrame *const previous_frame = nullptr);
        private:
            ICPSummary DoRegisterCeres(const VoxelHashMap &voxel_map,
                                        std::vector<Eigen::Vector3d> &raw_keypts,
                                        std::vector<Eigen::Vector3d> &world_keypts,
                                        std::vector<double> &timestamps,
                                        TrajectoryFrame &t_frame,
                                        const TrajectoryFrame *const previous_frame = nullptr);
            
            
            ICPSummary DoRegisterGN(const VoxelHashMap &voxel_map,
                                        std::vector<Eigen::Vector3d> &raw_keypts,
                                        std::vector<Eigen::Vector3d> &world_keypts,
                                        std::vector<double> &timestamps,
                                        TrajectoryFrame &t_frame,
                                        const TrajectoryFrame *const previous_frame = nullptr);

            CTICPOptions options_;
    };
}
#endif