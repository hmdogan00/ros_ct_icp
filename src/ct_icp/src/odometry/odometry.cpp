#ifndef CT_ICP_ODOMETRY_HPP
#define CT_ICP_ODOMETRY_HPP
#include "odometry.h"

namespace ct_icp {
    size_t Odometry::MapSize() const {
        return ::ct_icp::MapSize(voxel_map_);
    }

    void DistortFrame(std::vector<pandar_ros::WPoint3D>& points, Eigen::Quaterniond &begin_quat, Eigen::Quaterniond &end_quat,
                      Eigen::Vector3d &begin_t, Eigen::Vector3d &end_t) {
        Eigen::Quaterniond end_quat_I = end_quat.inverse(); // Rotation of the inverse pose
        Eigen::Vector3d end_t_I = -1.0 * (end_quat_I * end_t); // Translation of the inverse pose
        for (auto &point: points) {
            double alpha_timestamp = point.alpha_timestamp;
            Eigen::Quaterniond q_alpha = begin_quat.slerp(alpha_timestamp, end_quat);
            q_alpha.normalize();
            Eigen::Vector3d t = (1.0 - alpha_timestamp) * begin_t + alpha_timestamp * end_t;

            // Distort Raw Keypoints
            Eigen::Vector3d p = end_quat_I * (q_alpha * Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z) + t) + end_t_I;
            point.raw_point.x = p.x();
            point.raw_point.y = p.y();
            point.raw_point.z = p.z();
        }
    }

    inline void TransformPoint(MOTION_COMPENSATION comp, pandar_ros::WPoint3D p, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                               Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end) {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;
        double alpha_timestamp = p.alpha_timestamp;
        switch (comp) {
            case MOTION_COMPENSATION::NONE:
            case MOTION_COMPENSATION::CONSTANT_VELOCITY:
                R = q_end.toRotationMatrix();
                t = t_end;
                break;
            case MOTION_COMPENSATION::CONTINUOUS:
            case MOTION_COMPENSATION::ITERATIVE:
                R = q_begin.slerp(alpha_timestamp, q_end).normalized().toRotationMatrix();
                t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                break;
        }
        p.w_point = R * Eigen::Vector3d(p.raw_point.x, p.raw_point.y, p.raw_point.z) + t;
    }

    Odometry::RegistrationSummary Odometry::RegisterFrameWithEstimate(const pcl::PointCloud<pandar_ros::Point> &frame,
                                                                      const TrajectoryFrame &initial_estimate) {
        auto frame_index = InitializeMotion(&initial_estimate);
        return DoRegister(frame, frame_index);
    }

    Odometry::RegistrationSummary Odometry::RegisterFrame(const pcl::PointCloud<pandar_ros::Point>& frame) {
        int frame_index = InitializeMotion();
        return DoRegister(frame, frame_index);
    }

    
    int Odometry::InitializeMotion(const TrajectoryFrame* initial_estimate) {
                int index_frame = registered_frames_++;
        if (initial_estimate != nullptr) {
            // Insert previous estimate
            trajectory_.emplace_back(*initial_estimate);
            return index_frame;
        }

        // Initial Trajectory Estimate
        trajectory_.emplace_back(TrajectoryFrame());

        if (index_frame <= 1) {
            // Initialize first pose at Identity
            trajectory_[index_frame].begin_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].begin_t = Eigen::Vector3d(0., 0., 0.);
            trajectory_[index_frame].end_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].end_t = Eigen::Vector3d(0., 0., 0.);
        } else if (index_frame == 2) {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                // Different regimen for the second frame due to the bootstrapped elasticity
                Eigen::Matrix3d R_next_end =
                        trajectory_[index_frame - 1].end_R * trajectory_[index_frame - 2].end_R.inverse() *
                        trajectory_[index_frame - 1].end_R;
                Eigen::Vector3d t_next_end = trajectory_[index_frame - 1].end_t +
                                             trajectory_[index_frame - 1].end_R *
                                             trajectory_[index_frame - 2].end_R.inverse() *
                                             (trajectory_[index_frame - 1].end_t -
                                              trajectory_[index_frame - 2].end_t);

                trajectory_[index_frame].begin_R = trajectory_[index_frame - 1].end_R;
                trajectory_[index_frame].begin_t = trajectory_[index_frame - 1].end_t;
                trajectory_[index_frame].end_R = R_next_end;
                trajectory_[index_frame].end_t = t_next_end;
            } else {
                // Important ! Start with a rigid frame and let the ICP distort it !
                trajectory_[index_frame] = trajectory_[index_frame - 1];
                trajectory_[index_frame].end_t = trajectory_[index_frame].begin_t;
                trajectory_[index_frame].end_R = trajectory_[index_frame].begin_R;
            }
        } else {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                if (options_.motion_compensation == CONTINUOUS) {
                    // When continuous: use the previous begin_pose as reference
                    Eigen::Matrix3d R_next_begin =
                            trajectory_[index_frame - 1].begin_R * trajectory_[index_frame - 2].begin_R.inverse() *
                            trajectory_[index_frame - 1].begin_R;
                    Eigen::Vector3d t_next_begin = trajectory_[index_frame - 1].begin_t +
                                                   trajectory_[index_frame - 1].begin_R *
                                                   trajectory_[index_frame - 2].begin_R.inverse() *
                                                   (trajectory_[index_frame - 1].begin_t -
                                                    trajectory_[index_frame - 2].begin_t);

                    trajectory_[index_frame].begin_R = trajectory_[index_frame - 1].end_R; //R_next_begin;
                    trajectory_[index_frame].begin_t = trajectory_[index_frame - 1].end_t; //t_next_begin;
                } else {
                    // When not continuous: set the new begin and previous end pose to be consistent
                    trajectory_[index_frame].begin_R = trajectory_[index_frame - 1].end_R;
                    trajectory_[index_frame].begin_t = trajectory_[index_frame - 1].end_t;
                }

                Eigen::Matrix3d R_next_end =
                        trajectory_[index_frame - 1].end_R * trajectory_[index_frame - 2].end_R.inverse() *
                        trajectory_[index_frame - 1].end_R;
                Eigen::Vector3d t_next_end = trajectory_[index_frame - 1].end_t +
                                             trajectory_[index_frame - 1].end_R *
                                             trajectory_[index_frame - 2].end_R.inverse() *
                                             (trajectory_[index_frame - 1].end_t -
                                              trajectory_[index_frame - 2].end_t);

                trajectory_[index_frame].end_R = R_next_end;
                trajectory_[index_frame].end_t = t_next_end;
            } else {
                trajectory_[index_frame] = trajectory_[index_frame - 1];
                // Important ! Start with a rigid frame and let the ICP distort it !
                trajectory_[index_frame] = trajectory_[index_frame - 1];
                trajectory_[index_frame].end_t = trajectory_[index_frame].begin_t;
                trajectory_[index_frame].end_R = trajectory_[index_frame].begin_R;
            }
        }
        return index_frame;
    }

    std::vector<pandar_ros::WPoint3D> Odometry::InitializeFrame(const pcl::PointCloud<pandar_ros::Point>& const_frame, int frame_index) {
        double sample_size = frame_index < options_.init_num_frames ?
            options_.init_voxel_size : options_.voxel_size;
        std::vector<pandar_ros::WPoint3D> frame(const_frame.size());
        for (int i = 0; i < frame.size(); i++) {
            frame[i] = const_frame[i];
            frame[i].frame_index = frame_index;
        }

        std::mt19937_64 g;
        std::shuffle(frame.begin(), frame.end(), g);
        //Subsample the scan with voxels taking one random in every voxel
        sub_sample_frame(frame, sample_size);

        // No elastic ICP for first frame because no initialization of ego-motion
        if (frame_index == 1) {
            for (auto &point3D: frame) {
                point3D.alpha_timestamp = 1.0;
            }
        }

        std::shuffle(frame.begin(), frame.end(), g);

        if (frame_index > 1) {
            if (options_.motion_compensation == CONSTANT_VELOCITY) {
                // The motion compensation of Constant velocity modifies the raw points of the point cloud
                auto &tr_frame = trajectory_[frame_index];
                Eigen::Quaterniond begin_quat(tr_frame.begin_R);
                Eigen::Quaterniond end_quat(tr_frame.end_R);
                DistortFrame(frame, begin_quat, end_quat, tr_frame.begin_t, tr_frame.end_t);
            }

            auto q_begin = Eigen::Quaterniond(trajectory_[frame_index].begin_R);
            auto q_end = Eigen::Quaterniond(trajectory_[frame_index].end_R);
            Eigen::Vector3d t_begin = trajectory_[frame_index].begin_t;
            Eigen::Vector3d t_end = trajectory_[frame_index].end_t;
            for (auto &point3D: frame) {
                TransformPoint(options_.motion_compensation, point3D, q_begin, q_end, t_begin, t_end);
            }
        }

        double min_timestamp = std::numeric_limits<double>::max();
        double max_timestamp = std::numeric_limits<double>::min();
        for (auto &point: frame) {
            point.frame_index = frame_index;
            if (point.raw_point.timestamp > max_timestamp)
                max_timestamp = point.raw_point.timestamp;
            if (point.raw_point.timestamp < min_timestamp)
                min_timestamp = point.raw_point.timestamp;
        }

        trajectory_[frame_index].begin_timestamp = min_timestamp;
        trajectory_[frame_index].end_timestamp = max_timestamp;

        return frame;
    }

    Odometry::RegistrationSummary Odometry::TryRegister(std::vector<pandar_ros::WPoint3D>& frame,
        int frame_index, ct_icp::CTICPOptions& options,
        Odometry::RegistrationSummary& registration_summary,
        double sample_voxel_size) {

        std::vector<pandar_ros::WPoint3D> keypoints;
        grid_sampling(frame, keypoints, sample_voxel_size);

        int num_keypoints = (int)keypoints.size();
        registration_summary.sample_size = num_keypoints;
        
        {
            // CT ICP
            ICPSummary icp_summary;
            if (options_.ct_icp_options.solver == CT_ICP_SOLVER::GN) {
                icp_summary = CT_ICP_GN(options, voxel_map_, keypoints, trajectory_, frame_index);
            } else {
                icp_summary = CT_ICP_CERES(options, voxel_map_, keypoints, trajectory_, frame_index);
            }
            registration_summary.success = icp_summary.success;
            registration_summary.number_of_residuals = icp_summary.num_residuals_used;

            if (!registration_summary.success) {
                registration_summary.success = false;
                return registration_summary;
            }

            //Update frame
            auto q_begin = Eigen::Quaterniond(trajectory_[frame_index].begin_R);
            auto q_end = Eigen::Quaterniond(trajectory_[frame_index].end_R);
            Eigen::Vector3d t_begin = trajectory_[frame_index].begin_t;
            Eigen::Vector3d t_end = trajectory_[frame_index].end_t;
            for (auto &point: frame) {
                // Modifies the world point of the frame based on the raw_pt
                TransformPoint(options_.motion_compensation, point, q_begin, q_end, t_begin, t_end);
            }
        }
        registration_summary.keypoints = keypoints;
        registration_summary.frame = trajectory_[frame_index];
        return registration_summary;
    }

    bool Odometry::AssessRegistration(const std::vector<pandar_ros::WPoint3D>& points, Odometry::RegistrationSummary& summary) const {
        bool success = summary.success;
        if (summary.robust_level == 0 &&
            (summary.relative_orientation > options_.robust_threshold_relative_orientation ||
                summary.ego_orientation > options_.robust_threshold_ego_orientation)) {
            if (summary.robust_level < options_.robust_num_attempts_when_rotation) {
                summary.error_message = "Large rotations require at a robust_level of at least 1 (got:" +
                    std::to_string(summary.robust_level) + ").";
                return false;
            }
        }
        if (summary.relative_distance > options_.robust_relative_trans_threshold) {
            summary.error_message = "The relative distance is too important";
            return false;
        }
        // Only do neighbor assessment if enough motion
        bool do_neighbor_assessment = summary.distance_correction > 0.1;
        do_neighbor_assessment |= summary.relative_distance > options_.robust_neighborhood_min_dist;
        do_neighbor_assessment |= summary.relative_orientation > options_.robust_neighborhood_min_orientation;

        if (do_neighbor_assessment && registered_frames_ > options_.init_num_frames) {
            if (options_.robust_registration) {
                const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
                Voxel voxel;
                double ratio_empty_voxel = 0;
                double ratio_half_full_voxel = 0;

                for (auto &point: points) {
                    voxel = Voxel::Coordinates(point.w_point, kSizeVoxelMap);
                    if (voxel_map_.find(voxel) == voxel_map_.end())
                        ratio_empty_voxel += 1;
                    if (voxel_map_.find(voxel) != voxel_map_.end() &&
                        voxel_map_.at(voxel).NumPoints() > options_.max_num_points_in_voxel / 2) {
                        // Only count voxels which have at least
                        ratio_half_full_voxel += 1;
                    }
                }

                ratio_empty_voxel /= points.size();
                ratio_half_full_voxel /= points.size();

                if (ratio_half_full_voxel < options_.robust_full_voxel_threshold ||
                    ratio_empty_voxel > options_.robust_empty_voxel_threshold) {
                    success = false;
                    if (ratio_empty_voxel > options_.robust_empty_voxel_threshold)
                        summary.error_message = "[Odometry::AssessRegistration] Ratio of empty voxels " +
                                                std::to_string(ratio_empty_voxel) + "above threshold.";
                    else
                        summary.error_message = "[Odometry::AssessRegistration] Ratio of half full voxels " +
                                                std::to_string(ratio_half_full_voxel) + "below threshold.";

                }
            }
        }

        return success;
    }

    Odometry::RegistrationSummary Odometry::DoRegister(const pcl::PointCloud<pandar_ros::Point>& const_frame, int frame_index) {
        auto start_time = std::chrono::steady_clock::now();
        bool is_logging = options_.debug_print;
        CTICPOptions ct_options = options_.ct_icp_options;
        const double kSizeVoxelInitSample = options_.voxel_size;
        const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
        const double kMinDistPoints = options_.min_distance_points;
        const int kMaxNumPointsInVoxel = options_.max_num_points_in_voxel;

        if (is_logging) {
            std::cout << "\n\nRegistering frame " << frame_index << " (frame_id: " << frame_index << ")" << std::endl;
        }
        std::vector<pandar_ros::WPoint3D> frame = InitializeFrame(const_frame, frame_index);
        if (is_logging) {
            std::cout << "Number of points in subsampled frame: " << frame.size() << " out of " << const_frame.size() << std::endl;
        }
        if (frame_index > 0) {
            Eigen::Vector3d t_diff = trajectory_[frame_index].end_t - trajectory_[frame_index].begin_t;
            if (is_logging) {
                std::cout << "Initial ego-motion distance: " << t_diff.norm() << std::endl;
            }
        }

        const auto initial_estimate = trajectory_.back();
        RegistrationSummary summary;
        summary.frame = initial_estimate;
        auto previous_frame = initial_estimate;

        if (frame_index > 0) {
            bool good_enough_registration = false;
            summary.number_of_attempts = 1;
            double sample_voxel_size = frame_index < options_.init_num_frames ?
                                       options_.init_sample_voxel_size : options_.sample_voxel_size;
            double min_voxel_size = std::min(options_.init_voxel_size, options_.voxel_size);

            auto increase_robustness_level = [&]() {
                previous_frame = summary.frame;
                // Handle the failure cases
                trajectory_[frame_index] = initial_estimate;
                ct_options.voxel_neighborhood = std::min(++ct_options.voxel_neighborhood,
                                                             options_.robust_max_voxel_neighborhood);
                ct_options.ls_max_num_iters += 30;
                if (ct_options.max_num_residuals > 0)
                    ct_options.max_num_residuals = ct_options.max_num_residuals * 2;
                ct_options.num_iters_icp = std::min(ct_options.num_iters_icp + 20, 50);
                ct_options.threshold_orientation_norm = std::max(
                        ct_options.threshold_orientation_norm / 10, 1.e-5);
                ct_options.threshold_translation_norm = std::max(
                        ct_options.threshold_orientation_norm / 10, 1.e-4);
                sample_voxel_size = std::max(sample_voxel_size / 1.5, min_voxel_size);
                ct_options.ls_sigma *= 1.2;
                ct_options.max_dist_to_plane_ct_icp *= 1.5;
            };

            summary.robust_level = 0;
            do {
                if (summary.robust_level < next_robust_level_) {
                    // Increase the robustness for the first iteration after a failure
                    summary.robust_level++;
                    increase_robustness_level();
                    continue;
                }
                auto start_ct_icp = std::chrono::steady_clock::now();
    
                TryRegister(frame, frame_index, ct_options, summary, sample_voxel_size);
                auto end_ct_icp = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_ct_icp = end_ct_icp - start_time;
                if (is_logging) {
                    std::cout << "CT-ICP took " << elapsed_ct_icp.count() * 1000 << " ms." << std::endl;
                    std::cout << "No. of keypoints extracted: " << summary.sample_size <<
                        " / Actual no. of residuals: " << summary.number_of_residuals << std::endl;
                }
    

                if (frame_index > 0) {
                    summary.distance_correction = (trajectory_[frame_index].begin_t -
                                                   trajectory_[frame_index - 1].end_t).norm();

                    Eigen::Matrix3d delta_R = (trajectory_[frame_index - 1].end_R *
                                               trajectory_[frame_index].begin_R.inverse());
                    summary.relative_orientation = AngularDistance(trajectory_[frame_index - 1].end_R,
                                                                   trajectory_[frame_index].end_R);
                    summary.ego_orientation = summary.frame.EgoAngularDistance();
                }
    
                summary.relative_distance = (trajectory_[frame_index].end_t - trajectory_[frame_index].begin_t).norm();

                good_enough_registration = AssessRegistration(frame, summary);

                if (options_.robust_fail_early) summary.success = good_enough_registration;

                if (!good_enough_registration) {
                    if (options_.robust_registration && summary.number_of_attempts < options_.robust_num_attempts) {
                        // Either fail or
                        if (is_logging)
                            std::cout << "Registration Attempt n°" << summary.number_of_attempts
                                    << " failed with message: "
                                    << summary.error_message << std::endl;
                        double trans_distance = previous_frame.TranslationDistance(summary.frame);
                        double rot_distance = previous_frame.RotationDistance(summary.frame);
                        if (is_logging)
                            std::cout << "Distance to previous trans : " << trans_distance <<
                                    " rot distance " << rot_distance << std::endl;
                        increase_robustness_level();
                        summary.robust_level++;
                        summary.number_of_attempts++;
                    } else {
                        good_enough_registration = true;
                    }
                }
            } while (!good_enough_registration);

            trajectory_[frame_index].success = summary.success;

            if (!summary.success) {
                if (is_logging)
                    std::cout << "Failure to register, after " << summary.number_of_attempts << std::endl;
                return summary;
            }

            if (summary.number_of_attempts >= options_.robust_num_attempts)
                robust_num_consecutive_failures_++;
            else
                robust_num_consecutive_failures_ = 0;
        }

        if (is_logging) {
            if (frame_index > 0) {
                std::cout << "Trajectory correction [begin(t) - end(t-1)]: " << summary.distance_correction << std::endl;
                std::cout << "Final ego-motion distance: " << summary.relative_distance << std::endl;
            }
        }
        bool add_points = true;

        if (options_.robust_registration) {
            // Communicate whether we suspect an error due to too many attempts
            suspect_registration_error_ = summary.number_of_attempts >= options_.robust_num_attempts;
            if (is_logging) {
                std::cout << "[Robust Registration] "
                        << (suspect_registration_error_ ? "Suspect Registration due to a large number of attempts."
                                                        : "")
                        << "Might be failing. Consecutive failures: " << robust_num_consecutive_failures_ << std::endl;
                std::cout << "[Robust Registration] The rotation ego motion is "
                        << summary.ego_orientation << " (deg)/ " << " relative orientation "
                        << summary.relative_orientation << " (deg) " << std::endl;
            }

            if (summary.ego_orientation > options_.robust_threshold_ego_orientation ||
                summary.relative_orientation > options_.robust_threshold_relative_orientation) {
                if (is_logging)
                    std::cout << "[Robust Registration] Change in orientation too important. "
                               "Points will not be added." << std::endl;
                add_points = false;
            }

            if (suspect_registration_error_) {
                if (robust_num_consecutive_failures_ > 5) {
                    if (is_logging)
                        std::cout << "Adding points despite failure" << std::endl;
                }
                add_points |= (robust_num_consecutive_failures_ > 5);
            }

            next_robust_level_ = add_points ? options_.robust_minimal_level : options_.robust_minimal_level + 1;
            if (!summary.success)
                next_robust_level_ = options_.robust_minimal_level + 2;
            else {
                if (summary.relative_orientation > options_.robust_threshold_relative_orientation ||
                    summary.ego_orientation > options_.robust_threshold_ego_orientation) {
                    next_robust_level_ = options_.robust_minimal_level + 1;
                }
                if (summary.number_of_attempts > 1) {
                    next_robust_level_ = options_.robust_minimal_level + 1;
                }
            }
        }

        const double kMaxDistance = options_.max_distance;
        const Eigen::Vector3d location = trajectory_[frame_index].end_t;
        RemovePointsFarFromLocation(voxel_map_, location, kMaxDistance);

        if (is_logging) {
            std::cout << "Average Load Factor (Map): " << voxel_map_.load_factor() << std::endl;
            std::cout << "Number of Buckets (Map): " << voxel_map_.bucket_count() << std::endl;
            std::cout << "Number of points (Map): " << MapSize() << std::endl;
        }

        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        if (is_logging) {
            std::cout << "Elapsed time: " << elapsed_seconds.count() * 1000 << " ms." << std::endl;
        }

        summary.corrected_points = frame;
        // summary.all_corrected_points.resize(const_frame.size());
        // auto raw_points = const_frame.points;

        // for (int i = 0; i < summary.all_corrected_points.size(); i++) {
        //     summary.all_corrected_points[i] = raw_points[i];
        //     summary.all_corrected_points[i].frame_index = frame_info.frame_id;
        // }

        // const slam::Pose& beg_pose = summary.frame.begin_pose;
        // const slam::Pose& end_pose = summary.frame.end_pose;

        // for (pandar_ros::WPoint3D& point : summary.corrected_points) {
        //     point.w_point = beg_pose.ContinuousTransform(Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z), end_pose, point.raw_point.timestamp);
        // }

        // for (pandar_ros::WPoint3D& point : summary.all_corrected_points) {
        //     point.w_point = beg_pose.ContinuousTransform(Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z), end_pose, point.raw_point.timestamp);
        // }

        if (add_points) {
            //Update Voxel Map+
            AddPointsToMap(voxel_map_, frame, kSizeVoxelMap,
                           kMaxNumPointsInVoxel, kMinDistPoints);
        }
        return summary;
    }

    std::vector<TrajectoryFrame> Odometry::Trajectory() const {
        return trajectory_;
    }

    ArrayVector3d Odometry::GetLocalMap() const {
        return MapAsPointcloud(voxel_map_);
    }

    Odometry::Odometry(const OdometryOptions& options) {
        options_ = options;
        // Update the motion compensation
        switch (options_.motion_compensation) {
        case MOTION_COMPENSATION::NONE:
        case MOTION_COMPENSATION::CONSTANT_VELOCITY:
            // ElasticICP does not compensate the motion
            options_.ct_icp_options.point_to_plane_with_distortion = false;
            options_.ct_icp_options.distance = POINT_TO_PLANE;
            break;
        case MOTION_COMPENSATION::ITERATIVE:
            // ElasticICP compensates the motion at each ICP iteration
            options_.ct_icp_options.point_to_plane_with_distortion = true;
            options_.ct_icp_options.distance = POINT_TO_PLANE;
            break;
        case MOTION_COMPENSATION::CONTINUOUS:
            // ElasticICP compensates continuously the motion
            options_.ct_icp_options.point_to_plane_with_distortion = true;
            options_.ct_icp_options.distance = CT_POINT_TO_PLANE;
            break;
        }
        next_robust_level_ = options.robust_minimal_level;

        if (options_.log_to_file) {
            log_file_ = std::make_unique<std::ofstream>(options_.log_file_destination.c_str(),
                std::ofstream::trunc);
            log_out_ = log_file_.get();
            *log_out_ << "Debug Print ?" << options_.debug_print << std::endl;
        }
        else
            log_out_ = &std::cout;
    }

    ArrayVector3d MapAsPointcloud(const VoxelHashMap &map) {
        ArrayVector3d points;
        points.reserve(MapSize(map));
        for (auto &voxel: map) {
            for (int i(0); i < voxel.second.NumPoints(); ++i)
                points.push_back(voxel.second.points[i]);
        }
        return points;
    }

    size_t MapSize(const VoxelHashMap &map) {
        size_t map_size(0);
        for (auto &itr_voxel_map: map) {
            map_size += (itr_voxel_map.second).NumPoints();
        }
        return map_size;
    }

    void RemovePointsFarFromLocation(VoxelHashMap& map, const Eigen::Vector3d& location, double distance) {
        std::vector<Voxel> voxels_to_erase;
        for (auto& pair : map) {
            Eigen::Vector3d pt = pair.second.points[0];
            if ((pt - location).squaredNorm() > (distance * distance)) {
                voxels_to_erase.push_back(pair.first);
            }
        }
        for (auto& vox : voxels_to_erase)
            map.erase(vox);
    }

    void AddPointToMap(VoxelHashMap& map, const Eigen::Vector3d& point, double voxel_size,
        int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0) {
        short kx = static_cast<short>(point[0] / voxel_size);
        short ky = static_cast<short>(point[1] / voxel_size);
        short kz = static_cast<short>(point[2] / voxel_size);

        VoxelHashMap::iterator search = map.find(Voxel(kx, ky, kz));
        if (search != map.end()) {
            auto& voxel_block = (search.value());

            if (!voxel_block.IsFull()) {
                double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                for (int i(0); i < voxel_block.NumPoints(); ++i) {
                    auto& _point = voxel_block.points[i];
                    double sq_dist = (_point - point).squaredNorm();
                    if (sq_dist < sq_dist_min_to_points) {
                        sq_dist_min_to_points = sq_dist;
                    }
                }
                if (sq_dist_min_to_points > (min_distance_points * min_distance_points)) {
                    if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
                        voxel_block.AddPoint(point);
                    }
                }
            }
        }
        else {
            if (min_num_points <= 0) {
                // Do not add points (avoids polluting the map)
                VoxelBlock block(max_num_points_in_voxel);
                block.AddPoint(point);
                map[Voxel(kx, ky, kz)] = std::move(block);
            }
        }
    }

    void AddPointsToMap(VoxelHashMap& map, const std::vector<pandar_ros::WPoint3D>& points,
        double voxel_size, int max_num_points_in_voxel,
        double min_distance_points, int min_num_points) {
        for (const auto& point : points) {
            AddPointToMap(map, point.w_point, voxel_size, max_num_points_in_voxel, min_distance_points,
                min_num_points);
        }
    }

}
#endif