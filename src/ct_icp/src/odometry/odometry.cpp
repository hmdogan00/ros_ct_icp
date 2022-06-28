#ifndef CT_ICP_ODOMETRY_HPP
#define CT_ICP_ODOMETRY_HPP
#include "odometry.h"

namespace ct_icp {
    size_t Odometry::MapSize() const {
        return ::ct_icp::MapSize(voxel_map_);
    }

    void DistortFrame(std::vector<pandar_ros::WPoint3D>& points, const slam::Pose& begin, const slam::Pose& end) {
        slam::SE3 end_inverse = end.Inverse().pose;
        for (pandar_ros::WPoint3D& point : points) {
            auto interpolated = begin.InterpolatePose(end, point.raw_point.timestamp).pose;
            Eigen::Vector3d v = end_inverse * (interpolated * Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z));
            point.raw_point.x = v(0);
            point.raw_point.y = v(1);
            point.raw_point.z = v(2);
        }
    }

    inline void TransformPoint(MOTION_COMPENSATION comp, pandar_ros::WPoint3D p, const slam::Pose& begin, const slam::Pose& end) {
        slam::SE3 pose = end.pose;
        switch (comp) {
        case MOTION_COMPENSATION::NONE:
        case MOTION_COMPENSATION::CONSTANT_VELOCITY:
            break;
        case MOTION_COMPENSATION::CONTINUOUS:
        case MOTION_COMPENSATION::ITERATIVE:
            pose = begin.InterpolatePose(end, p.raw_point.timestamp).pose;
            break;
        }
        p.w_point = pose * Eigen::Vector3d(p.raw_point.x, p.raw_point.y, p.raw_point.z);
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

    size_t MapSize(const VoxelHashMap& map) {
        size_t map_size(0);
        for (auto& itr_voxel_map : map) {
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

    const auto compute_frame_info = [](const std::vector<double> timestamps, auto registered_fid) {
        Odometry::FrameInfo frame_info;
        auto begin = timestamps.cbegin();
        frame_info.registered_fid = registered_fid;
        frame_info.frame_id = registered_fid;
        auto min_max_pair = std::minmax_element(timestamps.cbegin(), timestamps.cend());
        frame_info.begin_timestamp = *(min_max_pair.first);
        frame_info.end_timestamp = *(min_max_pair.second);
        return frame_info;
    };

    Odometry::RegistrationSummary Odometry::RegisterFrame(const pcl::PointCloud<pandar_ros::Point>& frame, const std::vector<double>& timestamp_vector) {
        auto frame_info = compute_frame_info(timestamp_vector, registered_frames_++);
        InitializeMotion(frame_info, nullptr);
        return DoRegister(frame, frame_info);
    }

    void Odometry::InitializeMotion(FrameInfo frame_info, const TrajectoryFrame* initial_estimate) {
        if (initial_estimate != nullptr) {
            // Insert previous estimate
            trajectory_.emplace_back(*initial_estimate);
            return;
        }

        const auto kFrameIndex = frame_info.registered_fid;
        // Initial Trajectory Estimate
        trajectory_.emplace_back(TrajectoryFrame());
        trajectory_[kFrameIndex].begin_pose = slam::Pose(slam::SE3(), frame_info.begin_timestamp, frame_info.frame_id);
        trajectory_[kFrameIndex].end_pose = slam::Pose(slam::SE3(), frame_info.end_timestamp, frame_info.frame_id);

        if (kFrameIndex <= 1) {
            // Initialize first pose at Identity
        }
        else if (kFrameIndex == 2) {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                // Different regimen for the second frame due to the bootstrapped elasticity
                trajectory_[kFrameIndex].begin_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose *
                    trajectory_[kFrameIndex - 2].end_pose.pose.Inverse() *
                    trajectory_[kFrameIndex - 1].end_pose.pose;
            }
            else {
                // Important ! Start with a rigid frame and let the ICP distort it !
                // It would make more sense to start
                trajectory_[kFrameIndex].begin_pose.pose = trajectory_[kFrameIndex - 1].begin_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex].begin_pose.pose;
            }
        }
        else {
            const auto& frame_m_1 = trajectory_[kFrameIndex - 1];
            const auto& frame_m_2 = trajectory_[kFrameIndex - 2];

            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                if (options_.motion_compensation == CONTINUOUS) {
                    // When continuous: use the previous begin_pose as reference
                    auto next_begin = frame_m_1.begin_pose.pose *
                        frame_m_2.begin_pose.pose.Inverse() *
                        frame_m_1.begin_pose.pose;
                    trajectory_[kFrameIndex].begin_pose.pose = next_begin;
                }
                else {
                    // When not continuous: set the new begin and previous end pose to be consistent
                    trajectory_[kFrameIndex].begin_pose.pose = frame_m_1.end_pose.pose;
                }
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose *
                    trajectory_[kFrameIndex - 2].end_pose.pose.Inverse() *
                    trajectory_[kFrameIndex - 1].end_pose.pose;
            }
            else {
                trajectory_[kFrameIndex].begin_pose.pose = frame_m_1.begin_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = frame_m_1.begin_pose.pose;
            }
        }
    }

    std::vector<pandar_ros::WPoint3D> Odometry::InitializeFrame(const pcl::PointCloud<pandar_ros::Point>& const_frame, Odometry::FrameInfo frame_info) {
        double sample_size = frame_info.registered_fid < options_.init_num_frames ?
            options_.init_voxel_size : options_.voxel_size;
        std::vector<pandar_ros::WPoint3D> frame(const_frame.size());
        for (int i = 0; i < frame.size(); i++) {
            frame[i] = const_frame[i];
            frame[i].frame_index = frame_info.frame_id;
        }
        const int k_index_frame = frame_info.registered_fid;

        std::mt19937_64 g;
        std::shuffle(frame.begin(), frame.end(), g);
        // subsample the scan with voxels taking one random in every voxel
        ct_icp::sub_sample_frame(frame, sample_size);

        // no icp first time (because no ego motion)
        if (k_index_frame <= 1) {
            for (pandar_ros::WPoint3D& point : frame) {
                point.raw_point.timestamp = frame_info.end_timestamp;
            }
        }
        std::shuffle(frame.begin(), frame.end(), g);

        const ct_icp::TrajectoryFrame& tr_frame = trajectory_[k_index_frame];
        if (k_index_frame > 1) {
            if (options_.motion_compensation == ct_icp::CONSTANT_VELOCITY) {
                DistortFrame(frame, tr_frame.begin_pose, tr_frame.end_pose);
            }
        }

        for (pandar_ros::WPoint3D& point : frame) {
            TransformPoint(options_.motion_compensation, point, tr_frame.begin_pose, tr_frame.end_pose);
        }

        for (pandar_ros::WPoint3D& point : frame) {
            point.frame_index = frame_info.frame_id;
        }
        return frame;
    }

    Odometry::RegistrationSummary Odometry::TryRegister(std::vector<pandar_ros::WPoint3D>& frame,
        Odometry::FrameInfo frame_info, ct_icp::CTICPOptions& options,
        Odometry::RegistrationSummary& registration_summary,
        double sample_voxel_size) {

        std::vector<pandar_ros::WPoint3D> keypoints;
        grid_sampling(frame, keypoints, sample_voxel_size);

        const int k_index_frame = frame_info.registered_fid;
        int num_keypoints = (int)keypoints.size();
        registration_summary.sample_size = num_keypoints;

        const TrajectoryFrame* prev_frame = k_index_frame <= 1 ? nullptr : &trajectory_[k_index_frame - 1];
        {
            if (k_index_frame < options_.init_num_frames) {
                options.voxel_neighborhood = std::max(2, options.voxel_neighborhood);
                options.threshold_voxel_occupancy = 1;
                options.num_iters_icp = std::max(options.num_iters_icp, 15);
            }

            // IterateOverCallbacks(OdometryCallback::BEFORE_ITERATION,
            //                      frame, &keypoints);

            // CT ICP
            ICPSummary icp_summary;
            CT_ICP_Registration registration;
            registration.Options() = options;
            icp_summary = registration.Register(voxel_map_, keypoints, registration_summary.frame, prev_frame);

            registration_summary.success - icp_summary.success;
            registration_summary.number_of_residuals = icp_summary.num_residuals_used;

            if (!registration_summary.success) {
                registration_summary.success = false;
                return registration_summary;
            }

            // update frame
            slam::Pose pose_begin = registration_summary.frame.begin_pose;
            slam::Pose pose_end = registration_summary.frame.end_pose;
            for (pandar_ros::WPoint3D& point : frame) {
                TransformPoint(options_.motion_compensation, point, pose_begin, pose_end);
            }
            registration_summary.keypoints = keypoints;
        }
        // IterateOverCallbacks(OdometryCallback::ITERATION_COMPLETED,
        //                      frame, &keypoints, nullptr);

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

    Odometry::RegistrationSummary Odometry::DoRegister(const pcl::PointCloud<pandar_ros::Point>& const_frame, FrameInfo frame_info) {
        auto start_time = std::chrono::steady_clock::now();
        bool is_logging = options_.debug_print;
        CTICPOptions ct_options = options_.ct_icp_options;
        const double kSizeVoxelInitSample = options_.voxel_size;
        const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
        const double kMinDistPoints = options_.min_distance_points;
        const int kMaxNumPointsInVoxel = options_.max_num_points_in_voxel;

        const int k_index_frame = frame_info.registered_fid;
        if (is_logging) {
            std::cout << "\n\nRegistering frame " << k_index_frame << " (frame_id: " << frame_info.frame_id << ")" << std::endl;
        }
        std::vector<pandar_ros::WPoint3D> frame = InitializeFrame(const_frame, frame_info);
        if (is_logging) {
            std::cout << "Number of points in subsampled frame: " << frame.size() << " out of " << const_frame.size() << std::endl;
        }
        if (k_index_frame > 0) {
            Eigen::Vector3d t_diff = trajectory_[k_index_frame].end_pose.pose.tr - trajectory_[k_index_frame].begin_pose.pose.tr;
            if (is_logging) {
                std::cout << "Initial ego-motion distance: " << t_diff.norm() << std::endl;
            }
        }

        const ct_icp::TrajectoryFrame initial_estimate = trajectory_.back();
        RegistrationSummary summary;
        summary.frame = initial_estimate;
        ct_icp::TrajectoryFrame& current_frame = summary.frame;
        ct_icp::TrajectoryFrame previous_frame = initial_estimate;

        if (k_index_frame > 0) {
            bool good_enough = false;
            summary.number_of_attempts = 1;
            double sample_voxel_size = k_index_frame < options_.init_num_frames ? options_.init_sample_voxel_size : options_.sample_voxel_size;
            double min_voxel_size = std::min(options_.init_voxel_size, options_.voxel_size);

            auto increase_robustness = [&]() {
                // handle failure cases
                previous_frame = current_frame;
                current_frame = initial_estimate;
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
                    summary.robust_level++;
                    increase_robustness();
                    continue;
                }
                auto start_ct_icp = std::chrono::steady_clock::now();
                TryRegister(frame, frame_info, ct_options, summary, sample_voxel_size); // TODO: implement TryRegister
                auto end_ct_icp = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_ct_icp = end_ct_icp - start_time;
                if (is_logging) {
                    std::cout << "CT-ICP took " << elapsed_ct_icp.count() * 1000 << " ms." << std::endl;
                    std::cout << "No. of keypoints extracted: " << summary.sample_size <<
                        " / Actual no. of residuals: " << summary.number_of_residuals << std::endl;
                }

                if (k_index_frame > 0) {
                    summary.distance_correction = (current_frame.begin_pose.pose.tr - trajectory_[k_index_frame - 1].end_pose.pose.tr).norm();
                    auto norm = ((trajectory_[k_index_frame - 1].EndQuat().normalized().toRotationMatrix() *
                        current_frame.EndQuat().normalized().toRotationMatrix().transpose()).trace() - 1.) / 2.;
                    if (std::abs(norm) > 1. + 1.e-8) {
                        std::cout << "Not a rotation matrix " << norm << std::endl;
                    }

                    summary.relative_orientation = AngularDistance(trajectory_[k_index_frame - 1].end_pose.pose, current_frame.end_pose.pose);
                    summary.ego_orientation = summary.frame.EgoAngularDistance();
                }
                summary.relative_distance = (current_frame.EndTr() - current_frame.BeginTr()).norm();
                good_enough = AssessRegistration(frame, summary); // TODO: implement AssessRegistration

                if (options_.robust_fail_early) summary.success = good_enough;

                if (!good_enough) {
                    if (options_.robust_registration && summary.number_of_attempts < options_.robust_num_attempts) {
                        if (is_logging)
                            std::cout << "Registration Attempt no" << summary.number_of_attempts
                            << " failed with message: "
                            << summary.error_message << std::endl;
                        double trans_distance = previous_frame.TranslationDistance(summary.frame);
                        double rot_distance = previous_frame.RotationDistance(summary.frame);
                        if (is_logging)
                            std::cout << "Distance to previous trans : " << trans_distance <<
                            " rot distance " << rot_distance << std::endl;
                        increase_robustness();
                        summary.robust_level++;
                        summary.number_of_attempts++;
                    }
                    else {
                        good_enough = true;
                    }
                }
            } while (!good_enough);

            if (!summary.success) {
                if (is_logging)
                    std::cout << "Failure to register, after " << summary.number_of_attempts << std::endl;
                return summary;
            }

            if (summary.number_of_attempts >= options_.robust_num_attempts)
                robust_num_consecutive_failures_++;
            else
                robust_num_consecutive_failures_ = 0;

            trajectory_[k_index_frame] = summary.frame;
        }

        if (is_logging) {
            if (k_index_frame > 0) {
                std::cout << "Trajectory correction [begin(t) - end(t-1)]: " << summary.distance_correction << std::endl;
                std::cout << "Final ego-motion distance: " << summary.relative_distance << std::endl;
            }
        }
        bool add_points = true;

        if (options_.robust_registration) {
            std::cout << "Robust registration is not implemented!" << std::endl;
        }

        const double k_max_distance = options_.max_distance;
        const Eigen::Vector3d location = trajectory_[k_index_frame].EndTr();
        RemovePointsFarFromLocation(voxel_map_, location, k_max_distance);

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
        summary.all_corrected_points.resize(const_frame.size());
        auto raw_points = const_frame.points;

        for (int i = 0; i < summary.all_corrected_points.size(); i++) {
            summary.all_corrected_points[i] = raw_points[i];
            summary.all_corrected_points[i].frame_index = frame_info.frame_id;
        }

        const slam::Pose& beg_pose = summary.frame.begin_pose;
        const slam::Pose& end_pose = summary.frame.end_pose;

        for (pandar_ros::WPoint3D& point : summary.corrected_points) {
            point.w_point = beg_pose.ContinuousTransform(Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z), end_pose, point.raw_point.timestamp);
        }

        for (pandar_ros::WPoint3D& point : summary.all_corrected_points) {
            point.w_point = beg_pose.ContinuousTransform(Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z), end_pose, point.raw_point.timestamp);
        }

        if (add_points) {
            AddPointsToMap(voxel_map_, summary.corrected_points, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistPoints);
        }
        return summary;
    }
}
#endif