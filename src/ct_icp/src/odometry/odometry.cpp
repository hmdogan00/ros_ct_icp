#include "odometry.h"

namespace ct_icp {
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

    Odometry::RegistrationSummary Odometry::RegisterFrame(const pcl::PointCloud<pandar_ros::Point>& frame, const std::vector<double>& timestamps) {
        auto frame_info = compute_frame_info(timestamps, registered_frames_++);
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
}