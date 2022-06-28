
#include <mutex>
#include <thread>

#include "odometry/odometry.h"

int init, motion, dist, ls, solver, weight;

int frame_id = 0;
std::mutex registration_mutex;
std::unique_ptr<ct_icp::Odometry> odometry_ptr = nullptr;
const std::string main_frame_id = "/Odometry";
const std::string child_frame_id = "/child";

struct Options {
    ct_icp::OdometryOptions odometry_options;
    std::string lidar_topic = "/center/pandar";
    int max_num_threads = 1; // The maximum number of threads running in parallel the Dataset acquisition
    bool suspend_on_failure = false; // Whether to suspend the execution once an error is detected
    bool save_trajectory = true; // whether to save the trajectory
    std::string output_dir = "./outputs"; // The output path (relative or absolute) to save the pointclouds
    std::string sequence; // The desired sequence (only applicable if `all_sequences` is false)
    int start_index = 0; // The start index of the sequence (only applicable if `all_sequences` is false)
    int max_frames = -1; // The maximum number of frames to register (if -1 all frames in the Dataset are registered)
};

Options get_options(ros::NodeHandle nh) {
    Options options;
    nh.param<int>("/max_num_threads", options.max_num_threads, 1);
    nh.param<std::string>("/output_dir", options.output_dir, ".outputs");
    nh.param<std::string>("/lidar_topic", options.lidar_topic, "/center/pandar");
    nh.param<bool>("/save_trajectory", options.save_trajectory, true);
    nh.param<bool>("/suspend_on_failure", options.suspend_on_failure, false);

    nh.param<bool>("/odometry_options/debug_print", options.odometry_options.debug_print, true);
    nh.param<double>("/odometry_options/distance_error_threshold", options.odometry_options.distance_error_threshold, 5.0);
    nh.param<int>("/odometry_options/init_num_frames", options.odometry_options.init_num_frames, 20);
    nh.param<int>("/odometry_options/initialization", init, 1);
    options.odometry_options.initialization = static_cast<ct_icp::INITIALIZATION>(init);
    nh.param<bool>("/odometry_options/log_to_file", options.odometry_options.log_to_file, false);
    nh.param<double>("/odometry_options/max_distance", options.odometry_options.max_distance, 100.0);
    nh.param<int>("/odometry_options/max_num_points_in_voxel", options.odometry_options.max_num_points_in_voxel, 20);
    nh.param<double>("/odometry_options/min_distance_points", options.odometry_options.min_distance_points, 0.1);
    nh.param<int>("/odometry_options/motion_compensation", motion, 3);
    options.odometry_options.motion_compensation = static_cast<ct_icp::MOTION_COMPENSATION>(motion);
    nh.param<double>("/odometry_options/robust_full_voxel_threshold", options.odometry_options.robust_full_voxel_threshold, 0.7);
    nh.param<int>("/odometry_options/robust_max_voxel_neighborhood", options.odometry_options.robust_max_voxel_neighborhood, 4);
    nh.param<int>("/odometry_options/robust_minimal_level", options.odometry_options.robust_minimal_level, 0);
    nh.param<int>("/odometry_options/robust_num_attempts", options.odometry_options.robust_num_attempts, 6);
    nh.param<bool>("/odometry_options/robust_registration", options.odometry_options.robust_registration, false);
    nh.param<double>("/odometry_options/robust_threshold_ego_orientation", options.odometry_options.robust_threshold_ego_orientation, 2.0);
    nh.param<double>("/odometry_options/robust_threshold_relative_orientation", options.odometry_options.robust_threshold_relative_orientation, 2.0);
    nh.param<double>("/odometry_options/sample_voxel_size", options.odometry_options.sample_voxel_size, 1.5);
    nh.param<double>("/odometry_options/voxel_size", options.odometry_options.voxel_size, 0.5);

    nh.param<double>("/odometry_options/ct_icp_options/beta_constant_velocity", options.odometry_options.ct_icp_options.beta_constant_velocity, 0.001);
    nh.param<double>("/odometry_options/ct_icp_options/beta_location_consistency", options.odometry_options.ct_icp_options.beta_location_consistency, 0.001);
    nh.param<double>("/odometry_options/ct_icp_options/beta_orientation_consistency", options.odometry_options.ct_icp_options.beta_orientation_consistency, 0.0);
    nh.param<double>("/odometry_options/ct_icp_options/beta_small_velocity", options.odometry_options.ct_icp_options.beta_small_velocity, 0.0);
    nh.param<bool>("/odometry_options/ct_icp_options/debug_print", options.odometry_options.ct_icp_options.debug_print, false);
    nh.param<int>("/odometry_options/ct_icp_options/distance", dist, 1);
    options.odometry_options.ct_icp_options.distance = static_cast<ct_icp::ICP_DISTANCE>(dist);
    nh.param<int>("/odometry_options/ct_icp_options/loss_function", ls, 1);
    options.odometry_options.ct_icp_options.loss_function = static_cast<ct_icp::LEAST_SQUARES>(ls);
    nh.param<int>("/odometry_options/ct_icp_options/ls_max_num_iters", options.odometry_options.ct_icp_options.ls_max_num_iters, 10);
    nh.param<int>("/odometry_options/ct_icp_options/ls_num_threads", options.odometry_options.ct_icp_options.ls_num_threads, 8);
    nh.param<double>("/odometry_options/ct_icp_options/ls_sigma", options.odometry_options.ct_icp_options.ls_sigma, 0.1);
    nh.param<double>("/odometry_options/ct_icp_options/ls_tolerant_min_threshold", options.odometry_options.ct_icp_options.ls_tolerant_min_threshold, 0.05);
    nh.param<double>("/odometry_options/ct_icp_options/max_dist_to_plane_ct_icp", options.odometry_options.ct_icp_options.max_dist_to_plane_ct_icp, 0.05);
    nh.param<int>("/odometry_options/ct_icp_options/max_num_residuals", options.odometry_options.ct_icp_options.max_num_residuals, 1000);
    nh.param<int>("/odometry_options/ct_icp_options/max_number_neighbors", options.odometry_options.ct_icp_options.max_number_neighbors, 20);
    nh.param<int>("/odometry_options/ct_icp_options/min_num_residuals", options.odometry_options.ct_icp_options.min_num_residuals, 200);
    nh.param<int>("/odometry_options/ct_icp_options/min_number_neighbors", options.odometry_options.ct_icp_options.min_number_neighbors, 20);
    nh.param<int>("/odometry_options/ct_icp_options/num_closest_neighbors", options.odometry_options.ct_icp_options.num_closest_neighbors, 1);
    nh.param<int>("/odometry_options/ct_icp_options/num_iters_icp", options.odometry_options.ct_icp_options.num_iters_icp, 30);
    nh.param<bool>("/odometry_options/ct_icp_options/point_to_plane_with_distortion", options.odometry_options.ct_icp_options.point_to_plane_with_distortion, true);
    nh.param<double>("/odometry_options/ct_icp_options/power_planarity", options.odometry_options.ct_icp_options.power_planarity, 2.0);
    nh.param<double>("/odometry_options/ct_icp_options/size_voxel_map", options.odometry_options.ct_icp_options.size_voxel_map, 1.0);
    nh.param<int>("/odometry_options/ct_icp_options/solver", solver, 1);
    options.odometry_options.ct_icp_options.solver = static_cast<ct_icp::CT_ICP_SOLVER>(solver);
    nh.param<double>("/odometry_options/ct_icp_options/threshold_orientation_norm", options.odometry_options.ct_icp_options.threshold_orientation_norm, 0.05);
    nh.param<double>("/odometry_options/ct_icp_options/threshold_translation_norm", options.odometry_options.ct_icp_options.threshold_translation_norm, 0.005);
    nh.param<int>("/odometry_options/ct_icp_options/threshold_voxel_occupancy", options.odometry_options.ct_icp_options.threshold_voxel_occupancy, 5);
    nh.param<int>("/odometry_options/ct_icp_options/voxel_neighborhood", options.odometry_options.ct_icp_options.voxel_neighborhood, 1);
    nh.param<double>("/odometry_options/ct_icp_options/weight_alpha", options.odometry_options.ct_icp_options.weight_alpha, 0.9);
    nh.param<double>("/odometry_options/ct_icp_options/weight_neighborhood", options.odometry_options.ct_icp_options.weight_neighborhood, 0.1);
    nh.param<int>("/odometry_options/ct_icp_options/weighting_scheme", weight, 2);
    options.odometry_options.ct_icp_options.weighting_scheme = static_cast<ct_icp::WEIGHTING_SCHEME>(weight);

    return options;
}

void pcl_cb(const sensor_msgs::PointCloud2::ConstPtr& input) {
    std::lock_guard<std::mutex> guard{ registration_mutex };

    pcl::PointCloud<pandar_ros::Point> pcl_pc2;
    pcl::fromROSMsg(*input, pcl_pc2);

    // add timestamp fields to double vector
    auto stamp = pcl_pc2.header.stamp;
    std::vector<double> timestamp_vec;
    for (int i = 0; i < pcl_pc2.size(); i++) {
        double timestamp = std::fmod(pcl_pc2.points[i].timestamp, 100);
        timestamp_vec.push_back(timestamp);
    }

    ct_icp::Odometry::RegistrationSummary summary = odometry_ptr->RegisterFrame(pcl_pc2, timestamp_vec);

    // slam::SE3 p = odometry_ptr->RegisterFrame(pcl_pc2, timestamp_vec)[frame_id].begin_pose.pose;
    // std::cout << "qw qx qy qz tx ty tz\n" << std::endl;
    // std::cout << p.quat.w() << " " << p.quat.x() << " " << p.quat.y() << " " << p.quat.z() << " " << p.tr.x() << " " << p.tr.y() << " " << p.tr.z() << std::endl;
    frame_id++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ct_icp");
    ros::NodeHandle nh;

    // Parameter registration
    Options options = get_options(nh);

    odometry_ptr = std::make_unique<ct_icp::Odometry>(options.odometry_options);

    ros::Subscriber sub_pcl = nh.subscribe(options.lidar_topic, 200000, pcl_cb);
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}