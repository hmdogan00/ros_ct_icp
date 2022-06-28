#include <omp.h>
#include <chrono>
#include <queue>
#include <thread>

#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "ct_icp.h"
#include "../cost_functions/cost_func.h"

namespace ct_icp {
    
    void sub_sample_frame(std::vector<pandar_ros::WPoint3D> &frame, double size_voxel) {
        std::unordered_map<Voxel, std::vector<pandar_ros::WPoint3D>> grid;
        for (int i = 0; i < (int) frame.size(); i++) {
            auto kx = static_cast<short>(frame[i].raw_point.x / size_voxel);
            auto ky = static_cast<short>(frame[i].raw_point.y / size_voxel);
            auto kz = static_cast<short>(frame[i].raw_point.z / size_voxel);
            grid[Voxel(kx, ky, kz)].push_back(frame[i]);
        }
        frame.resize(0);
        int step = 0; //to take one random point inside each voxel (but with identical results when lunching the SLAM a second time)
        for (const auto &n: grid) {
            if (n.second.size() > 0) {
                //frame.push_back(n.second[step % (int)n.second.size()]);
                frame.push_back(n.second[0]);
                step++;
            }
        }
    }
    
    void grid_sampling(const std::vector<pandar_ros::WPoint3D> &frame,
                    std::vector<pandar_ros::WPoint3D> &keypoints,
                    double size_voxel_subsampling) {
        // TODO Replace std::list by a vector ?
        keypoints.clear();
        std::vector<pandar_ros::WPoint3D> frame_sub;
        frame_sub.resize(frame.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            frame_sub[i] = frame[i];
        }
        sub_sample_frame(frame_sub, size_voxel_subsampling);
        keypoints.reserve(frame_sub.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            keypoints.push_back(frame_sub[i]);
        }
    }
    
    struct Neighborhood {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d center = Eigen::Vector3d::Zero();

        Eigen::Vector3d normal = Eigen::Vector3d::Zero();

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

        double a2D = 1.0; // Planarity coefficient
    };

    // Computes normal and planarity coefficient
    Neighborhood compute_neighborhood_distribution(const ArrayVector3d &points) {
        Neighborhood neighborhood;
        // Compute the normals
        Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
        for (auto &point: points) {
            barycenter += point;
        }
        barycenter /= (double) points.size();
        neighborhood.center = barycenter;

        Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
        for (auto &point: points) {
            for (int k = 0; k < 3; ++k)
                for (int l = k; l < 3; ++l)
                    covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                               (point(l) - barycenter(l));
        }
        covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
        covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
        covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
        neighborhood.covariance = covariance_Matrix;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
        Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
        neighborhood.normal = normal;

        // Compute planarity from the eigen values
        double sigma_1 = sqrt(std::abs(
                es.eigenvalues()[2])); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
        double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
        double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
        neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

        if (neighborhood.a2D != neighborhood.a2D) {
            LOG(ERROR) << "FOUND NAN!!!";
            throw std::runtime_error("error");
        }

        return neighborhood;
    }

    using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

    struct Comparator {
        bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
            return std::get<0>(left) < std::get<0>(right);
        }
    };
    using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, Comparator>;

    inline ArrayVector3d
    search_neighbors(const VoxelHashMap &map,
                     const Eigen::Vector3d &point,
                     int nb_voxels_visited,
                     double size_voxel_map,
                     int max_num_neighbors,
                     int threshold_voxel_capacity = 1,
                     std::vector<Voxel> *voxels = nullptr) {

        if (voxels != nullptr)
            voxels->reserve(max_num_neighbors);

        short kx = static_cast<short>(point[0] / size_voxel_map);
        short ky = static_cast<short>(point[1] / size_voxel_map);
        short kz = static_cast<short>(point[2] / size_voxel_map);

        priority_queue_t priority_queue;

        Voxel voxel(kx, ky, kz);
        for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
            for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
                for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                    voxel.x = kxx;
                    voxel.y = kyy;
                    voxel.z = kzz;

                    auto search = map.find(voxel);
                    if (search != map.end()) {
                        const auto &voxel_block = search.value();
                        if (voxel_block.NumPoints() < threshold_voxel_capacity)
                            continue;
                        for (int i(0); i < voxel_block.NumPoints(); ++i) {
                            auto &neighbor = voxel_block.points[i];
                            double distance = (neighbor - point).norm();
                            if (priority_queue.size() == max_num_neighbors) {
                                if (distance < std::get<0>(priority_queue.top())) {
                                    priority_queue.pop();
                                    priority_queue.emplace(distance, neighbor, voxel);
                                }
                            } else
                                priority_queue.emplace(distance, neighbor, voxel);
                        }
                    }
                }
            }
        }

        auto size = priority_queue.size();
        ArrayVector3d closest_neighbors(size);
        if (voxels != nullptr) {
            voxels->resize(size);
        }
        for (auto i = 0; i < size; ++i) {
            closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
            if (voxels != nullptr)
                (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
            priority_queue.pop();
        }


        return closest_neighbors;
    }

    inline ArrayVector3d
    select_closest_neighbors(const std::vector<std::vector<Eigen::Vector3d> const *> &neighbors_ptr,
                             const Eigen::Vector3d &pt_keypoint,
                             int num_neighbors, int max_num_neighbors) {
        std::vector<std::pair<double, Eigen::Vector3d>> distance_neighbors;
        distance_neighbors.reserve(neighbors_ptr.size());
        for (auto &it_ptr: neighbors_ptr) {
            for (auto &it: *it_ptr) {
                double sq_dist = (pt_keypoint - it).squaredNorm();
                distance_neighbors.emplace_back(sq_dist, it);
            }
        }


        int real_number_neighbors = std::min(max_num_neighbors, (int) distance_neighbors.size());
        std::partial_sort(distance_neighbors.begin(),
                          distance_neighbors.begin() + real_number_neighbors,
                          distance_neighbors.end(),
                          [](const std::pair<double, Eigen::Vector3d> &left,
                             const std::pair<double, Eigen::Vector3d> &right) {
                              return left.first < right.first;
                          });

        ArrayVector3d neighbors(real_number_neighbors);
        for (auto i(0); i < real_number_neighbors; ++i)
            neighbors[i] = distance_neighbors[i].second;
        return neighbors;
    }

        // A Builder to abstract the different configurations of ICP optimization
    class ICPOptimizationBuilder {
    public:
        using CTICP_PointToPlaneResidual = ceres::AutoDiffCostFunction<CTPointToPlaneFunctor, 1, 4, 3, 4, 3>;
        using PointToPlaneResidual = ceres::AutoDiffCostFunction<PointToPlaneFunctor, 1, 4, 3>;

        explicit ICPOptimizationBuilder(const CTICPOptions *options,
                                        const std::vector<Eigen::Vector3d> &raw_points,
                                        const std::vector<Eigen::Vector3d> &world_points,
                                        const std::vector<double> &timestamps) :
                options_(options),
                timestamps_(timestamps),
                raw_points_(raw_points),
                world_points_(world_points) {
            corrected_raw_points_.resize(world_points.size());
            for (int i(0); i < raw_points_.size(); ++i)
                corrected_raw_points_[i] = raw_points_[i];

            max_num_residuals_ = options->max_num_residuals;
        }

        bool InitProblem(int num_residuals) {
            problem = std::make_unique<ceres::Problem>();
            parameter_block_set_ = false;

            // Select Loss function
            switch (options_->loss_function) {
                case LEAST_SQUARES::STANDARD:
                    break;
                case LEAST_SQUARES::CAUCHY:
                    loss_function = new ceres::CauchyLoss(options_->ls_sigma);
                    break;
                case LEAST_SQUARES::HUBER:
                    loss_function = new ceres::HuberLoss(options_->ls_sigma);
                    break;
                case LEAST_SQUARES::TOLERANT:
                    loss_function = new ceres::TolerantLoss(options_->ls_tolerant_min_threshold,
                                                            options_->ls_sigma);
                    break;
                case LEAST_SQUARES::TRUNCATED:
                    loss_function = new ct_icp::TruncatedLoss(options_->ls_sigma);
                    break;
            }

            // Resize the number of residuals
            vector_ct_icp_residuals_.resize(num_residuals);
            vector_cost_functors_.resize(num_residuals);
            begin_quat_ = nullptr;
            end_quat_ = nullptr;
            begin_t_ = nullptr;
            end_t_ = nullptr;

            return true;
        }

        void DistortFrame(slam::Pose &begin_pose, slam::Pose &end_pose) {
            if (options_->distance == POINT_TO_PLANE) {
                // Distorts the frame (put all raw_points in the coordinate frame of the pose at the end of the acquisition)
                auto end_pose_I = end_pose.Inverse().pose; // Rotation of the inverse pose

                for (int i(0); i < world_points_.size(); ++i) {
                    Eigen::Vector3d raw_point = raw_points_[i];
                    double timestamp = timestamps_[i];
                    auto interpolated_pose = begin_pose.InterpolatePose(end_pose, timestamp);

                    // Distort Raw Keypoints
                    corrected_raw_points_[i] = end_pose_I * (interpolated_pose * raw_point);
                }
            }
        }

        inline void AddParameterBlocks(Eigen::Quaterniond &begin_quat, Eigen::
        Quaterniond &end_quat, Eigen::Vector3d &begin_t, Eigen::Vector3d &end_t) {
            CHECK(!parameter_block_set_) << "The parameter block was already set";
            auto *parameterization = new ceres::EigenQuaternionParameterization();
            begin_t_ = &begin_t.x();
            end_t_ = &end_t.x();
            begin_quat_ = &begin_quat.x();
            end_quat_ = &end_quat.x();

            switch (options_->distance) {
                case CT_POINT_TO_PLANE:
                    problem->AddParameterBlock(begin_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(begin_t_, 3);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
                case POINT_TO_PLANE:
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
            }

            parameter_block_set_ = true;
        }


        inline void SetResidualBlock(int residual_id,
                                     int keypoint_id,
                                     const Eigen::Vector3d &reference_point,
                                     const Eigen::Vector3d &reference_normal,
                                     double weight = 1.0,
                                     double alpha_timestamp = -1.0) {

            CTPointToPlaneFunctor *ct_point_to_plane_functor = nullptr;
            PointToPlaneFunctor *point_to_plane_functor = nullptr;
            void *cost_functor = nullptr;
            void *cost_function = nullptr;
            if (alpha_timestamp < 0 || alpha_timestamp > 1)
                throw std::runtime_error("BAD ALPHA TIMESTAMP !");
            switch (options_->distance) {
                case CT_POINT_TO_PLANE:
                    ct_point_to_plane_functor = new CTPointToPlaneFunctor(reference_point,
                                                                          corrected_raw_points_[keypoint_id],
                                                                          reference_normal,
                                                                          alpha_timestamp, weight);
                    cost_functor = ct_point_to_plane_functor;
                    cost_function = static_cast<void *>(new CTICP_PointToPlaneResidual(ct_point_to_plane_functor));
                    break;
                case POINT_TO_PLANE:
                    point_to_plane_functor = new PointToPlaneFunctor(reference_point,
                                                                     corrected_raw_points_[keypoint_id],
                                                                     reference_normal,
                                                                     weight);
                    cost_functor = point_to_plane_functor;
                    cost_function = static_cast<void *>(new PointToPlaneResidual(point_to_plane_functor));
                    break;
            }
            vector_ct_icp_residuals_[residual_id] = cost_function;
            vector_cost_functors_[residual_id] = cost_functor;
        }


        std::unique_ptr<ceres::Problem> GetProblem(int &out_number_of_residuals) {
            out_number_of_residuals = 0;
            for (auto &pt_to_plane_residual: vector_ct_icp_residuals_) {
                if (pt_to_plane_residual != nullptr) {
                    if (max_num_residuals_ <= 0 || out_number_of_residuals < max_num_residuals_) {

                        switch (options_->distance) {
                            case CT_POINT_TO_PLANE:
                                problem->AddResidualBlock(
                                        static_cast<CTICP_PointToPlaneResidual *>(pt_to_plane_residual), loss_function,
                                        begin_quat_, begin_t_, end_quat_, end_t_);
                                break;
                            case POINT_TO_PLANE:
                                problem->AddResidualBlock(
                                        static_cast<PointToPlaneResidual *>(pt_to_plane_residual), loss_function,
                                        end_quat_, end_t_);
                                break;
                        }
                        out_number_of_residuals++;
                    } else {
                        // Need to deallocate memory from the allocated pointers not managed by Ceres
                        CTICP_PointToPlaneResidual *ct_pt_to_pl_ptr = nullptr;
                        PointToPlaneResidual *pt_to_pl_ptr = nullptr;
                        switch (options_->distance) {
                            case CT_POINT_TO_PLANE:
                                ct_pt_to_pl_ptr = static_cast<CTICP_PointToPlaneResidual *>(pt_to_plane_residual);
                                delete ct_pt_to_pl_ptr;
                                break;
                            case POINT_TO_PLANE:
                                pt_to_pl_ptr = static_cast<PointToPlaneResidual *>(pt_to_plane_residual);
                                delete pt_to_pl_ptr;
                                break;
                        }
                    }
                }
            }

            std::fill(vector_cost_functors_.begin(), vector_cost_functors_.end(), nullptr);
            std::fill(vector_ct_icp_residuals_.begin(), vector_ct_icp_residuals_.end(), nullptr);

            return std::move(problem);
        }

    private:
        const CTICPOptions *options_;
        std::unique_ptr<ceres::Problem> problem = nullptr;
        int max_num_residuals_ = -1;

        // Parameters block pointers
        bool parameter_block_set_ = false;
        double *begin_quat_ = nullptr;
        double *end_quat_ = nullptr;
        double *begin_t_ = nullptr;
        double *end_t_ = nullptr;

        // Pointers managed by ceres
        const std::vector<Eigen::Vector3d> &world_points_;
        const std::vector<Eigen::Vector3d> &raw_points_;
        const std::vector<double> &timestamps_;
        std::vector<Eigen::Vector3d> corrected_raw_points_;

        std::vector<void *> vector_cost_functors_;
        std::vector<void *> vector_ct_icp_residuals_;
        ceres::LossFunction *loss_function = nullptr;
    };

    ICPSummary CT_ICP_Registration::DoRegisterCeres(const VoxelHashMap &voxels_map,
                                                std::vector<Eigen::Vector3d> &raw_kpts,
                                                std::vector<Eigen::Vector3d> &world_kpts,
                                                std::vector<double> &timestamps,
                                                TrajectoryFrame &frame_to_optimize,
                                                const TrajectoryFrame *const _previous_frame) {
        size_t num_points = raw_kpts.size();
        auto &options = Options();

        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();
        const short nb_voxels_visited = options.voxel_neighborhood;
        const int kMinNumNeighbors = options.min_number_neighbors;
        const int kThresholdCapacity = options.threshold_voxel_occupancy;

        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = options.ls_max_num_iters;
        ceres_options.num_threads = options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
        Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();
        if (_previous_frame) {
            previous_velocity = _previous_frame->EndTr() - _previous_frame->BeginTr();
            previous_orientation = _previous_frame->EndQuat();
        }

        auto &begin_pose = frame_to_optimize.begin_pose;
        auto &end_pose = frame_to_optimize.end_pose;
        auto &begin_quat = frame_to_optimize.begin_pose.QuatRef();
        auto &begin_t = frame_to_optimize.begin_pose.TrRef();
        auto &end_quat = frame_to_optimize.end_pose.QuatRef();
        auto &end_t = frame_to_optimize.end_pose.TrRef();

        auto previous_begin_pose = frame_to_optimize.begin_pose.pose;
        auto previous_end_pose = frame_to_optimize.end_pose.pose;

        int number_of_residuals;

        ICPOptimizationBuilder builder(&options, raw_kpts, world_kpts, timestamps);
        if (options.point_to_plane_with_distortion) {
            builder.DistortFrame(begin_pose, end_pose);
        }

        auto transform_keypoints = [&]() {
            // Elastically distorts the frame to improve on Neighbor estimation
            for (auto i(0); i < num_points; ++i) {
                if (options.point_to_plane_with_distortion ||
                    options.distance == CT_POINT_TO_PLANE) {
                    double timestamp = timestamps[i];
                    auto world_point_proxy = world_kpts[i];
                    auto interpolated_pose = frame_to_optimize.begin_pose.InterpolatePose(
                            frame_to_optimize.end_pose, timestamp);
                    world_point_proxy = interpolated_pose * raw_kpts[i];
                } else {
                    auto world_point_proxy = world_kpts[i];
                    world_point_proxy = frame_to_optimize.end_pose * raw_kpts[i];
                }
            }
        };

        auto estimate_point_neighborhood = [&](ArrayVector3d &vector_neighbors,
                                               Eigen::Vector3d &location,
                                               double &planarity_weight) {

            auto neighborhood = compute_neighborhood_distribution(vector_neighbors);
            planarity_weight = std::pow(neighborhood.a2D, options.power_planarity);

            if (neighborhood.normal.dot(frame_to_optimize.BeginTr() - location) < 0) {
                neighborhood.normal = -1.0 * neighborhood.normal;
            }
            return neighborhood;
        };

        double lambda_weight = std::abs(options.weight_alpha);
        double lambda_neighborhood = std::abs(options.weight_neighborhood);
        const double kMaxPointToPlane = options.max_dist_to_plane_ct_icp;
        const double sum = lambda_weight + lambda_neighborhood;
        CHECK(sum > 0.0) << "Invalid requirement: weight_alpha(" << options.weight_alpha <<
                         ") + weight_neighborhood(" << options.weight_neighborhood << ") <= 0 " << std::endl;
        lambda_weight /= sum;
        lambda_neighborhood /= sum;

        for (int iter(0); iter < options.num_iters_icp; iter++) {
            transform_keypoints();

            builder.InitProblem(num_points * options.num_closest_neighbors);
            builder.AddParameterBlocks(begin_quat, end_quat, begin_t, end_t);

            // Add Point-to-plane residuals
            int num_keypoints = num_points;
            int num_threads = options.ls_num_threads;
#pragma omp parallel for num_threads(num_threads)
            for (int k = 0; k < num_keypoints; ++k) {
                Eigen::Vector3d raw_point = raw_kpts[k];
                double timestamp = timestamps[k];
                Eigen::Vector3d world_point = world_kpts[k];

                // Neighborhood search
                std::vector<Voxel> voxels;
                auto vector_neighbors = search_neighbors(voxels_map, world_point,
                                                         nb_voxels_visited, options.size_voxel_map,
                                                         options.max_number_neighbors, kThresholdCapacity,
                                                         options.estimate_normal_from_neighborhood ? nullptr : &voxels);

                if (vector_neighbors.size() < kMinNumNeighbors)
                    continue;

                double weight;
                auto neighborhood = estimate_point_neighborhood(vector_neighbors,
                                                                raw_point,
                                                                weight);

                weight = lambda_weight * weight +
                         lambda_neighborhood * std::exp(-(vector_neighbors[0] -
                                                          world_point).norm() /
                                                        (kMaxPointToPlane * kMinNumNeighbors));

                double point_to_plane_dist;
                std::set<Voxel> neighbor_voxels;
                for (int i(0); i < options.num_closest_neighbors; ++i) {
                    point_to_plane_dist = std::abs(
                            (world_point - vector_neighbors[i]).transpose() * neighborhood.normal);
                    if (point_to_plane_dist < options.max_dist_to_plane_ct_icp) {
                        builder.SetResidualBlock(options.num_closest_neighbors * k + i, k,
                                                 vector_neighbors[i],
                                                 neighborhood.normal, weight,
                                                 begin_pose.GetAlphaTimestamp(timestamp, end_pose));
                    }
                }
            }

            auto problem = builder.GetProblem(number_of_residuals);

            if (_previous_frame && options.distance == CT_POINT_TO_PLANE) {
                // Add Regularisation residuals
                problem->AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistencyFunctor,
                                                  LocationConsistencyFunctor::NumResiduals(), 3>(
                                                  new LocationConsistencyFunctor(_previous_frame->EndTr(),
                                                                                 sqrt(number_of_residuals *
                                                                                      options.beta_location_consistency))),
                                          nullptr,
                                          &begin_t.x());
                problem->AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityFunctor,
                                                  ConstantVelocityFunctor::NumResiduals(), 3, 3>(
                                                  new ConstantVelocityFunctor(previous_velocity,
                                                                              sqrt(number_of_residuals * options.beta_constant_velocity))),
                                          nullptr,
                                          &begin_t.x(),
                                          &end_t.x());

                // SMALL VELOCITY
                problem->AddResidualBlock(new ceres::AutoDiffCostFunction<SmallVelocityFunctor,
                                                  SmallVelocityFunctor::NumResiduals(), 3, 3>(
                                                  new SmallVelocityFunctor(sqrt(number_of_residuals * options.beta_small_velocity))),
                                          nullptr,
                                          &begin_t.x(), &end_t.x());

                // ORIENTATION CONSISTENCY
                problem->AddResidualBlock(new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor,
                                                  OrientationConsistencyFunctor::NumResiduals(), 4>(
                                                  new OrientationConsistencyFunctor(previous_orientation,
                                                                                    sqrt(number_of_residuals *
                                                                                         options.beta_orientation_consistency))),
                                          nullptr,
                                          &begin_quat.x());
            }
            if (number_of_residuals < options.min_number_neighbors) {
                std::stringstream ss_out;
                ss_out << "[CT_ICP] Error : not enough keypoints selected in ct-icp !" << std::endl;
                ss_out << "[CT_ICP] number_of_residuals : " << number_of_residuals << std::endl;
                ICPSummary summary;
                summary.success = false;
                summary.num_residuals_used = number_of_residuals;
                summary.error_log = ss_out.str();
                if (options.debug_print) {
                    std::cout << summary.error_log;
                }
                return summary;
            }

            ceres::Solver::Summary summary;
            ceres::Solve(ceres_options, problem.get(), &summary);

            frame_to_optimize.begin_pose.pose.quat.normalize();
            frame_to_optimize.end_pose.pose.quat.normalize();

            if (!summary.IsSolutionUsable()) {
                std::cout << summary.FullReport() << std::endl;
                throw std::runtime_error("Error During Optimization");
            }
            if (options.debug_print) {
                std::cout << summary.BriefReport() << std::endl;
            }

            begin_quat.normalize();
            end_quat.normalize();

            double diff_trans = (previous_begin_pose.tr - frame_to_optimize.BeginTr()).norm() +
                                (previous_end_pose.tr - frame_to_optimize.EndTr()).norm();
            double diff_rot = slam::AngularDistance(frame_to_optimize.begin_pose.pose, previous_begin_pose) +
                              slam::AngularDistance(frame_to_optimize.end_pose.pose, previous_end_pose);

            previous_begin_pose = frame_to_optimize.begin_pose.pose;
            previous_end_pose = frame_to_optimize.end_pose.pose;

            if (options.point_to_plane_with_distortion) {
                builder.DistortFrame(begin_pose, end_pose);
            }

            if ((diff_rot < options.threshold_orientation_norm && diff_trans < options.threshold_translation_norm)) {
                if (options.debug_print)
                    std::cout << "CT_ICP: Finished with N=" << iter << " ICP iterations" << std::endl;

                break;
            } else if (options.debug_print) {
                std::cout << "[CT-ICP]: Rotation diff: " << diff_rot << "(deg)" << std::endl;
                std::cout << "[CT-ICP]: Translation diff: " << diff_trans << "(m)" << std::endl;
            }
        }
        transform_keypoints();

        ICPSummary summary;
        summary.success = true;
        summary.num_residuals_used = number_of_residuals;

        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();

        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::Register(const VoxelHashMap &voxel_map, std::vector<pandar_ros::WPoint3D> &keypoints,
                                             TrajectoryFrame &trajectory_frame,
                                             const TrajectoryFrame *const previous_frame) {
        std::vector<Eigen::Vector3d> raw_points;
        std::vector<Eigen::Vector3d> world_points;
        std::vector<double> timestamps;
        for (int i = 0; i < keypoints.size(); i++){
            raw_points.push_back(Eigen::Vector3d(keypoints[i].raw_point.x, keypoints[i].raw_point.y, keypoints[i].raw_point.z));
            world_points.push_back(keypoints[i].w_point);
            timestamps.push_back(keypoints[i].raw_point.timestamp);
        }
        DoRegisterCeres(voxel_map, raw_points, world_points, timestamps, trajectory_frame, previous_frame);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::Register(const VoxelHashMap &voxel_map,
                                             pcl::PointCloud<pandar_ros::WPoint3D> &keypoints,
                                             TrajectoryFrame &trajectory_frame,
                                             const TrajectoryFrame *const previous_frame) {
        std::vector<Eigen::Vector3d> raw_points;
        std::vector<Eigen::Vector3d> world_points;
        std::vector<double> timestamps;
        for (pandar_ros::WPoint3D point : keypoints.points) {
            raw_points.push_back(Eigen::Vector3d(point.raw_point.x, point.raw_point.y, point.raw_point.z));
            world_points.push_back(point.w_point);
            timestamps.push_back(point.raw_point.timestamp);
        }
        DoRegisterCeres(voxel_map, raw_points, world_points, timestamps, trajectory_frame, previous_frame);
    }
}