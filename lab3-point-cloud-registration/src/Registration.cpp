#include "Registration.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

struct PointDistance {
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // This class should include an auto-differentiable cost function.
    // To rotate a point given an axis-angle rotation, use
    // the Ceres function:
    // AngleAxisRotatePoint(...) (see ceres/rotation.h)
    // Similarly to the Bundle Adjustment case initialize the struct variables with the source and the target point.
    // You have to optimize only the 6-dimensional array (rx, ry, rz, tx ,ty, tz).
    // WARNING: When dealing with the AutoDiffCostFunction template parameters,
    // pay attention to the order of the template parameters
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    PointDistance(const Eigen::Vector3d &source_point, const Eigen::Vector3d &target_point) {
        source_[0] = source_point(0);
        source_[1] = source_point(1);
        source_[2] = source_point(2);
        target_[0] = target_point(0);
        target_[1] = target_point(1);
        target_[2] = target_point(2);
    }

    template <typename T>
    bool operator()(const T *const transformation, T *residuals) const {
        T source_point[3] = {T(source_[0]), T(source_[1]), T(source_[2])};
        T rotated_point[3];
        ceres::AngleAxisRotatePoint(transformation, source_point, rotated_point);

        residuals[0] = rotated_point[0] + transformation[3] - T(target_[0]);
        residuals[1] = rotated_point[1] + transformation[4] - T(target_[1]);
        residuals[2] = rotated_point[2] + transformation[5] - T(target_[2]);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &source_point, const Eigen::Vector3d &target_point) {
        return new ceres::AutoDiffCostFunction<PointDistance, 3, 6>(
            new PointDistance(source_point, target_point));
    }

    double source_[3];
    double target_[3];
};

Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename) {
    open3d::io::ReadPointCloud(cloud_source_filename, source_);
    open3d::io::ReadPointCloud(cloud_target_filename, target_);
    Eigen::Vector3d gray_color;
    source_for_icp_ = source_;
}

Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target) {
    source_ = cloud_source;
    target_ = cloud_target;
    source_for_icp_ = source_;
}

void Registration::draw_registration_result() {
    // clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    // different color
    Eigen::Vector3d color_s;
    Eigen::Vector3d color_t;
    color_s << 1, 0.706, 0;
    color_t << 0, 0.651, 0.929;

    target_clone.PaintUniformColor(color_t);
    source_clone.PaintUniformColor(color_s);
    source_clone.Transform(transformation_);

    auto src_pointer = std::make_shared<open3d::geometry::PointCloud>(source_clone);
    auto target_pointer = std::make_shared<open3d::geometry::PointCloud>(target_clone);
    open3d::visualization::DrawGeometries({src_pointer, target_pointer});
    return;
}

ICPResult Registration::execute_icp_registration(double threshold, int max_iteration, double relative_rmse, std::string mode) {
    std::cout << "Starting ICP" << std::endl;
    ICPResult result;

    if (mode == "svd" or mode == "lm") {
        auto start = std::chrono::steady_clock::now();
        source_for_icp_.Transform(transformation_);
        double prev_rmse = std::numeric_limits<double>::infinity();
        int it;
        for (it = 0; it < max_iteration; ++it) {
            std::tuple<std::vector<size_t>, std::vector<size_t>, double> res = find_closest_point(threshold);
            auto source_indices = std::get<0>(res);
            auto target_indices = std::get<1>(res);
            double rmse = std::get<2>(res);
            std::cout << '\r' << "ICP Inlier RMSE: " << rmse << std::flush;

            if (prev_rmse - rmse < relative_rmse)
                break;
            prev_rmse = rmse;
            Eigen::Matrix4d transformation;
            if (mode == "svd")
                transformation = get_svd_icp_transformation(source_indices, target_indices);
            else if (mode == "lm")
                transformation = get_lm_icp_transformation(source_indices, target_indices);
            source_for_icp_.Transform(transformation);
            transformation_ = transformation * transformation_; //FIXED
        }
        std::cout << std::endl;
        auto end = std::chrono::steady_clock::now();
        auto time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double final_rmse = compute_rmse();
        result = {final_rmse, time_ms, it};
    } else if (mode.rfind("o3d") == 0) { // if open3d ICP method is selected
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug); // Set to Debug to get detailed information about the ICP process
        std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> transformation_estimation;
        if (mode == "o3d-p2point") {
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationPointToPoint>();
        } else if (mode == "o3d-p2plane") {
            target_.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(3, 30)); // hardcoded parameters for normal estimation, you can change them if you want
            target_.NormalizeNormals();
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationPointToPlane>();
        } else if (mode == "o3d-gen") {
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationForGeneralizedICP>();
        } else {
            std::cerr << "Unknown Open3D ICP mode: " << mode << std::endl;
            return {0.0, 0.0, 0};
        }
        auto start = std::chrono::steady_clock::now();
        auto reg_p2p = open3d::pipelines::registration::RegistrationICP(
            source_for_icp_,
            target_,
            threshold,
            transformation_,
            *transformation_estimation,
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, relative_rmse, max_iteration)); // FIXED
        transformation_ = reg_p2p.transformation_;
        auto end = std::chrono::steady_clock::now();
        auto time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double final_rmse = compute_rmse();
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Info); // Reset verbosity level to default
        target_.normals_.clear(); // Clear normals if they were estimated for point-to-plane ICP to avoid affecting subsequent registrations
        result = {final_rmse, time_ms, 0}; // NB: unfortunately Open3D's RegistrationICP does not provide the number of iterations, so we set it to 0, you can watch the debug output to see how many iterations it performed
    } else {
        std::cerr << "Unknown ICP mode: " << mode << std::endl;
        result = {0.0, 0.0, 0};
    }

    source_for_icp_ = source_;
    return result;
}

std::tuple<std::vector<size_t>, std::vector<size_t>, double> Registration::find_closest_point(double threshold) {
    std::vector<size_t> source_indices;
    std::vector<size_t> target_indices;

    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    size_t num_source_points = source_for_icp_.points_.size();

    std::vector<int> idx(1);
    std::vector<double> dist2(1);
    double threshold_sq = threshold * threshold;
    double sum_sq = 0.0;

    for (size_t i = 0; i < num_source_points; ++i) {
        target_kd_tree.SearchKNN(source_for_icp_.points_[i], 1, idx, dist2);
        if (dist2[0] < threshold_sq) {
            source_indices.push_back(i);
            target_indices.push_back(static_cast<size_t>(idx[0]));
            sum_sq += dist2[0];
        }
    }

    double rmse = 0.0;
    if (!source_indices.empty()) {
        rmse = std::sqrt(sum_sq / static_cast<double>(source_indices.size()));
    }

    return {source_indices, target_indices, rmse};
}

Eigen::Matrix4d Registration::get_svd_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    int n = static_cast<int>(source_indices.size());

    Eigen::Vector3d centroid_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_t = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        centroid_s += source_for_icp_.points_[source_indices[i]];
        centroid_t += target_.points_[target_indices[i]];
    }
    centroid_s /= static_cast<double>(n);
    centroid_t /= static_cast<double>(n);

    Eigen::MatrixXd P_s(n, 3);
    Eigen::MatrixXd P_t(n, 3);
    for (int i = 0; i < n; ++i) {
        P_s.row(i) = (source_for_icp_.points_[source_indices[i]] - centroid_s).transpose();
        P_t.row(i) = (target_.points_[target_indices[i]] - centroid_t).transpose();
    }

    Eigen::Matrix3d H = P_s.transpose() * P_t;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0.0) {
        Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
        correction(2, 2) = -1.0;
        R = V * correction * U.transpose();
    }

    Eigen::Vector3d t = centroid_t - R * centroid_s;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;
    return transformation;
}

Eigen::Matrix4d Registration::get_lm_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    double transformation_array[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    ceres::Problem problem;
    int n = static_cast<int>(source_indices.size());
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector3d &source_point = source_for_icp_.points_[source_indices[i]];
        const Eigen::Vector3d &target_point = target_.points_[target_indices[i]];
        ceres::CostFunction *cost_function = PointDistance::Create(source_point, target_point);
        problem.AddResidualBlock(cost_function, nullptr, transformation_array);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 50;
    options.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double rotation_matrix[9];
    ceres::AngleAxisToRotationMatrix(transformation_array, rotation_matrix);

    Eigen::Matrix3d R;
    R << rotation_matrix[0], rotation_matrix[3], rotation_matrix[6],
         rotation_matrix[1], rotation_matrix[4], rotation_matrix[7],
         rotation_matrix[2], rotation_matrix[5], rotation_matrix[8];

    Eigen::Vector3d t(transformation_array[3], transformation_array[4], transformation_array[5]);

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;
    return transformation;
}

void Registration::execute_descriptor_registration() {
    auto bbox_s = source_.GetAxisAlignedBoundingBox();
    auto bbox_t = target_.GetAxisAlignedBoundingBox();
    double extent_s = (bbox_s.max_bound_ - bbox_s.min_bound_).norm();
    double extent_t = (bbox_t.max_bound_ - bbox_t.min_bound_).norm();
    double extent = std::max(extent_s, extent_t);

    double voxel_size = extent * 0.02;
    double normal_radius = voxel_size * 2.0;
    double feature_radius = voxel_size * 5.0;
    double ransac_distance = voxel_size * 1.5;

    std::cout << "Descriptor registration parameters:"
              << " voxel=" << voxel_size
              << " normal_r=" << normal_radius
              << " feature_r=" << feature_radius
              << " ransac_d=" << ransac_distance << std::endl;

    auto source_down = source_.VoxelDownSample(voxel_size);
    auto target_down = target_.VoxelDownSample(voxel_size);

    std::cout << "Downsampled: source=" << source_down->points_.size()
              << " target=" << target_down->points_.size() << std::endl;

    source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_radius, 30));
    target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_radius, 30));
    source_down->NormalizeNormals();
    target_down->NormalizeNormals();

    auto source_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *source_down, open3d::geometry::KDTreeSearchParamHybrid(feature_radius, 100));
    auto target_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *target_down, open3d::geometry::KDTreeSearchParamHybrid(feature_radius, 100));

    // Mutual nearest-neighbour matching in the 33-D FPFH feature space.
    open3d::geometry::KDTreeFlann source_feat_tree(*source_fpfh);
    open3d::geometry::KDTreeFlann target_feat_tree(*target_fpfh);

    int n_src = static_cast<int>(source_fpfh->data_.cols());
    int n_tgt = static_cast<int>(target_fpfh->data_.cols());

    std::vector<int> src_to_tgt(n_src, -1);
    for (int i = 0; i < n_src; ++i) {
        std::vector<int> idx(1);
        std::vector<double> dist2(1);
        Eigen::VectorXd query = source_fpfh->data_.col(i);
        target_feat_tree.SearchKNN(query, 1, idx, dist2);
        src_to_tgt[i] = idx[0];
    }

    open3d::pipelines::registration::CorrespondenceSet correspondences;
    for (int j = 0; j < n_tgt; ++j) {
        std::vector<int> idx(1);
        std::vector<double> dist2(1);
        Eigen::VectorXd query = target_fpfh->data_.col(j);
        source_feat_tree.SearchKNN(query, 1, idx, dist2);
        int i = idx[0];
        if (src_to_tgt[i] == j) {
            correspondences.push_back(Eigen::Vector2i(i, j));
        }
    }

    std::cout << "Descriptor matching: " << correspondences.size()
              << " mutual correspondences." << std::endl;

    if (static_cast<int>(correspondences.size()) < 3) {
        std::cerr << "Too few mutual correspondences. Keeping identity transformation." << std::endl;
        transformation_ = Eigen::Matrix4d::Identity();
        return;
    }

    open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength edge_checker(0.9);
    open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance dist_checker(ransac_distance);
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers;
    checkers.push_back(edge_checker);
    checkers.push_back(dist_checker);

    auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnCorrespondence(
        *source_down,
        *target_down,
        correspondences,
        ransac_distance,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
        3,
        checkers,
        open3d::pipelines::registration::RANSACConvergenceCriteria(100000, 0.999));

    transformation_ = result.transformation_;

    std::cout << "RANSAC done. fitness=" << result.fitness_
              << " inlier_rmse=" << result.inlier_rmse_ << std::endl;
}

void Registration::set_transformation(Eigen::Matrix4d init_transformation) {
    transformation_ = init_transformation;
}

Eigen::Matrix4d Registration::get_transformation() {
    return transformation_;
}

double Registration::compute_rmse() {
    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    open3d::geometry::PointCloud source_clone = source_;
    source_clone.Transform(transformation_);
    int num_source_points = source_clone.points_.size();
    Eigen::Vector3d source_point;
    std::vector<int> idx(1);
    std::vector<double> dist2(1);
    double mse = 0.0;
    for (size_t i = 0; i < num_source_points; ++i) {
        source_point = source_clone.points_[i];
        target_kd_tree.SearchKNN(source_point, 1, idx, dist2);
        mse = mse * i / (i + 1) + dist2[0] / (i + 1);
    }
    return sqrt(mse);
}

void Registration::write_tranformation_matrix(std::string filename) {
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << transformation_;
        outfile.close();
    }
}

void Registration::save_merged_cloud(std::string filename) {
    // Paint source orange, target blue so the alignment is visually clear
    // when the merged ply is opened in MeshLab.
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    Eigen::Vector3d color_s(1.0, 0.706, 0.0);
    Eigen::Vector3d color_t(0.0, 0.651, 0.929);
    source_clone.PaintUniformColor(color_s);
    target_clone.PaintUniformColor(color_t);

    source_clone.Transform(transformation_);
    open3d::geometry::PointCloud merged = target_clone + source_clone;
    open3d::io::WritePointCloud(filename, merged);
}

Eigen::Matrix4d Registration::get_noisy_transformation(double rot_noise_deg_std, double trans_noise_mm) {
    Eigen::Matrix4d T = get_transformation();

    static thread_local std::mt19937 gen(std::random_device{}());

    std::normal_distribution<double> rot_dist(0.0, rot_noise_deg_std);

    // centroid
    Eigen::Vector3d center_local = source_.GetCenter();

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    Eigen::Vector3d noise_rad( // rotation noise in radians
        rot_dist(gen) * M_PI / 180.0,
        rot_dist(gen) * M_PI / 180.0,
        rot_dist(gen) * M_PI / 180.0);

    double angle = noise_rad.norm();
    Eigen::Matrix3d R_noise = Eigen::Matrix3d::Identity();
    if (angle > 1e-12) {
        Eigen::Vector3d axis = noise_rad / angle;
        R_noise = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }

    Eigen::Matrix3d R_new = R * R_noise;

    Eigen::Vector3d dir = Eigen::Vector3d::Random().normalized();

    // Point clouds are stored in millimeters, so translation noise is applied directly in mm.
    Eigen::Vector3d t_noise = trans_noise_mm * dir;

    Eigen::Vector3d t_new = t + R * (Eigen::Matrix3d::Identity() - R_noise) * center_local + R * t_noise;

    Eigen::Matrix4d T_new = Eigen::Matrix4d::Identity();
    T_new.block<3, 3>(0, 0) = R_new;
    T_new.block<3, 1>(0, 3) = t_new;

    return T_new;
}