#include <hdl_localization/pose_estimator.hpp>

#include <cmath>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>
#include "config_manager.h"

namespace hdl_localization {

/**
 * @brief constructor
 * @param registration        registration method
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 */
PoseEstimator::PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration)
    : registration(registration),
      cool_time_duration(cool_time_duration),
      init_stamp(0.0),
      prev_stamp(0.0),
      last_correction_stamp(0.0),
      gravity_aligned_(false) {
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;

  // Load noise parameters from config (with sensible defaults)
  ConfigManager& cfg = ConfigManager::getInstance();
  const bool imu_enabled = cfg.isImuEnabled();
  max_fitness_score = cfg.getNdtMaxFitnessScore();

  const double pn_pos   = cfg.getUkfProcessNoisePosition();
  // When IMU is enabled, velocity is driven by acc integration → allow larger velocity noise
  // When IMU is disabled, keep velocity noise small to avoid drift from blind prediction
  const double pn_vel   = imu_enabled ? cfg.getUkfProcessNoiseVelocity() : 0.1;
  const double pn_ori   = cfg.getUkfProcessNoiseOrientation();
  const double pn_ab    = cfg.getUkfProcessNoiseAccBias();
  const double pn_gb    = cfg.getUkfProcessNoiseGyroBias();
  const double mn_pos   = cfg.getUkfMeasurementNoisePosition();
  const double mn_ori   = cfg.getUkfMeasurementNoiseOrientation();

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3)  *= static_cast<float>(pn_pos);
  process_noise.middleRows(3, 3)  *= static_cast<float>(pn_vel);
  process_noise.middleRows(6, 4)  *= static_cast<float>(pn_ori);
  process_noise.middleRows(10, 3) *= static_cast<float>(pn_ab);
  process_noise.middleRows(13, 3) *= static_cast<float>(pn_gb);

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= static_cast<float>(mn_pos);
  measurement_noise.middleRows(3, 4) *= static_cast<float>(mn_ori);

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z()).normalized();
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * @param timestamp    timestamp in seconds
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(double timestamp) {
  if (init_stamp == 0.0) {
    init_stamp = timestamp;
  }

  if ((timestamp - init_stamp) < cool_time_duration || prev_stamp == 0.0 || prev_stamp == timestamp) {
    prev_stamp = timestamp;
    return;
  }

  double dt = timestamp - prev_stamp;
  prev_stamp = timestamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
}

/**
 * @brief predict
 * @param timestamp    timestamp in seconds
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(double timestamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  if (init_stamp == 0.0) {
    init_stamp = timestamp;
  }

  if ((timestamp - init_stamp) < cool_time_duration || prev_stamp == 0.0 || prev_stamp == timestamp) {
    // Collect IMU samples during cool_time for gravity alignment
    gravity_calib_samples_.push_back(acc);
    prev_stamp = timestamp;
    return;
  }

  // First frame after cool_time: perform gravity alignment once
  if (!gravity_aligned_) {
    gravity_aligned_ = true;
    if (!gravity_calib_samples_.empty()) {
      Eigen::Vector3f mean_acc = Eigen::Vector3f::Zero();
      for (const auto& a : gravity_calib_samples_) mean_acc += a;
      mean_acc /= static_cast<float>(gravity_calib_samples_.size());

      Eigen::Quaternionf align = Eigen::Quaternionf::FromTwoVectors(mean_acc.normalized(), Eigen::Vector3f(0.0f, 0.0f, 1.0f));
      Eigen::Quaternionf q_cur(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
      Eigen::Quaternionf q_new = (align * q_cur).normalized();
      ukf->mean[6] = q_new.w();
      ukf->mean[7] = q_new.x();
      ukf->mean[8] = q_new.y();
      ukf->mean[9] = q_new.z();
    }
    gravity_calib_samples_.clear();
    gravity_calib_samples_.shrink_to_fit();
  }

  double dt = timestamp - prev_stamp;
  prev_stamp = timestamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;

  ukf->predict(control);
}

/**
 * @brief update the state of the odomety-based pose estimation
 */
void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta) {
  if(!odom_ukf) {
    Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
    odom_mean.block<4, 1>(3, 0) = Eigen::Vector4f(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    OdomSystem odom_system;
    odom_ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  // invert quaternion if the rotation axis is flipped
  Eigen::Quaternionf quat(odom_delta.block<3, 3>(0, 0));
  if(odom_quat().coeffs().dot(quat.coeffs()) < 0.0) {
    quat.coeffs() *= -1.0f;
  }

  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = odom_delta.block<3, 1>(0, 3);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity() * odom_delta.block<3, 1>(0, 3).norm() + Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1 - std::abs(quat.w())) + Eigen::Matrix4f::Identity() * 1e-3;

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control);
}

/**
 * @brief correct
 * @param timestamp   timestamp in seconds
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(double timestamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  if (init_stamp == 0.0) {
    init_stamp = timestamp;
  }

  last_correction_stamp = timestamp;

  Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f imu_guess;
  Eigen::Matrix4f odom_guess;
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  if(!odom_ukf) {
    init_guess = imu_guess = matrix();
  } else {
    imu_guess = matrix();
    odom_guess = odom_matrix();

    Eigen::VectorXf imu_mean(7);
    Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
    imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
    imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

    imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
    imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
    imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
    imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

    Eigen::VectorXf odom_mean = odom_ukf->mean;
    Eigen::MatrixXf odom_cov = odom_ukf->cov;

    if (imu_mean.tail<4>().dot(odom_mean.tail<4>()) < 0.0) {
      odom_mean.tail<4>() *= -1.0;
    }

    Eigen::MatrixXf inv_imu_cov = imu_cov.inverse();
    Eigen::MatrixXf inv_odom_cov = odom_cov.inverse();

    Eigen::MatrixXf fused_cov = (inv_imu_cov + inv_odom_cov).inverse();
    Eigen::VectorXf fused_mean = fused_cov * inv_imu_cov * imu_mean + fused_cov * inv_odom_cov * odom_mean;

    init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(fused_mean[0], fused_mean[1], fused_mean[2]);
    init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(fused_mean[3], fused_mean[4], fused_mean[5], fused_mean[6]).normalized().toRotationMatrix();
  }

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);

  // Reject bad scan matching results to prevent UKF state corruption
  if (!registration->hasConverged()) {
    last_fitness_ = std::numeric_limits<double>::infinity();
    last_status_ = CorrectionStatus::NotConverged;
    last_observation = init_guess;
    return aligned;
  }

  double fitness = registration->getFitnessScore();
  last_fitness_ = fitness;
  if (fitness > max_fitness_score) {
    last_status_ = CorrectionStatus::SkippedHighFitness;
    last_observation = init_guess;
    return aligned;
  }
  last_status_ = CorrectionStatus::Corrected;

  Eigen::Matrix4f trans = registration->getFinalTransformation();
  Eigen::Vector3f p = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

  if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }

  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  last_observation = trans;

  wo_pred_error = no_guess.inverse() * registration->getFinalTransformation();

  ukf->correct(observation);

  // Soft-enforce Vz ≈ 0 for ground vehicle (~0.22 m/s 1-sigma tolerance)
  // Only active after gravity alignment to avoid fighting initial orientation correction
  if (gravity_aligned_) {
    constexpr float R_vz = 0.05f;
    float S = ukf->cov(5, 5) + R_vz;
    Eigen::Matrix<float, 16, 1> K = ukf->cov.col(5) / S;
    ukf->mean -= K * ukf->mean[5];
    ukf->cov  -= K * ukf->cov.row(5);
  }

  imu_pred_error = imu_guess.inverse() * registration->getFinalTransformation();

  if(odom_ukf) {
    if (observation.tail<4>().dot(odom_ukf->mean.tail<4>()) < 0.0) {
      odom_ukf->mean.tail<4>() *= -1.0;
    }

    odom_ukf->correct(observation);
    odom_pred_error = odom_guess.inverse() * registration->getFinalTransformation();
  }

  return aligned;
}

/* getters */
double PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

Eigen::Vector3f PoseEstimator::odom_pos() const {
  return Eigen::Vector3f(odom_ukf->mean[0], odom_ukf->mean[1], odom_ukf->mean[2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat() const {
  return Eigen::Quaternionf(odom_ukf->mean[3], odom_ukf->mean[4], odom_ukf->mean[5], odom_ukf->mean[6]).normalized();
}

Eigen::Matrix4f PoseEstimator::odom_matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = odom_quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = odom_pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}
}