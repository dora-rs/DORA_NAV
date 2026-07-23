#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <vector>
#include <boost/optional.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <Eigen/Geometry>

namespace kkl {
  namespace alg {
template<typename T, class System> class UnscentedKalmanFilterX;
  }
}

namespace hdl_localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief outcome of the last scan-matching correction attempt
   */
  enum class CorrectionStatus {
    None,                // no correction attempted yet
    Corrected,           // NDT accepted, UKF updated
    SkippedHighFitness,  // NDT converged but fitness above threshold
    NotConverged         // NDT failed to converge
  };

  /**
   * @brief constructor
   * @param registration        registration method
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0);
  ~PoseEstimator();

  /**
   * @brief predict
   * @param timestamp    timestamp in seconds
   */
  void predict(double timestamp);

  /**
   * @brief predict
   * @param timestamp    timestamp in seconds
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(double timestamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  /**
   * @brief update the state of the odomety-based pose estimation
   */
  void predict_odom(const Eigen::Matrix4f& odom_delta);

  /**
   * @brief correct
   * @param timestamp   timestamp in seconds
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */
  pcl::PointCloud<PointT>::Ptr correct(double timestamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  /* getters */
  double last_correction_time() const;
  double last_fitness() const { return last_fitness_; }
  CorrectionStatus last_status() const { return last_status_; }

  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;

  Eigen::Vector3f odom_pos() const;
  Eigen::Quaternionf odom_quat() const;
  Eigen::Matrix4f odom_matrix() const;

  const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& odom_prediction_error() const;

private:
  double init_stamp;             // when the estimator was initialized
  double prev_stamp;             // when the estimator was updated last time
  double last_correction_stamp;  // when the estimator performed the correction step
  double cool_time_duration;
  double max_fitness_score;      // reject NDT correction if fitness score exceeds this
  double last_fitness_ = 0.0;                                  // fitness score of the last correction attempt
  CorrectionStatus last_status_ = CorrectionStatus::None;      // outcome of the last correction attempt

  // Gravity alignment: collect IMU samples during cool_time, align once at end
  std::vector<Eigen::Vector3f> gravity_calib_samples_;
  bool gravity_aligned_;

  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;

  Eigen::Matrix4f last_observation;
  boost::optional<Eigen::Matrix4f> wo_pred_error;
  boost::optional<Eigen::Matrix4f> imu_pred_error;
  boost::optional<Eigen::Matrix4f> odom_pred_error;

  pcl::Registration<PointT, PointT>::Ptr registration;
  };

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP
