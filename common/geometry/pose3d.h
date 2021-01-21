#pragma once

#include <eigen3/Eigen/Dense>
#include <iomanip>

namespace common {

class Pose3d {
 public:
  Pose3d() {
    r_quat_.setIdentity();
    t_vec_.setZero();
  };

  Pose3d(const Eigen::Quaterniond& r_quat, const Eigen::Vector3d& t_vec)
      : r_quat_(r_quat), t_vec_(t_vec) {}

  Pose3d(const Eigen::Matrix3d& r_mat, const Eigen::Vector3d& t_vec)
      : r_quat_(r_mat), t_vec_(t_vec) {}

  Pose3d(const double* const r_quat, const double* const t_vec)
      : r_quat_(r_quat), t_vec_(t_vec) {}

  static Pose3d Identity() { return {}; }

  void setIdentity() {
    r_quat_.setIdentity();
    t_vec_.setZero();
  }

  Pose3d Inv() const {
    Eigen::Quaterniond r_inv = r_quat_.inverse();
    Eigen::Vector3d t_inv = r_inv * t_vec_;
    return {r_inv, -t_inv};
  }

  Pose3d operator*(const Pose3d& rhs) const {
    return Pose3d(r_quat_ * rhs.r_quat_, r_quat_ * rhs.t_vec_ + t_vec_);
  }

  Pose3d& operator*=(const Pose3d& rhs) {
    t_vec_ += r_quat_ * rhs.t_vec_;
    r_quat_ *= rhs.r_quat_;
    return *this;
  }

  Eigen::Vector3d Transform(const Eigen::Vector3d& point) const {
    return r_quat_ * point + t_vec_;
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const {
    return Transform(point);
  }

  Eigen::Vector3d Rotate(const Eigen::Vector3d& vec) const {
    return r_quat_ * vec;
  }

  // Spherical linear interpolation to `pose_to`
  Pose3d Interpolate(const Pose3d& pose_to, double t) const {
    Pose3d pose_dst = t > 0 ? pose_to : (*this) * pose_to.Inv() * (*this);
    t = t > 0 ? t : -t;
    int whole = std::floor(t);
    double frac = t - whole;
    Eigen::Quaterniond r_interp = r_quat_.slerp(frac, pose_dst.r_quat_);
    while (whole--) r_interp *= pose_dst.r_quat_;
    Eigen::Vector3d t_interp = (pose_dst.t_vec_ - t_vec_) * t + t_vec_;
    return {r_interp, t_interp};
  }

  std::string ToString() const {
    std::ostringstream oss;
    double w = r_quat_.w(), a = r_quat_.x(), b = r_quat_.y(), c = r_quat_.z();
    double x = t_vec_.x(), y = t_vec_.y(), z = t_vec_.z();
    oss << std::setprecision(3) << "[Pose3d] r_quat = (" << w << " " << a << " "
        << b << " " << c << "), p = (" << x << " " << y << " " << z << ")";
    return oss.str();
  }

  // const Eigen::Quaterniond& r_quat() const { return r_quat_; }

  Eigen::Quaterniond r_quat() const { return r_quat_; }

  // const Eigen::Vector3d& t_vec() const { return t_vec_; }

  Eigen::Vector3d t_vec() const { return t_vec_; }

 protected:
  Eigen::Quaterniond r_quat_;  // orientation/rotation
  Eigen::Vector3d t_vec_;      // positon/translation
};

inline Pose3d Interpolate(const Pose3d& pose_from, const Pose3d& pose_to,
                          double t) {
  return pose_from.Interpolate(pose_to, t);
}

}  // namespace common
