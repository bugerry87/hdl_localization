#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x,
 * acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;

public:
  PoseSystem() { dt = 0.01; }

  // system equation
  VectorXt f(const VectorXt &state, const VectorXt &control) const {
    VectorXt next_state(16);

    next_state.middleRows(0, 3) = control.middleRows(0, 3);
    Quaterniont qt_(control[3], control[4], control[5], control[6]);
    qt_.normalize();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    return next_state;
  }

  // observation equation
  VectorXt h(const VectorXt &state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
};

} // namespace hdl_localization

#endif // POSE_SYSTEM_HPP
