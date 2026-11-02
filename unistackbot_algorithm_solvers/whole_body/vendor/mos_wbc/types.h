/**
 * @ Author: David
 * @ Create Time: 2026-06-11 22:31:57
 * @ Modified by: Your name
 * @ Modified time: 2026-06-14 22:23:09
 * @ Description:
 */

#pragma once

#include <Eigen/Dense>

namespace core {

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecX = Eigen::VectorXd;
using Mat3 = Eigen::Matrix3d;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Quat = Eigen::Quaterniond;
using SE3 = Eigen::Isometry3d;

struct ControlResult {
  VecX q;
  VecX dq;
  VecX ddq;
  VecX gravity;
};

}  // namespace core
