/**
 * @ Author: David
 * @ Create Time: 2026-06-11 22:49:40
 * @ Modified by: David
 * @ Modified time: 2026-07-13 21:03:51
 * @ Description:
 */

#pragma once

#include <memory>

#include "types.h"   // [vendored patch 2026-10-29] 原 "arm_controller/types.h" → 同目录相对引用

namespace core {

class WbcSolver {
 public:
  WbcSolver(const std::string& urdf_path, const std::string& config_path);
  ~WbcSolver();
  // 初始姿态
  bool setInitJoint(const std::vector<double>& q);
  // 外部规划器设置姿态、速度、加速度
  bool setNewJoint(const std::vector<double>& q, const std::vector<double>& dq, const std::vector<double>& ddq);
  // 将当前关节指令冻结为笛卡尔运动的零空间姿态参考
  bool setPostureReference();
  // 将配置的笛卡尔控制帧转换到 wrist_pitch_link 后检查肩部安全可达半径
  bool isCartesianTargetReachable(const SE3& left_hand_pose, const SE3& right_hand_pose) const;
  // 设置笛卡尔目标
  bool setCartesianTarget(const SE3& left_hand_pose, const SE3& right_hand_pose);
  // 直接获取关节控制指令
  ControlResult getControlResult();

  int32_t getJointNum();
  SE3 getLeftHandPose();
  SE3 getRightHandPose();
  // Vec3 getLeftElbowPos();
  // Vec3 getRightElbowPos();
  // SE3 getBodyCameraPose();
  SE3 getLeftCameraPose();
  SE3 getRightCameraPose();
  SE3 getHeadCameraPose();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace core
