#include <gtest/gtest.h>

#include "follow_me_ros/ros_utils.hpp"

using namespace follow_me_ros;

// With a level body, the Gazebo gimbal chain must give the same rotation as the core model.
TEST(CameraRotation, LevelBodyMatchesCoreModel) {
  OdomState o;
  o.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Vec3::UnitZ()));
  o.yaw = 0.4;
  GimbalState g;
  g.yaw = -1.1;
  g.pitch = 0.25;
  g.valid = true;
  EXPECT_TRUE(cameraRotation(o, g).isApprox(follow_me::rotWorldToCam(0.4 - 1.1, 0.25), 1e-12));
}

// A rolled body must tilt the camera (this is what the yaw-only model missed).
TEST(CameraRotation, BodyRollIsTakenIntoAccount) {
  OdomState o;
  o.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Vec3::UnitX()));
  GimbalState g;
  const Mat3 R = cameraRotation(o, g);
  EXPECT_TRUE((R * R.transpose()).isApprox(Mat3::Identity(), 1e-12));
  EXPECT_FALSE(R.isApprox(follow_me::rotWorldToCam(0.0, 0.0), 1e-6));
}
