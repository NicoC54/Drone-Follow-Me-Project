#include <gtest/gtest.h>

#include "follow_me_core/attitude_control.hpp"

using namespace follow_me;

namespace {
QuadrotorParams calmAir() {
  QuadrotorParams p;
  p.wind.setZero();
  p.gust_sigma = 0.0;
  return p;
}

double tiltDeg(const Quat& q) {
  return rad2deg(std::acos(std::clamp((q.toRotationMatrix() * Vec3::UnitZ()).z(), -1.0, 1.0)));
}

// Closed loop: attitude controller at 500 Hz on the true state.
struct Rig {
  QuadrotorParams qp = calmAir();
  AttitudeControlParams cp;
  Quadrotor quad{qp, Vec3(0, 0, 10), 0.0, 1};
  MulticopterController ctrl{cp, qp};
  MulticopterController::Output out{};
  void run(const Vec3& a_des, double yaw_des, double duration) {
    const double dt = 0.002;
    for (int k = 0; k < static_cast<int>(duration / dt); ++k) {
      out = ctrl.update(a_des, yaw_des, quad.state().q, quad.state().w, dt);
      quad.step(out.motor_cmd, dt);
    }
  }
};
}  // namespace

TEST(Quadrotor, AllocationIsInvertibleAndHoverIsBalanced) {
  const QuadrotorParams p;
  EXPECT_GT(std::abs(allocationMatrix(p).determinant()), 1e-9);
  Quadrotor quad(p, Vec3::Zero(), 0.0, 1);
  const RotorWrench rw = rotorWrench(p, Vec4::Constant(quad.hoverMotorSpeed()));
  EXPECT_NEAR(rw.thrust, p.mass * p.g, 1e-9);
  EXPECT_LT(rw.torque.norm(), 1e-12);
}

TEST(Quadrotor, FallsFreelyWithMotorsOff) {
  Quadrotor quad(calmAir(), Vec3(0, 0, 100), 0.0, 1);
  for (int k = 0; k < 500; ++k) quad.step(Vec4::Zero(), 0.002);  // 1 s, motors spin down
  EXPECT_LT(quad.state().v.z(), -7.0);                             // close to -g * 1 s
  EXPECT_LT(quad.state().w.norm(), 1e-6);                          // no torque: no rotation
}

TEST(Mixer, ReproducesFeasibleRequestsAndPrioritisesRollPitch) {
  const QuadrotorParams qp;
  MulticopterController ctrl(AttitudeControlParams{}, qp);
  bool sat = true;
  const Vec4 f = ctrl.mix(15.0, Vec3(0.2, -0.1, 0.02), &sat);
  EXPECT_FALSE(sat);
  const Vec4 u = allocationMatrix(qp) * f;
  EXPECT_TRUE(u.isApprox(Vec4(15.0, 0.2, -0.1, 0.02), 1e-9));
  // Impossible yaw request at hover: yaw is reduced to what fits, roll torque is kept.
  const Vec4 f2 = ctrl.mix(15.0, Vec3(0.3, 0.0, 5.0), &sat);
  EXPECT_TRUE(sat);
  const Vec4 u2 = allocationMatrix(qp) * f2;
  EXPECT_NEAR(u2(1), 0.3, 1e-9);
  EXPECT_GE(f2.minCoeff(), 0.0);
}

TEST(AttitudeControl, HoversInPlace) {
  Rig r;
  r.run(Vec3::Zero(), 0.0, 5.0);
  EXPECT_LT((r.quad.state().p - Vec3(0, 0, 10)).norm(), 0.05);
  EXPECT_LT(tiltDeg(r.quad.state().q), 0.1);
}

TEST(AttitudeControl, TracksATiltRequestQuickly) {
  Rig r;
  r.run(Vec3(3.0, 0.0, 0.0), 0.0, 0.4);  // 3 m/s^2 forward -> atan(3/9.81) = 17 deg pitch
  EXPECT_NEAR(tiltDeg(r.quad.state().q), rad2deg(std::atan(3.0 / 9.81)), 1.0);
  r.run(Vec3(3.0, 0.0, 0.0), 0.0, 1.0);
  EXPECT_NEAR(r.quad.state().v.z(), 0.0, 0.1);  // thrust compensates the tilt: altitude held
}

TEST(AttitudeControl, RespectsTiltLimitAndTurnsToHeading) {
  Rig r;
  r.run(Vec3(20.0, 0.0, 0.0), 1.5, 2.0);  // absurd acceleration + 86 deg heading change
  EXPECT_LT(tiltDeg(r.quad.state().q), r.cp.max_tilt_deg + 1.0);
  const Vec3 x_body = r.quad.state().q.toRotationMatrix().col(0);
  EXPECT_NEAR(std::atan2(x_body.y(), x_body.x()), 1.5, 0.05);
}
