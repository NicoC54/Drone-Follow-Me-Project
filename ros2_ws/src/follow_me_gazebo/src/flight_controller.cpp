// FlightController: Gazebo system plugin = the follower's flight-controller "firmware".
//
// Runs INSIDE the physics loop, at every physics step (500 Hz), so it never lags behind the
// simulation whatever the CPU load (the ROS nodes were seen to stall for 100 ms under the GUI):
//
//   truth (Gazebo rigid body) -> simulated sensors (IMU 250 Hz, mag 50, baro 25, GNSS 10 Hz)
//   -> navigation ESKF (follow_me::NavEskf)
//   -> attitude + rate control + mixer (follow_me::MulticopterController), on the NAV estimate
//   -> motor dynamics + rotor wrench (follow_me::MotorModel, rotorWrench)
//   -> force + torque applied to the Gazebo body (Gazebo integrates the 6-DOF dynamics)
//
// Interface with the "companion computer" (ROS nodes), via gz-transport + ros_gz_bridge:
//   in : <setpoint_topic>  gz.msgs.Twist   linear = desired acceleration (world ENU, m/s^2),
//                                          angular.z = desired yaw (rad)
//   out: <nav_topic>       gz.msgs.OdometryWithCovariance   navigation estimate, 100 Hz
//        <status_topic>    gz.msgs.Float_V  [tilt_deg, thrust_N, saturated, w0, w1, w2, w3], 50 Hz
// Without setpoint for <setpoint_timeout> s, it holds its position (on its own navigation).
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>
#include <optional>
#include <random>

#include "follow_me_core/attitude_control.hpp"
#include "follow_me_core/nav_eskf.hpp"

namespace follow_me_gazebo {

using follow_me::Mat3;
using follow_me::Quat;
using follow_me::Vec3;

namespace {
Vec3 toEigen(const gz::math::Vector3d& v) { return Vec3(v.X(), v.Y(), v.Z()); }
gz::math::Vector3d toGz(const Vec3& v) { return gz::math::Vector3d(v.x(), v.y(), v.z()); }
template <typename T>
T sdfGet(const std::shared_ptr<const sdf::Element>& sdf, const std::string& key, const T& def) {
  return sdf->HasElement(key) ? sdf->Get<T>(key) : def;
}
}  // namespace

class FlightController : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate {
 public:
  FlightController()
      : motors_(quad_),
        ctrl_(follow_me::AttitudeControlParams{}, quad_),
        sensors_(sensor_prm_, 42),
        nav_(sensor_prm_, follow_me::NavFilterParams{}) {}

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    gz::sim::Model model(entity);
    const auto link_name = sdfGet<std::string>(sdf, "link_name", "base_link");
    link_ = gz::sim::Link(model.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      gzerr << "[FlightController] link '" << link_name << "' not found\n";
      return;
    }
    link_.EnableVelocityChecks(ecm, true);
    link_.EnableAccelerationChecks(ecm, true);
    setpoint_timeout_ = sdfGet<double>(sdf, "setpoint_timeout", 0.5);
    sensors_ = follow_me::NavSensors(sensor_prm_, sdfGet<int>(sdf, "seed", 42));

    const auto sp_topic = sdfGet<std::string>(sdf, "setpoint_topic", "/model/follower/fc/setpoint");
    node_.Subscribe(sp_topic, &FlightController::onSetpoint, this);
    nav_pub_ = node_.Advertise<gz::msgs::OdometryWithCovariance>(
        sdfGet<std::string>(sdf, "nav_topic", "/model/follower/nav/odometry"));
    status_pub_ = node_.Advertise<gz::msgs::Float_V>(
        sdfGet<std::string>(sdf, "status_topic", "/model/follower/fc/status"));
    gzmsg << "[FlightController] ready on link '" << link_name << "', setpoints on " << sp_topic
          << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || !link_.Valid(ecm)) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (dt <= 0.0) return;

    // ---------- truth from the physics engine ----------
    const auto pose = link_.WorldPose(ecm);
    const auto vel = link_.WorldLinearVelocity(ecm);
    const auto wvel = link_.WorldAngularVelocity(ecm);
    if (!pose || !vel || !wvel) return;  // available from the 2nd step
    const auto& r = pose->Rot();
    const Quat q_true(r.W(), r.X(), r.Y(), r.Z());
    const Mat3 R = q_true.toRotationMatrix();
    const Vec3 p_true = toEigen(pose->Pos()), v_true = toEigen(*vel);
    const Vec3 w_body = R.transpose() * toEigen(*wvel);
    const auto acc = link_.WorldLinearAcceleration(ecm);
    const Vec3 a_true = acc ? toEigen(*acc) : Vec3::Zero();
    const Vec3 f_body = R.transpose() * (a_true + Vec3(0, 0, quad_.g));  // specific force

    // ---------- navigation: simulated sensors -> ESKF ----------
    if (!nav_.initialised()) {
      nav_.init(sensors_.gnss(p_true, v_true).pos, sensors_.magYaw(q_true));
      motors_.reset(follow_me::Vec4::Constant(std::sqrt(quad_.mass * quad_.g / (4.0 * quad_.kf))));
      hold_p_ = nav_.position();
      hold_yaw_ = follow_me::yawOf(nav_.attitude());
      next_imu_ = next_mag_ = next_baro_ = next_gnss_ = next_nav_pub_ = next_status_ = t;
      last_imu_ = t;
    }
    if (t >= next_imu_ - 1e-9) {
      const double dti = t - last_imu_ > 0 ? t - last_imu_ : 1.0 / sensor_prm_.imu_rate;
      nav_.predict(sensors_.imu(f_body, w_body, dti), dti);
      last_imu_ = t;
      next_imu_ += 1.0 / sensor_prm_.imu_rate;
    }
    if (t >= next_mag_ - 1e-9) { nav_.updateMagYaw(sensors_.magYaw(q_true)); next_mag_ += 1.0 / sensor_prm_.mag_rate; }
    if (t >= next_baro_ - 1e-9) { nav_.updateBaro(sensors_.baro(p_true.z())); next_baro_ += 1.0 / sensor_prm_.baro_rate; }
    if (t >= next_gnss_ - 1e-9) { nav_.updateGnss(sensors_.gnss(p_true, v_true)); next_gnss_ += 1.0 / sensor_prm_.gnss_rate; }

    // ---------- setpoint from the companion computer (or position hold) ----------
    Vec3 a_des;
    double yaw_des;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_t_ = t;  // sim time used to stamp incoming setpoints
      if (sp_ && t - sp_->t_rx < setpoint_timeout_) {
        a_des = sp_->a;
        yaw_des = sp_->yaw;
        holding_ = false;
      } else {
        if (!holding_) {  // lost the companion computer: hold here
          hold_p_ = nav_.position();
          hold_yaw_ = follow_me::yawOf(nav_.attitude());
          holding_ = true;
          if (sp_) gzwarn << "[FlightController] no setpoint for " << setpoint_timeout_ << " s: position hold\n";
        }
        a_des = 1.0 * (hold_p_ - nav_.position()) - 2.0 * nav_.velocity();
        yaw_des = hold_yaw_;
      }
    }

    // ---------- attitude + rate control on the NAVIGATION estimate ----------
    const auto out = ctrl_.update(a_des, yaw_des, nav_.attitude(), nav_.rate(), dt);
    motors_.step(out.motor_cmd, dt);
    const follow_me::RotorWrench rw = follow_me::rotorWrench(quad_, motors_.speeds());

    // Aerodynamics (same model as the offline sim): drag on airspeed + correlated gusts.
    const Vec3 n(normal_(rng_), normal_(rng_), normal_(rng_));
    gust_ += -gust_ * dt / quad_.gust_tau + quad_.gust_sigma * std::sqrt(2.0 * dt / quad_.gust_tau) * n;
    const Vec3 force = R * Vec3(0, 0, rw.thrust) +
                       quad_.mass * (-quad_.drag * (v_true - quad_.wind) + gust_);
    link_.AddWorldWrench(ecm, toGz(force), toGz(R * rw.torque));

    // ---------- outputs ----------
    if (t >= next_nav_pub_ - 1e-9) {
      publishNav(info);
      next_nav_pub_ += 0.01;
    }
    if (t >= next_status_ - 1e-9) {
      gz::msgs::Float_V st;
      st.add_data(follow_me::rad2deg(std::acos(std::clamp(R(2, 2), -1.0, 1.0))));
      st.add_data(out.thrust);
      st.add_data(out.saturated ? 1.0f : 0.0f);
      for (int i = 0; i < 4; ++i) st.add_data(motors_.speeds()(i));
      status_pub_.Publish(st);
      next_status_ += 0.02;
    }
  }

 private:
  struct Setpoint {
    Vec3 a;
    double yaw;
    double t_rx;
  };

  void onSetpoint(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    sp_ = Setpoint{Vec3(msg.linear().x(), msg.linear().y(), msg.linear().z()), msg.angular().z(),
                   last_t_};
  }

  void publishNav(const gz::sim::UpdateInfo& info) {
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec);
    gz::msgs::OdometryWithCovariance m;
    m.mutable_header()->mutable_stamp()->set_sec(sec.count());
    m.mutable_header()->mutable_stamp()->set_nsec(static_cast<int>(nsec.count()));
    auto* fid = m.mutable_header()->add_data();
    fid->set_key("frame_id");
    fid->add_value("world");
    auto* cid = m.mutable_header()->add_data();
    cid->set_key("child_frame_id");
    cid->add_value("follower_nav");

    const Vec3& p = nav_.position();
    const Quat& q = nav_.attitude();
    const Mat3 R = q.toRotationMatrix();
    auto* pose = m.mutable_pose_with_covariance()->mutable_pose();
    pose->mutable_position()->set_x(p.x());
    pose->mutable_position()->set_y(p.y());
    pose->mutable_position()->set_z(p.z());
    pose->mutable_orientation()->set_w(q.w());
    pose->mutable_orientation()->set_x(q.x());
    pose->mutable_orientation()->set_y(q.y());
    pose->mutable_orientation()->set_z(q.z());
    // ROS convention: twist expressed in the child (body) frame.
    const Vec3 v_body = R.transpose() * nav_.velocity();
    const Vec3 w = nav_.rate();
    auto* tw = m.mutable_twist_with_covariance()->mutable_twist();
    tw->mutable_linear()->set_x(v_body.x());
    tw->mutable_linear()->set_y(v_body.y());
    tw->mutable_linear()->set_z(v_body.z());
    tw->mutable_angular()->set_x(w.x());
    tw->mutable_angular()->set_y(w.y());
    tw->mutable_angular()->set_z(w.z());

    // Covariances, 6x6 row-major: pose = [x y z rot_x rot_y rot_z], twist = [v_body w_body]
    const auto& P = nav_.P();
    auto* pc = m.mutable_pose_with_covariance()->mutable_covariance();
    auto* tc = m.mutable_twist_with_covariance()->mutable_covariance();
    const Mat3 Pv_body = R.transpose() * P.block<3, 3>(3, 3) * R;
    const double gyro_var = sensor_prm_.gyro_sigma * sensor_prm_.gyro_sigma;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) {
        double vp = 0.0, vt = 0.0;
        if (i < 3 && j < 3) {
          vp = P(i, j);
          vt = Pv_body(i, j);
        } else if (i >= 3 && j >= 3) {
          vp = P(3 + i, 3 + j);  // attitude error block (indices 6..8)
          vt = (i == j) ? gyro_var : 0.0;
        }
        pc->add_data(vp);
        tc->add_data(vt);
      }
    nav_pub_.Publish(m);
  }

  // Parameters (defaults of follow_me_core: the SDF model must match mass and inertia)
  follow_me::QuadrotorParams quad_;
  follow_me::NavSensorParams sensor_prm_;
  follow_me::MotorModel motors_;
  follow_me::MulticopterController ctrl_;
  follow_me::NavSensors sensors_;
  follow_me::NavEskf nav_;

  gz::sim::Link link_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher nav_pub_, status_pub_;
  std::mutex mutex_;
  std::optional<Setpoint> sp_;
  double setpoint_timeout_ = 0.5;
  double last_t_ = 0.0;  // sim time of the last PreUpdate (guarded by mutex_)
  bool holding_ = true;
  Vec3 hold_p_ = Vec3::Zero();
  double hold_yaw_ = 0.0;
  double next_imu_ = 0, last_imu_ = 0, next_mag_ = 0, next_baro_ = 0, next_gnss_ = 0;
  double next_nav_pub_ = 0, next_status_ = 0;
  Vec3 gust_ = Vec3::Zero();
  std::mt19937_64 rng_{7};
  std::normal_distribution<double> normal_{0.0, 1.0};

};

}  // namespace follow_me_gazebo

GZ_ADD_PLUGIN(follow_me_gazebo::FlightController, gz::sim::System,
              follow_me_gazebo::FlightController::ISystemConfigure,
              follow_me_gazebo::FlightController::ISystemPreUpdate)
