# Leader state estimated by the tracker (EKF), world frame ENU.
uint8 SEARCH=0
uint8 TRACK=1
uint8 COAST=2

std_msgs/Header header
uint8 status
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] covariance      # 6x6 row-major covariance of [px py pz vx vy vz]
float64 last_nis            # normalised innovation squared of the last measurement
