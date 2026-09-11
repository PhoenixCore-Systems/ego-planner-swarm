#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Clock::SharedPtr node_clock;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;
bool receive_scheduled_traj_ = false;
vector<UniformBspline> scheduled_traj_;
double scheduled_traj_duration_;
rclcpp::Time scheduled_start_time_;
int scheduled_traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
std::string output_frame_ = "world";
double max_start_past_sec_ = 1.0;
double max_start_future_sec_ = 2.0;
double scheduled_start_threshold_sec_ = 0.20;

bool validateBsplineMessage(
  const traj_utils::msg::Bspline & msg,
  const rclcpp::Time & now,
  std::string & reason)
{
  constexpr int kSupportedOrder = 3;
  constexpr double kEpsilon = 1e-9;
  if (msg.order != kSupportedOrder)
  {
    reason = "only cubic (order=3) B-splines are supported";
    return false;
  }
  if (msg.pos_pts.size() < static_cast<std::size_t>(msg.order + 1))
  {
    reason = "too few control points";
    return false;
  }
  if (msg.knots.size() != msg.pos_pts.size() + static_cast<std::size_t>(msg.order) + 1U)
  {
    reason = "knot/control-point count mismatch";
    return false;
  }
  for (const auto & point : msg.pos_pts)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      reason = "non-finite control point";
      return false;
    }
  }
  for (std::size_t i = 0; i < msg.knots.size(); ++i)
  {
    if (!std::isfinite(msg.knots[i]))
    {
      reason = "non-finite knot";
      return false;
    }
    if (i > 0 && msg.knots[i] < msg.knots[i - 1])
    {
      reason = "knots are not nondecreasing";
      return false;
    }
  }

  const std::size_t order = static_cast<std::size_t>(msg.order);
  const double domain_start = msg.knots[order];
  const double domain_end = msg.knots[msg.knots.size() - order - 1U];
  if (!(domain_end - domain_start > kEpsilon))
  {
    reason = "trajectory duration is not positive";
    return false;
  }

  if (msg.start_time.sec == 0 && msg.start_time.nanosec == 0U)
  {
    reason = "trajectory start_time is zero";
    return false;
  }
  const rclcpp::Time start_time(msg.start_time, now.get_clock_type());
  const double start_offset_sec = (start_time - now).seconds();
  if (start_offset_sec < -max_start_past_sec_)
  {
    reason = "trajectory start_time is implausibly old";
    return false;
  }
  if (start_offset_sec > max_start_future_sec_)
  {
    reason = "trajectory start_time is implausibly far in the future";
    return false;
  }

  // UniformBspline differentiates twice in this server.  Reject knot vectors
  // that would create a zero derivative denominator before constructing it.
  for (std::size_t i = 0; i + 1U < msg.pos_pts.size(); ++i)
  {
    if (!(msg.knots[i + order + 1U] - msg.knots[i + 1U] > kEpsilon))
    {
      reason = "velocity derivative has a zero knot span";
      return false;
    }
  }
  for (std::size_t i = 0; i + 2U < msg.pos_pts.size(); ++i)
  {
    if (!(msg.knots[i + order + 1U] - msg.knots[i + 2U] > kEpsilon))
    {
      reason = "acceleration derivative has a zero knot span";
      return false;
    }
  }
  return true;
}

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  std::string invalid_reason;
  if (!node_clock || !validateBsplineMessage(*msg, node_clock->now(), invalid_reason))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("traj_server"),
      "Rejected invalid B-spline trajectory: %s", invalid_reason.c_str());
    return;
  }

  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  vector<UniformBspline> parsed;
  parsed.push_back(pos_traj);
  parsed.push_back(parsed[0].getDerivative());
  parsed.push_back(parsed[1].getDerivative());
  const rclcpp::Time requested_start(msg->start_time, node_clock->get_clock_type());
  const double start_offset = (requested_start - node_clock->now()).seconds();
  if (receive_traj_ && start_offset > scheduled_start_threshold_sec_)
  {
    scheduled_traj_ = std::move(parsed);
    scheduled_traj_duration_ = scheduled_traj_[0].getTimeSum();
    scheduled_start_time_ = requested_start;
    scheduled_traj_id_ = msg->traj_id;
    receive_scheduled_traj_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("traj_server"),
      "Scheduled trajectory %d in %.3f s while trajectory %d remains active",
      scheduled_traj_id_, start_offset, traj_id_);
    return;
  }

  // Initial, stop, and safety-recovery trajectories take authority now (or at
  // their small publication offset) and cancel any previously queued future
  // continuation.
  traj_ = std::move(parsed);
  traj_duration_ = traj_[0].getTimeSum();
  start_time_ = requested_start;
  traj_id_ = msg->traj_id;
  receive_traj_ = true;
  receive_scheduled_traj_ = false;
  scheduled_traj_.clear();
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  const double yaw_dt = std::max(1e-3, (time_now - time_last).seconds());
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * yaw_dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / yaw_dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / yaw_dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / yaw_dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  // 统一时间源
  if (!node_clock)
    return;
  rclcpp::Time time_now = node_clock->now();
  if (receive_scheduled_traj_ && time_now >= scheduled_start_time_)
  {
    traj_ = std::move(scheduled_traj_);
    traj_duration_ = scheduled_traj_duration_;
    start_time_ = scheduled_start_time_;
    traj_id_ = scheduled_traj_id_;
    receive_scheduled_traj_ = false;
    scheduled_traj_.clear();
  }
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  double t_cur = (time_now - start_time_).seconds();

  // A newly accepted trajectory may intentionally start a few milliseconds in
  // the future.  Do not publish the zero-initialized command in that interval:
  // the downstream mux will retain the previous valid command (or HOLD when
  // none exists) until this trajectory becomes active.
  if (t_cur < 0.0)
    return;

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now;
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = output_frame_;
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  node_clock = node->get_clock();

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);
  node->declare_parameter("traj_server/output_frame", "world");
  node->get_parameter("traj_server/output_frame", output_frame_);
  node->declare_parameter("traj_server/max_start_past_sec", 1.0);
  node->get_parameter("traj_server/max_start_past_sec", max_start_past_sec_);
  node->declare_parameter("traj_server/max_start_future_sec", 2.0);
  node->get_parameter("traj_server/max_start_future_sec", max_start_future_sec_);
  node->declare_parameter("traj_server/scheduled_start_threshold_sec", 0.20);
  node->get_parameter(
    "traj_server/scheduled_start_threshold_sec", scheduled_start_threshold_sec_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
