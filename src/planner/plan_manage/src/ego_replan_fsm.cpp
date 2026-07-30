
#include <ego_planner/ego_replan_fsm.h>
#include <cmath>
#include <stdexcept>

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = false;
    replan_from_measured_state_ = false;
    tracking_error_emergency_latched_ = false;
    resetTrackingErrorDebounce();
    resetStallDebounce();

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/max_consecutive_plan_failures", 8);
    node_->declare_parameter("fsm/plan_failure_retry_delay", 0.25);
    node_->declare_parameter("fsm/tracking_error_monitor_enabled", true);
    node_->declare_parameter("fsm/tracking_error_soft_threshold", 0.5);
    node_->declare_parameter("fsm/tracking_error_hard_threshold", 1.0);
    node_->declare_parameter("fsm/tracking_error_soft_duration", 0.25);
    node_->declare_parameter("fsm/tracking_error_hard_duration", 0.10);
    node_->declare_parameter("fsm/goal_tolerance", 0.3);
    node_->declare_parameter("fsm/goal_velocity_tolerance", 0.2);
    node_->declare_parameter("fsm/observed_space_target_margin", 0.0);
    node_->declare_parameter(
        "fsm/observed_space_target_search_half_angle_deg", 60.0);
    node_->declare_parameter("fsm/observed_space_target_search_steps", 3);
    node_->declare_parameter("fsm/stall_commanded_speed", 0.15);
    node_->declare_parameter("fsm/stall_measured_speed", 0.05);
    node_->declare_parameter("fsm/stall_duration", 2.0);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/max_consecutive_plan_failures", max_consecutive_plan_failures_);
    node_->get_parameter("fsm/plan_failure_retry_delay", plan_failure_retry_delay_);
    node_->get_parameter("fsm/tracking_error_monitor_enabled", tracking_error_monitor_enabled_);
    node_->get_parameter("fsm/tracking_error_soft_threshold", tracking_error_soft_threshold_);
    node_->get_parameter("fsm/tracking_error_hard_threshold", tracking_error_hard_threshold_);
    node_->get_parameter("fsm/tracking_error_soft_duration", tracking_error_soft_duration_);
    node_->get_parameter("fsm/tracking_error_hard_duration", tracking_error_hard_duration_);
    node_->get_parameter("fsm/goal_tolerance", goal_tolerance_);
    node_->get_parameter("fsm/goal_velocity_tolerance", goal_velocity_tolerance_);
    node_->get_parameter(
        "fsm/observed_space_target_margin",
        observed_space_target_margin_);
    node_->get_parameter(
        "fsm/observed_space_target_search_half_angle_deg",
        observed_space_target_search_half_angle_deg_);
    node_->get_parameter(
        "fsm/observed_space_target_search_steps",
        observed_space_target_search_steps_);
    node_->get_parameter("fsm/stall_commanded_speed", stall_commanded_speed_);
    node_->get_parameter("fsm/stall_measured_speed", stall_measured_speed_);
    node_->get_parameter("fsm/stall_duration", stall_duration_);

    if (tracking_error_monitor_enabled_ &&
        (!std::isfinite(stall_commanded_speed_) ||
         stall_commanded_speed_ <= 0.0 ||
         !std::isfinite(stall_measured_speed_) ||
         stall_measured_speed_ < 0.0 ||
         stall_measured_speed_ >= stall_commanded_speed_ ||
         !std::isfinite(stall_duration_) ||
         stall_duration_ <= 0.0))
    {
      throw std::invalid_argument(
          "Stall detection requires 0 <= measured < commanded speed thresholds and a positive duration.");
    }

    if (tracking_error_monitor_enabled_ &&
        (!std::isfinite(tracking_error_soft_threshold_) ||
         !std::isfinite(tracking_error_hard_threshold_) ||
         tracking_error_soft_threshold_ <= 0.0 ||
         tracking_error_hard_threshold_ <= tracking_error_soft_threshold_ ||
         !std::isfinite(tracking_error_soft_duration_) ||
         !std::isfinite(tracking_error_hard_duration_) ||
         tracking_error_soft_duration_ < 0.0 ||
         tracking_error_hard_duration_ < 0.0))
    {
      throw std::invalid_argument(
          "Tracking-error thresholds must satisfy 0 < soft < hard and debounce durations must be non-negative.");
    }
    if (!std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
        !std::isfinite(goal_velocity_tolerance_) || goal_velocity_tolerance_ < 0.0)
    {
      throw std::invalid_argument(
          "Goal tolerance must be positive and goal velocity tolerance must be non-negative.");
    }
    if (!std::isfinite(observed_space_target_margin_) ||
        observed_space_target_margin_ < 0.0)
    {
      throw std::invalid_argument(
          "fsm/observed_space_target_margin must be non-negative.");
    }
    if (!std::isfinite(observed_space_target_search_half_angle_deg_) ||
        observed_space_target_search_half_angle_deg_ < 0.0 ||
        observed_space_target_search_half_angle_deg_ > 90.0 ||
        observed_space_target_search_steps_ < 0)
    {
      throw std::invalid_argument(
          "fsm/observed_space_target_search_half_angle_deg must be in [0, 90] and "
          "fsm/observed_space_target_search_steps must be non-negative. Zero for "
          "either restricts the target search to the straight reference ray.");
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Tracking safety monitor: enabled=%s soft=%.2fm/%.2fs hard=%.2fm/%.2fs "
        "goal=%.2fm/%.2fmps stall=%.2f/%.2fmps for %.2fs target_search=+-%.0fdeg/%dsteps",
        tracking_error_monitor_enabled_ ? "true" : "false",
        tracking_error_soft_threshold_,
        tracking_error_soft_duration_,
        tracking_error_hard_threshold_,
        tracking_error_hard_duration_,
        goal_tolerance_,
        goal_velocity_tolerance_,
        stall_commanded_speed_,
        stall_measured_speed_,
        stall_duration_,
        observed_space_target_search_half_angle_deg_,
        observed_space_target_search_steps_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    cancel_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "planning/cancel",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          this->cancelPlanningCallback(request, response);
        });

    auto terrain_profile_qos = rclcpp::QoS(1);
    terrain_profile_qos.reliable();
    terrain_profile_sub_ = node_->create_subscription<p30_interfaces::msg::TerrainProfile>(
        "/terrain/profile",
        terrain_profile_qos,
        [this](const std::shared_ptr<const p30_interfaces::msg::TerrainProfile> &msg)
        {
          this->terrainProfileCallback(msg);
        });

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::terrainProfileCallback(
      const std::shared_ptr<const p30_interfaces::msg::TerrainProfile> &msg)
  {
    TerrainRefProfile profile;
    profile.stamp = rclcpp::Clock().now();
    profile.desired_agl = msg->desired_agl;
    profile.sample_spacing = msg->sample_spacing;
    profile.corridor_width = msg->corridor_width;

    const size_t n = std::min({
        msg->z_ref_points.size(),
        msg->ground_z.size(),
        msg->confidence.size(),
        msg->support_count.size()});
    profile.samples.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
      const auto &pt = msg->z_ref_points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
          !std::isfinite(msg->ground_z[i]) || !std::isfinite(msg->confidence[i]))
      {
        continue;
      }
      TerrainRefSample sample;
      sample.z_ref_point = Eigen::Vector3d(pt.x, pt.y, pt.z);
      sample.ground_z = msg->ground_z[i];
      sample.confidence = msg->confidence[i];
      sample.support_count = msg->support_count[i];
      profile.samples.push_back(sample);
    }
    profile.valid = profile.desired_agl > 0.0 && !profile.samples.empty();
    planner_manager_->setTerrainProfile(profile);
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      if (tracking_error_emergency_latched_)
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "A new goal was accepted; clearing the tracking-error emergency-stop latch.");
        tracking_error_emergency_latched_ = false;
      }
      replan_from_measured_state_ = false;
      resetTrackingErrorDebounce();
      resetStallDebounce();

      end_pt_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      consecutive_plan_failures_ = 0;

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "Received a waypoint while the planner is busy; replacing the active target and restarting from the new global trajectory.");
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(
        msg->pose.position.x,
        msg->pose.position.y,
        std::max(0.0, static_cast<double>(msg->pose.position.z)));

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::cancelPlanningCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    have_target_ = false;
    have_new_target_ = false;
    replan_from_measured_state_ = false;
    tracking_error_emergency_latched_ = false;
    consecutive_plan_failures_ = 0;
    flag_escape_emergency_ = false;
    resetTrackingErrorDebounce();
    resetStallDebounce();

    if (!have_odom_)
    {
      changeFSMExecState(WAIT_TARGET, "CANCEL");
      response->success = false;
      response->message =
          "Planner target cleared, but no odometry was available for a stop spline.";
      RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
      return;
    }

    const bool stop_published = callEmergencyStop(odom_pos_);
    changeFSMExecState(WAIT_TARGET, "CANCEL");

    response->success = stop_published;
    response->message = stop_published
                            ? "Planner target cleared and current-odometry stop spline published."
                            : "Planner target cleared, but stop spline publication failed.";
    if (stop_published)
    {
      RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
    }
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (tracking_error_emergency_latched_ &&
        exec_state_ == EMERGENCY_STOP &&
        new_state != EMERGENCY_STOP)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          2000,
          "Tracking-error emergency stop remains latched; a new accepted goal is required before leaving HOLD.");
      return;
    }

    if (new_state == EMERGENCY_STOP && exec_state_ != EMERGENCY_STOP)
    {
      // Every entry into EMERGENCY_STOP must publish exactly one stop spline.
      flag_escape_emergency_ = true;
    }

    if (new_state == exec_state_)
      continously_called_times_++;
    else
    {
      continously_called_times_ = 1;
      resetTrackingErrorDebounce();
    }

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::resetTrackingErrorDebounce()
  {
    soft_tracking_error_active_ = false;
    hard_tracking_error_active_ = false;
    soft_tracking_error_since_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    hard_tracking_error_since_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  }

  void EGOReplanFSM::resetStallDebounce()
  {
    stall_active_ = false;
    stall_since_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        consecutive_plan_failures_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        consecutive_plan_failures_++;
        last_plan_failure_time_ = rclcpp::Clock().now();
        if (consecutive_plan_failures_ >= max_consecutive_plan_failures_)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "Planner failed %d consecutive new-trajectory attempts; entering emergency stop until a safer target arrives.",
              consecutive_plan_failures_);
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (
          consecutive_plan_failures_ >= max_consecutive_plan_failures_ &&
          (rclcpp::Clock().now() - last_plan_failure_time_).seconds() < plan_failure_retry_delay_)
      {
        changeFSMExecState(EMERGENCY_STOP, "FSM");
        break;
      }

      const bool planned_from_measured_state = replan_from_measured_state_;
      const bool replan_success = planned_from_measured_state
                                      ? planFromMeasuredState(1)
                                      : planFromCurrentTraj(1);
      if (replan_success)
      {
        consecutive_plan_failures_ = 0;
        replan_from_measured_state_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        consecutive_plan_failures_++;
        last_plan_failure_time_ = rclcpp::Clock().now();
        if (consecutive_plan_failures_ >= max_consecutive_plan_failures_)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "Planner failed %d consecutive replans; entering emergency stop until a safer target arrives.",
              consecutive_plan_failures_);
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::clamp(t_cur, 0.0, info->duration_);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      const double tracking_error = (odom_pos_ - pos).norm();

      if (tracking_error_monitor_enabled_)
      {
        const rclcpp::Time debounce_now = node_->now();

        // Stall check first. A vehicle that is commanded to move but does not
        // move is the props-off / wedged case, and the soft threshold below
        // hides it: soft trips at roughly 0.7 m of error and then replans from
        // measured odometry, which resets the error before the hard threshold
        // is ever reached. This timer is therefore kept out of
        // resetTrackingErrorDebounce() and cleared only by real motion, a new
        // accepted goal, or a cancel.
        const double commanded_speed =
            info->velocity_traj_.evaluateDeBoorT(t_cur).norm();
        const double measured_speed_now = odom_vel_.norm();
        if (commanded_speed >= stall_commanded_speed_ &&
            measured_speed_now <= stall_measured_speed_)
        {
          if (!stall_active_)
          {
            stall_active_ = true;
            stall_since_ = debounce_now;
          }

          const double stalled_for = (debounce_now - stall_since_).seconds();
          if (stalled_for >= stall_duration_)
          {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Commanded %.3fm/s but measured %.3fm/s for %.3fs; the vehicle is not "
                "following the trajectory. Publishing an odometry-position emergency stop "
                "and latching HOLD until a new goal.",
                commanded_speed,
                measured_speed_now,
                stalled_for);
            tracking_error_emergency_latched_ = true;
            replan_from_measured_state_ = false;
            resetStallDebounce();
            changeFSMExecState(EMERGENCY_STOP, "STALL");
            break;
          }
        }
        else
        {
          stall_active_ = false;
        }

        if (tracking_error >= tracking_error_hard_threshold_)
        {
          if (!hard_tracking_error_active_)
          {
            hard_tracking_error_active_ = true;
            hard_tracking_error_since_ = debounce_now;
          }

          if ((debounce_now - hard_tracking_error_since_).seconds() >=
              tracking_error_hard_duration_)
          {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Tracking error %.3fm exceeded the hard %.3fm threshold for %.3fs; "
                "publishing an odometry-position emergency stop and latching HOLD until a new goal.",
                tracking_error,
                tracking_error_hard_threshold_,
                (debounce_now - hard_tracking_error_since_).seconds());
            tracking_error_emergency_latched_ = true;
            replan_from_measured_state_ = false;
            changeFSMExecState(EMERGENCY_STOP, "TRACKING_HARD");
            break;
          }
        }
        else
        {
          hard_tracking_error_active_ = false;
        }

        if (tracking_error >= tracking_error_soft_threshold_)
        {
          if (!soft_tracking_error_active_)
          {
            soft_tracking_error_active_ = true;
            soft_tracking_error_since_ = debounce_now;
          }

          if ((debounce_now - soft_tracking_error_since_).seconds() >=
              tracking_error_soft_duration_)
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "Tracking error %.3fm exceeded the soft %.3fm threshold for %.3fs; "
                "requesting a replan from measured odometry.",
                tracking_error,
                tracking_error_soft_threshold_,
                (debounce_now - soft_tracking_error_since_).seconds());
            replan_from_measured_state_ = true;
            changeFSMExecState(REPLAN_TRAJ, "TRACKING_SOFT");
            break;
          }
        }
        else
        {
          soft_tracking_error_active_ = false;
        }
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          const double measured_goal_distance = (end_pt_ - odom_pos_).norm();
          const double measured_speed = odom_vel_.norm();
          if (measured_goal_distance <= goal_tolerance_ &&
              measured_speed <= goal_velocity_tolerance_)
          {
            have_target_ = false;
            have_trigger_ = false;

            if (target_type_ == TARGET_TYPE::PRESET_TARGET)
            {
              wp_id_ = 0;
              planNextWaypoint(wps_[wp_id_]);
            }

            changeFSMExecState(WAIT_TARGET, "FSM");
            goto force_return;
          }
          else if (measured_goal_distance > goal_tolerance_)
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "Nominal trajectory ended, but measured goal distance is %.3fm "
                "(tolerance %.3fm, speed %.3fm/s); replanning from odometry.",
                measured_goal_distance,
                goal_tolerance_,
                measured_speed);
            replan_from_measured_state_ = true;
            changeFSMExecState(REPLAN_TRAJ, "GOAL_TRACKING");
          }
          else
          {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "Inside goal position tolerance, waiting for measured speed %.3fm/s "
                "to fall below %.3fm/s before completion.",
                measured_speed,
                goal_velocity_tolerance_);
          }
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        const double retry_elapsed = (rclcpp::Clock().now() - last_plan_failure_time_).seconds();
        if (
            enable_fail_safe_ &&
            !tracking_error_emergency_latched_ &&
            (!planner_manager_->grid_map_->observedSpaceGateEnabled() ||
             planner_manager_->grid_map_->observedSpaceFresh()) &&
            odom_vel_.norm() < 0.1 &&
            (consecutive_plan_failures_ < max_consecutive_plan_failures_ || retry_elapsed > plan_failure_retry_delay_))
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::planFromMeasuredState(const int trial_times /*=1*/)
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    const bool flag_random_poly_init = timesOfConsecutiveStateCalls().first > 1;
    for (int i = 0; i < trial_times; ++i)
    {
      if (callReboundReplan(true, flag_random_poly_init || i > 0))
      {
        return true;
      }
    }
    return false;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == INIT ||
        exec_state_ == WAIT_TARGET ||
        exec_state_ == EMERGENCY_STOP ||
        !have_target_)
      return;

    if (map->observedSpaceGateEnabled() && !map->observedSpaceFresh())
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Visibility cloud is stale or unavailable (age=%.3fs); emergency stop.",
          map->observedSpaceAge());
      replan_from_measured_state_ = false;
      changeFSMExecState(EMERGENCY_STOP, "OBSERVED_SPACE");
      return;
    }

    if (info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();
    t_cur = std::clamp(t_cur, 0.0, info->duration_);

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = rclcpp::Clock().now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (!map->observedSpaceGateEnabled() &&
          t_cur < t_2_3 && t >= t_2_3) // Preserve upstream behavior when the opt-in gate is disabled.
        break;

      const Eigen::Vector3d trajectory_point =
          info->position_traj_.evaluateDeBoorT(t);
      const bool observed_space_violation =
          map->observedSpaceGateEnabled() &&
          !map->isObservedSpaceCovered(trajectory_point);
      bool occ =
          map->getInflateOccupancy(trajectory_point) != 0;

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {
        const bool replan_success = observed_space_violation
                                        ? planFromMeasuredState()
                                        : planFromCurrentTraj();
        if (replan_success)
        {
          consecutive_plan_failures_ = 0;
          replan_from_measured_state_ = false;
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (observed_space_violation)
            replan_from_measured_state_ = true;

          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "%s on the active trajectory; emergency stop in %.3fs.",
                observed_space_violation ? "Unknown or outside observed space" : "Suddenly discovered obstacle",
                t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    if (planner_manager_->grid_map_->observedSpaceGateEnabled() &&
        !planner_manager_->grid_map_->observedSpaceFresh())
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "Skipping replan because the visibility cloud is stale or unavailable.");
      return false;
    }

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    const double previous_progress_time =
        planner_manager_->global_data_.last_progress_time_;
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }

    if (planner_manager_->grid_map_->observedSpaceGateEnabled())
    {
      // The local target must stay inside recently observed space, otherwise the
      // trajectory that reaches it necessarily ends in unknown space and
      // EGOPlannerManager::validateFullObservedTrajectory() rejects every
      // candidate. The two checks are not redundant: validation only rejects,
      // the target is what makes a valid trajectory possible at all.
      //
      // Restricting the target to the straight start->reference ray, however,
      // makes the planner refuse to move at a bend: that ray points into rock,
      // so the reachable point lands inside reboundReplan()'s 0.2 m minimum
      // planning distance and every replan aborts with "Close to goal". Yawing
      // the reference direction and keeping whichever candidate reaches
      // furthest preserves the observed-space guarantee while still giving the
      // optimizer a usable subgoal when the goal direction itself is blocked.
      const double stopping_distance =
          start_vel_.squaredNorm() /
          (2.0 * std::max(1e-3, planner_manager_->pp_.max_acc_));
      const double effective_frontier_margin =
          std::max(observed_space_target_margin_, stopping_distance);

      Eigen::Vector3d best_reach = start_pt_;
      double best_progress = -std::numeric_limits<double>::infinity();
      double best_reach_distance = 0.0;
      double best_yaw_offset_deg = 0.0;
      bool best_was_clamped = true;
      bool start_is_observed = true;

      const Eigen::Vector3d reference_delta = local_target_pt_ - start_pt_;
      // Candidates are scored by how far they advance along the reference
      // direction, not by raw reach. Raw reach makes the choice near-arbitrary
      // whenever several directions see out to the full horizon, which jitters
      // the subgoal every replan, and it happily prefers a wide-open side
      // chamber over the goal direction. There is no global planner to undo
      // that wandering.
      const Eigen::Vector3d reference_dir =
          reference_delta.norm() > 1e-6
              ? reference_delta.normalized()
              : Eigen::Vector3d::Zero();
      const int search_steps =
          observed_space_target_search_half_angle_deg_ > 0.0
              ? observed_space_target_search_steps_
              : 0;
      const double step_deg =
          search_steps > 0
              ? observed_space_target_search_half_angle_deg_ /
                    static_cast<double>(search_steps)
              : 0.0;

      for (int step = 0; step <= search_steps; ++step)
      {
        // Straight ahead first, then alternating sides, so an equal reach keeps
        // the candidate closest to the goal direction.
        for (const int side : {1, -1})
        {
          if (step == 0 && side < 0)
            continue;

          const double yaw_offset_deg =
              static_cast<double>(side) * step_deg * static_cast<double>(step);
          const double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;
          const Eigen::Vector3d candidate_delta =
              Eigen::AngleAxisd(yaw_offset_rad, Eigen::Vector3d::UnitZ()) *
              reference_delta;
          const Eigen::Vector3d candidate_target = start_pt_ + candidate_delta;

          Eigen::Vector3d candidate_reach = start_pt_;
          bool candidate_clamped = true;
          start_is_observed =
              planner_manager_->grid_map_->clampToObservedSpace(
                  start_pt_,
                  candidate_target,
                  effective_frontier_margin,
                  candidate_reach,
                  candidate_clamped);
          if (!start_is_observed)
            break;

          const Eigen::Vector3d candidate_offset = candidate_reach - start_pt_;
          const double candidate_progress =
              candidate_offset.dot(reference_dir);
          if (candidate_progress > best_progress)
          {
            best_progress = candidate_progress;
            best_reach_distance = candidate_offset.norm();
            best_reach = candidate_reach;
            best_yaw_offset_deg = yaw_offset_deg;
            best_was_clamped = candidate_clamped;
          }
        }

        if (!start_is_observed)
          break;
      }

      // best_reach equals the reference target exactly when the straight-ahead
      // candidate reached it in full (0 deg is evaluated first, so it wins ties),
      // so assigning unconditionally is a no-op in the unobstructed case and
      // correctly adopts a rotated subgoal otherwise. Assigning only in the
      // clamped branch would leave the unreachable reference target in place
      // whenever a rotated candidate won, and every trajectory to it would then
      // be rejected by full-trajectory validation.
      local_target_pt_ = best_reach;

      if (!start_is_observed || best_was_clamped ||
          std::abs(best_yaw_offset_deg) > 1e-9)
      {
        planner_manager_->global_data_.last_progress_time_ =
            previous_progress_time;
        local_target_vel_.setZero();
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "Local target limited to observed space at [%.2f, %.2f, %.2f] with zero "
            "terminal velocity: %.0f deg off the reference direction reached %.2fm "
            "(frontier margin %.2fm, stopping distance %.2fm).",
            local_target_pt_.x(),
            local_target_pt_.y(),
            local_target_pt_.z(),
            best_yaw_offset_deg,
            std::max(0.0, best_reach_distance),
            effective_frontier_margin,
            stopping_distance);
      }
    }
  }

} // namespace ego_planner
