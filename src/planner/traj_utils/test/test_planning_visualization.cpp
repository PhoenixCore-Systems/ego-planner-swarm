#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <traj_utils/planning_visualization.h>

using namespace std::chrono_literals;

namespace
{

class RclcppGuard
{
public:
  RclcppGuard()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  ~RclcppGuard()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

TEST(PlanningVisualization, UsesConfiguredFrameAndNodeRosClock)
{
  RclcppGuard guard;

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("visualization/frame_id", "map"),
    });
  auto planner_node =
    std::make_shared<rclcpp::Node>("planning_visualization_test_node", options);
  auto io_node = std::make_shared<rclcpp::Node>("planning_visualization_test_io");
  ego_planner::PlanningVisualization visualization(planner_node);

  visualization_msgs::msg::Marker::SharedPtr received_goal;
  visualization_msgs::msg::Marker::SharedPtr received_optimal;
  auto goal_sub = io_node->create_subscription<visualization_msgs::msg::Marker>(
    "goal_point", 10,
    [&received_goal](visualization_msgs::msg::Marker::SharedPtr msg) {
      received_goal = std::move(msg);
    });
  auto optimal_sub = io_node->create_subscription<visualization_msgs::msg::Marker>(
    "optimal_list", 10,
    [&received_optimal](visualization_msgs::msg::Marker::SharedPtr msg) {
      received_optimal = std::move(msg);
    });
  auto clock_pub = io_node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(io_node);

  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = 42;
  clock.clock.nanosec = 125000000;
  constexpr int64_t expected_stamp_ns = 42125000000LL;

  const auto clock_deadline = std::chrono::steady_clock::now() + 3s;
  while (
    planner_node->now().nanoseconds() != expected_stamp_ns &&
    std::chrono::steady_clock::now() < clock_deadline)
  {
    clock_pub->publish(clock);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(planner_node->now().nanoseconds(), expected_stamp_ns);

  Eigen::MatrixXd optimal_points(3, 2);
  optimal_points <<
    0.0, 1.0,
    0.0, 2.0,
    1.0, 1.0;

  const auto marker_deadline = std::chrono::steady_clock::now() + 3s;
  while (
    (!received_goal || !received_optimal) &&
    std::chrono::steady_clock::now() < marker_deadline)
  {
    clock_pub->publish(clock);
    visualization.displayGoalPoint(
      Eigen::Vector3d::Zero(), Eigen::Vector4d(1.0, 1.0, 1.0, 1.0), 0.2, 1);
    visualization.displayOptimalList(optimal_points, 2);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_NE(received_goal, nullptr);
  ASSERT_NE(received_optimal, nullptr);
  EXPECT_EQ(received_goal->header.frame_id, "map");
  EXPECT_EQ(received_optimal->header.frame_id, "map");
  EXPECT_EQ(
    rclcpp::Time(received_goal->header.stamp).nanoseconds(),
    expected_stamp_ns);
  EXPECT_EQ(
    rclcpp::Time(received_optimal->header.stamp).nanoseconds(),
    expected_stamp_ns);
}

}  // namespace
