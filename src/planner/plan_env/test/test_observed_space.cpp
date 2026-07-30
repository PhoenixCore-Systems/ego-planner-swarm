#include <gtest/gtest.h>

#include <Eigen/Core>

#include <chrono>
#include <cmath>
#include <vector>

#include "plan_env/observed_space.h"

namespace
{

ObservedSpaceGrid makeGrid(
  int ray_dilation_voxels = 0,
  double seed_radius = 0.15,
  double retention_sec = 0.5)
{
  ObservedSpaceGrid grid;
  grid.configure(
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3i(20, 20, 20),
    0.1,
    retention_sec,
    ray_dilation_voxels,
    seed_radius);
  return grid;
}

TEST(ObservedSpaceGrid, RayMarksOnlyCellsBeforeMeasuredHit)
{
  auto grid = makeGrid();
  const auto stats = grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);

  EXPECT_EQ(stats.accepted_ray_count, 1u);
  EXPECT_TRUE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));
  EXPECT_TRUE(grid.isObserved(Eigen::Vector3d(0.7, 0.0, 0.0)));
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.9, 0.0, 0.0)));
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.4, 0.5, 0.0)));
}

TEST(ObservedSpaceGrid, ObservationHistoryExpires)
{
  auto grid = makeGrid();
  grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);
  ASSERT_TRUE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));

  grid.expire(1.6);
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));
  EXPECT_EQ(grid.observedVoxelCount(), 0u);
}

TEST(ObservedSpaceGrid, RayRasterToleranceIsSeparateAndBounded)
{
  auto no_dilation = makeGrid(0);
  auto one_voxel = makeGrid(1);
  const std::vector<Eigen::Vector3d> endpoints{
    Eigen::Vector3d(0.8, 0.0, 0.0)};
  no_dilation.addObservation(
    Eigen::Vector3d::Zero(), endpoints, 1.0, 0.1, 1.0);
  one_voxel.addObservation(
    Eigen::Vector3d::Zero(), endpoints, 1.0, 0.1, 1.0);

  EXPECT_FALSE(no_dilation.isObserved(Eigen::Vector3d(0.4, 0.1, 0.0)));
  EXPECT_TRUE(one_voxel.isObserved(Eigen::Vector3d(0.4, 0.1, 0.0)));
  EXPECT_FALSE(one_voxel.isObserved(Eigen::Vector3d(0.4, 0.3, 0.0)));
  EXPECT_FALSE(one_voxel.isObserved(Eigen::Vector3d(0.8, 0.0, 0.0)));
}

TEST(ObservedSpaceGrid, PhysicalClearanceRequiresObservedSweptVolume)
{
  auto grid = makeGrid(0, 0.25);
  grid.addObservation(
    Eigen::Vector3d::Zero(), {}, 1.0, 0.1, 1.0);

  EXPECT_TRUE(grid.isObservedWithClearance(
      Eigen::Vector3d::Zero(), 0.1));
  EXPECT_FALSE(grid.isObservedWithClearance(
      Eigen::Vector3d::Zero(), 0.3));
}

TEST(ObservedSpaceGrid, SegmentClampsInsideUnknownFrontier)
{
  auto grid = makeGrid();
  grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);

  Eigen::Vector3d target;
  bool clamped = false;
  ASSERT_TRUE(grid.clampSegment(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.0,
      0.2,
      target,
      clamped));
  EXPECT_TRUE(clamped);
  EXPECT_GE(target.x(), 0.4);
  EXPECT_LT(target.x(), 0.7);
  EXPECT_NEAR(target.y(), 0.0, 1e-9);
  EXPECT_NEAR(target.z(), 0.0, 1e-9);
}

TEST(ObservedSpaceGrid, RetainedFrameLimitCannotUnderflowSharedVoxel)
{
  auto grid = makeGrid(0, 0.15, 1.0);
  for (int frame = 0; frame < 300; ++frame) {
    grid.addObservation(
      Eigen::Vector3d::Zero(),
      {Eigen::Vector3d(0.8, 0.0, 0.0)},
      static_cast<double>(frame) * 0.001,
      0.1,
      1.0);
  }

  grid.expire(1.256);
  EXPECT_TRUE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));

  grid.expire(2.0);
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));
  EXPECT_FALSE(grid.hasObservation());
}

TEST(ObservedSpaceGrid, VisualizationTracksOnlyRetainedActiveCells)
{
  auto grid = makeGrid();
  grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);

  std::vector<Eigen::Vector3d> points;
  grid.observedPoints(1, points);
  EXPECT_EQ(points.size(), grid.observedVoxelCount());

  grid.expire(1.6);
  grid.observedPoints(1, points);
  EXPECT_TRUE(points.empty());
}

TEST(ObservedSpaceGrid, DegenerateSegmentStillFailsClosedInUnknown)
{
  auto grid = makeGrid();
  Eigen::Vector3d target;
  bool clamped = false;
  EXPECT_FALSE(grid.clampSegment(
      Eigen::Vector3d(0.7, 0.7, 0.7),
      Eigen::Vector3d(0.7, 0.7, 0.7),
      0.0,
      0.0,
      target,
      clamped));
  EXPECT_TRUE(clamped);
}

TEST(ObservedSpaceGrid, ClockRegressionDiscardsPreviousHistory)
{
  auto grid = makeGrid();
  grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    2.0,
    0.1,
    1.0);
  ASSERT_TRUE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));

  grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.0, 0.8, 0.0)},
    1.0,
    0.1,
    1.0);
  EXPECT_FALSE(grid.isObserved(Eigen::Vector3d(0.4, 0.0, 0.0)));
  EXPECT_TRUE(grid.isObserved(Eigen::Vector3d(0.0, 0.4, 0.0)));
}

TEST(ObservedSpaceGrid, DenseLidarFrameIsUniqueBoundedAndTimely)
{
  ObservedSpaceGrid grid;
  grid.configure(
    // Match the 40 x 150 x 5 m live cave map at 0.1 m resolution.
    Eigen::Vector3d(-20.0, -75.0, -0.01),
    Eigen::Vector3i(400, 1500, 50),
    0.1,
    0.5,
    1,
    0.2);

  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Vector3d sensor_origin(0.0, 0.0, 2.0);
  std::vector<Eigen::Vector3d> endpoints;
  endpoints.reserve(360 * 21);
  for (int elevation_step = -10; elevation_step <= 10;
    ++elevation_step)
  {
    const double elevation =
      static_cast<double>(elevation_step) * kPi / 180.0;
    const double planar_scale = 8.5 * std::cos(elevation);
    for (int azimuth_step = 0; azimuth_step < 360;
      ++azimuth_step)
    {
      const double azimuth =
        static_cast<double>(azimuth_step) * kPi / 180.0;
      endpoints.emplace_back(
        sensor_origin.x() + planar_scale * std::cos(azimuth),
        sensor_origin.y() + planar_scale * std::sin(azimuth),
        sensor_origin.z() + 8.5 * std::sin(elevation));
    }
  }

  const auto start = std::chrono::steady_clock::now();
  const auto stats = grid.addObservation(
    sensor_origin, endpoints, 1.0, 0.1, 8.5);
  const double elapsed_sec =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();

  EXPECT_EQ(stats.accepted_ray_count, endpoints.size());
  // A first frame increments every retained voxel exactly once.
  EXPECT_EQ(stats.frame_voxel_count, stats.retained_voxel_count);
  EXPECT_EQ(stats.frame_voxel_count, grid.observedVoxelCount());
  EXPECT_GT(stats.frame_voxel_count, 100000u);

  // Every measured terminal remains unknown, including terminals which fall
  // inside another ray's one-voxel raster dilation.
  for (const auto & endpoint : endpoints) {
    EXPECT_FALSE(grid.isObserved(endpoint));
  }

  // This intentionally loose release-build guard catches a return to the
  // multi-million-entry duplicate/sort path without depending on a tight CI
  // scheduling budget. Operational latency is reported from the focused run.
  EXPECT_LT(elapsed_sec, 2.0);
}

TEST(ObservedSpaceGrid, OutOfMapSensorOriginIsReportedAsRejected)
{
  auto grid = makeGrid();
  const auto stats = grid.addObservation(
    Eigen::Vector3d(50.0, 0.0, 0.0),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);

  // An origin outside the grid discards the whole frame before ray tracing.
  // The counters alone look identical to a contribution-free accepted update,
  // so the flag is what makes the rejection diagnosable in flight logs.
  EXPECT_FALSE(stats.sensor_origin_in_map);
  EXPECT_EQ(stats.endpoint_count, 1u);
  EXPECT_EQ(stats.accepted_ray_count, 0u);
  EXPECT_EQ(stats.frame_voxel_count, 0u);
  EXPECT_FALSE(grid.hasObservation());
}

TEST(ObservedSpaceGrid, AcceptedUpdateReportsOriginInMap)
{
  auto grid = makeGrid();
  const auto stats = grid.addObservation(
    Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.8, 0.0, 0.0)},
    1.0,
    0.1,
    1.0);

  EXPECT_TRUE(stats.sensor_origin_in_map);
  EXPECT_EQ(stats.accepted_ray_count, 1u);
}

}  // namespace
