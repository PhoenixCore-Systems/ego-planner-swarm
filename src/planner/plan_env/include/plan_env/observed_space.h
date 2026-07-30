#ifndef PLAN_ENV__OBSERVED_SPACE_H_
#define PLAN_ENV__OBSERVED_SPACE_H_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_set>
#include <vector>

/**
 * A bounded-history voxel mask populated only by the free part of measured
 * sensor rays.
 *
 * This class deliberately does not contain obstacle state. GridMap overlays
 * its existing inflated obstacle buffer on top of this mask, so an endpoint
 * can never become free merely because it was used to trace a visibility ray.
 */
class ObservedSpaceGrid
{
public:
  struct UpdateStats
  {
    std::size_t endpoint_count{0};
    std::size_t accepted_ray_count{0};
    std::size_t frame_voxel_count{0};
    std::size_t retained_voxel_count{0};
    /// False when the sensor origin fell outside the configured grid, which
    /// discards the whole frame before any ray is traced. Without this flag the
    /// rejection is indistinguishable from an accepted update that happened to
    /// contribute nothing.
    bool sensor_origin_in_map{false};
  };

  void configure(
    const Eigen::Vector3d & origin,
    const Eigen::Vector3i & voxel_num,
    double resolution,
    double retention_sec,
    int ray_dilation_voxels,
    double seed_radius);

  void clear();
  void expire(double now_sec);

  UpdateStats addObservation(
    const Eigen::Vector3d & sensor_origin,
    const std::vector<Eigen::Vector3d> & hit_endpoints,
    double receive_time_sec,
    double min_ray_length,
    double max_ray_length);

  bool configured() const {return configured_;}
  bool hasObservation() const {return has_observation_;}
  double lastObservationTime() const {return last_observation_time_sec_;}
  std::size_t observedVoxelCount() const
  {
    return active_voxel_addresses_.size();
  }

  bool isObserved(const Eigen::Vector3d & position) const;
  bool isObservedWithClearance(
    const Eigen::Vector3d & position,
    double clearance_m) const;

  /**
   * Clamp a straight local reference to the last continuously observed sample.
   *
   * The caller remains responsible for setting zero terminal velocity when
   * `clamped` is true. Obstacle avoidance is intentionally not performed here.
   */
  bool clampSegment(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & desired,
    double clearance_m,
    double frontier_margin_m,
    Eigen::Vector3d & target,
    bool & clamped) const;

  void observedPoints(
    int stride_voxels,
    std::vector<Eigen::Vector3d> & points) const;

private:
  struct ObservationFrame
  {
    double receive_time_sec{0.0};
    std::vector<int> voxel_addresses;
  };

  bool positionToIndex(
    const Eigen::Vector3d & position,
    Eigen::Vector3i & index) const;
  bool indexInMap(const Eigen::Vector3i & index) const;
  int address(const Eigen::Vector3i & index) const;
  Eigen::Vector3d indexToPosition(const Eigen::Vector3i & index) const;
  Eigen::Vector3i addressToIndex(int voxel_address) const;
  bool markScratchAddress(
    std::vector<std::uint64_t> & scratch_words,
    int voxel_address) const;
  bool scratchContains(
    const std::vector<std::uint64_t> & scratch_words,
    int voxel_address) const;
  void clearScratchAddress(
    std::vector<std::uint64_t> & scratch_words,
    int voxel_address) const;
  void addDilatedVoxel(
    const Eigen::Vector3i & index,
    std::vector<int> & addresses);
  void removeOldestFrame();

  bool configured_{false};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3i voxel_num_{Eigen::Vector3i::Zero()};
  double resolution_{0.1};
  double resolution_inv_{10.0};
  double retention_sec_{0.5};
  int ray_dilation_voxels_{0};
  double seed_radius_{0.0};

  std::vector<std::uint8_t> observation_ref_count_;
  // Reusable, sparse-cleared frame workspaces. Keeping one bit per voxel for
  // free cells and one for terminals bounds scratch memory to V / 4 bytes and
  // prevents dense scans from accumulating duplicate ray/dilation entries.
  std::vector<std::uint64_t> frame_voxel_scratch_;
  std::vector<std::uint64_t> terminal_voxel_scratch_;
  std::unordered_set<int> active_voxel_addresses_;
  std::deque<ObservationFrame> frames_;
  bool has_observation_{false};
  double last_observation_time_sec_{0.0};
};

#endif  // PLAN_ENV__OBSERVED_SPACE_H_
