#include "plan_env/observed_space.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "plan_env/raycast.h"

void ObservedSpaceGrid::configure(
  const Eigen::Vector3d & origin,
  const Eigen::Vector3i & voxel_num,
  double resolution,
  double retention_sec,
  int ray_dilation_voxels,
  double seed_radius)
{
  configured_ = false;
  observation_ref_count_.clear();
  frame_voxel_scratch_.clear();
  terminal_voxel_scratch_.clear();
  active_voxel_addresses_.clear();
  frames_.clear();
  has_observation_ = false;
  last_observation_time_sec_ = 0.0;

  if (!origin.allFinite() || voxel_num.minCoeff() <= 0 ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(retention_sec) || retention_sec < 0.0 ||
    !std::isfinite(seed_radius))
  {
    return;
  }

  const auto voxel_count_x = static_cast<std::size_t>(voxel_num.x());
  const auto voxel_count_y = static_cast<std::size_t>(voxel_num.y());
  const auto voxel_count_z = static_cast<std::size_t>(voxel_num.z());
  const auto max_size = std::numeric_limits<std::size_t>::max();
  if (voxel_count_x > max_size / voxel_count_y ||
    voxel_count_x * voxel_count_y > max_size / voxel_count_z)
  {
    return;
  }
  const auto voxel_count =
    voxel_count_x * voxel_count_y * voxel_count_z;
  if (voxel_count >
    static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    return;
  }

  origin_ = origin;
  voxel_num_ = voxel_num;
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution_;
  retention_sec_ = retention_sec;
  ray_dilation_voxels_ = std::clamp(ray_dilation_voxels, 0, 1);
  seed_radius_ = std::max(0.0, seed_radius);

  observation_ref_count_.assign(voxel_count, 0);
  constexpr std::size_t kScratchWordBits =
    std::numeric_limits<std::uint64_t>::digits;
  const std::size_t scratch_word_count =
    (voxel_count + kScratchWordBits - 1) / kScratchWordBits;
  frame_voxel_scratch_.assign(scratch_word_count, 0);
  terminal_voxel_scratch_.assign(scratch_word_count, 0);
  configured_ = true;
}

void ObservedSpaceGrid::clear()
{
  std::fill(
    observation_ref_count_.begin(),
    observation_ref_count_.end(), 0);
  std::fill(
    frame_voxel_scratch_.begin(),
    frame_voxel_scratch_.end(), 0);
  std::fill(
    terminal_voxel_scratch_.begin(),
    terminal_voxel_scratch_.end(), 0);
  active_voxel_addresses_.clear();
  frames_.clear();
  has_observation_ = false;
  last_observation_time_sec_ = 0.0;
}

void ObservedSpaceGrid::expire(double now_sec)
{
  if (!configured_ || !std::isfinite(now_sec)) {
    return;
  }

  // A ROS clock reset invalidates the age ordering of the retained frames.
  // Clearing is conservative and allows the next observation to seed a new
  // history in the reset time domain.
  if (!frames_.empty() &&
    now_sec + 1e-9 < frames_.back().receive_time_sec)
  {
    clear();
    return;
  }

  while (!frames_.empty() &&
    now_sec - frames_.front().receive_time_sec > retention_sec_)
  {
    removeOldestFrame();
  }
  has_observation_ = !frames_.empty();
}

ObservedSpaceGrid::UpdateStats ObservedSpaceGrid::addObservation(
  const Eigen::Vector3d & sensor_origin,
  const std::vector<Eigen::Vector3d> & hit_endpoints,
  double receive_time_sec,
  double min_ray_length,
  double max_ray_length)
{
  UpdateStats stats;
  stats.endpoint_count = hit_endpoints.size();
  if (!configured_ || !sensor_origin.allFinite() ||
    !std::isfinite(receive_time_sec))
  {
    return stats;
  }

  if (!frames_.empty() &&
    receive_time_sec + 1e-9 < frames_.back().receive_time_sec)
  {
    clear();
  }
  expire(receive_time_sec);

  std::vector<int> frame_addresses;
  if (hit_endpoints.size() <=
    std::numeric_limits<std::size_t>::max() / 16)
  {
    frame_addresses.reserve(hit_endpoints.size() * 16);
  }
  std::vector<int> terminal_addresses;
  terminal_addresses.reserve(hit_endpoints.size());

  Eigen::Vector3i origin_index;
  if (!positionToIndex(sensor_origin, origin_index)) {
    return stats;
  }
  stats.sensor_origin_in_map = true;

  const int seed_steps =
    static_cast<int>(std::ceil(seed_radius_ * resolution_inv_));
  for (int x = -seed_steps; x <= seed_steps; ++x) {
    for (int y = -seed_steps; y <= seed_steps; ++y) {
      for (int z = -seed_steps; z <= seed_steps; ++z) {
        const Eigen::Vector3d offset(x, y, z);
        if (offset.norm() * resolution_ > seed_radius_ + 1e-9) {
          continue;
        }
        addDilatedVoxel(
          origin_index + Eigen::Vector3i(x, y, z), frame_addresses);
      }
    }
  }

  const double bounded_min_ray_length =
    std::isfinite(min_ray_length) ?
    std::max(0.0, min_ray_length) : 0.0;
  const double bounded_max_ray_length =
    std::isfinite(max_ray_length) ?
    std::max(bounded_min_ray_length, max_ray_length) : 0.0;
  const Eigen::Vector3d ray_start =
    (sensor_origin - origin_) * resolution_inv_;
  RayCaster raycaster;
  Eigen::Vector3d ray_voxel;

  for (const auto & measured_endpoint : hit_endpoints) {
    if (!measured_endpoint.allFinite()) {
      continue;
    }

    Eigen::Vector3d ray = measured_endpoint - sensor_origin;
    const double measured_length = ray.norm();
    if (!std::isfinite(measured_length) ||
      measured_length < bounded_min_ray_length ||
      measured_length < 1e-6)
    {
      continue;
    }

    if (measured_length > bounded_max_ray_length) {
      ray *= bounded_max_ray_length / measured_length;
    }
    const Eigen::Vector3d endpoint = sensor_origin + ray;
    const Eigen::Vector3d ray_end =
      (endpoint - origin_) * resolution_inv_;
    const Eigen::Vector3i terminal_index(
      static_cast<int>(std::floor(ray_end.x())),
      static_cast<int>(std::floor(ray_end.y())),
      static_cast<int>(std::floor(ray_end.z())));
    if (indexInMap(terminal_index)) {
      const int terminal_address = address(terminal_index);
      if (markScratchAddress(
          terminal_voxel_scratch_, terminal_address))
      {
        terminal_addresses.push_back(terminal_address);
      }
    }
    if (!raycaster.setInput(ray_start, ray_end)) {
      continue;
    }

    ++stats.accepted_ray_count;
    while (raycaster.step(ray_voxel)) {
      const Eigen::Vector3i index(
        static_cast<int>(ray_voxel.x()),
        static_cast<int>(ray_voxel.y()),
        static_cast<int>(ray_voxel.z()));
      addDilatedVoxel(index, frame_addresses);
    }
  }

  // A one-voxel raster tolerance can reach the terminal voxel from the
  // penultimate ray cell. Remove every terminal after all rays are combined
  // so another nearby ray cannot accidentally re-mark a measured hit.
  //
  // Each frame address was admitted through frame_voxel_scratch_, so this
  // compaction is linear in the number of unique touched voxels. Clear both
  // reusable masks sparsely from their address lists rather than scanning the
  // whole map between sensor frames.
  auto free_write = frame_addresses.begin();
  for (const int voxel_address : frame_addresses) {
    clearScratchAddress(frame_voxel_scratch_, voxel_address);
    if (!scratchContains(terminal_voxel_scratch_, voxel_address)) {
      *free_write++ = voxel_address;
    }
  }
  frame_addresses.erase(free_write, frame_addresses.end());
  for (const int terminal_address : terminal_addresses) {
    clearScratchAddress(terminal_voxel_scratch_, terminal_address);
  }

  // Refcounts are one byte per map voxel to keep the additional map memory
  // bounded. Retaining at most 255 frames makes overflow impossible while
  // preserving the configured time window at all practical sensor rates.
  constexpr std::size_t kMaxRetainedFrames =
    std::numeric_limits<std::uint8_t>::max();
  while (frames_.size() >= kMaxRetainedFrames) {
    removeOldestFrame();
  }

  for (const int voxel_address : frame_addresses) {
    auto & count = observation_ref_count_.at(
      static_cast<std::size_t>(voxel_address));
    if (count == 0) {
      active_voxel_addresses_.insert(voxel_address);
    }
    ++count;
  }

  stats.frame_voxel_count = frame_addresses.size();
  frames_.push_back(ObservationFrame{
      receive_time_sec, std::move(frame_addresses)});
  has_observation_ = true;
  last_observation_time_sec_ = receive_time_sec;
  stats.retained_voxel_count = active_voxel_addresses_.size();
  return stats;
}

bool ObservedSpaceGrid::isObserved(
  const Eigen::Vector3d & position) const
{
  Eigen::Vector3i index;
  if (!configured_ || !positionToIndex(position, index)) {
    return false;
  }
  return observation_ref_count_.at(
    static_cast<std::size_t>(address(index))) > 0;
}

bool ObservedSpaceGrid::isObservedWithClearance(
  const Eigen::Vector3d & position,
  double clearance_m) const
{
  Eigen::Vector3i center;
  if (!configured_ || !positionToIndex(position, center)) {
    return false;
  }

  const double clearance = std::max(0.0, clearance_m);
  const int steps =
    static_cast<int>(std::ceil(clearance * resolution_inv_));
  for (int x = -steps; x <= steps; ++x) {
    for (int y = -steps; y <= steps; ++y) {
      for (int z = -steps; z <= steps; ++z) {
        const Eigen::Vector3d offset(x, y, z);
        if (offset.norm() * resolution_ > clearance + 1e-9) {
          continue;
        }
        const Eigen::Vector3i index =
          center + Eigen::Vector3i(x, y, z);
        if (!indexInMap(index) ||
          observation_ref_count_.at(
            static_cast<std::size_t>(address(index))) == 0)
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool ObservedSpaceGrid::clampSegment(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & desired,
  double clearance_m,
  double frontier_margin_m,
  Eigen::Vector3d & target,
  bool & clamped) const
{
  if (!start.allFinite() || !desired.allFinite()) {
    target = start;
    clamped = true;
    return false;
  }

  target = start;
  clamped = true;
  if (!isObservedWithClearance(start, clearance_m)) {
    return false;
  }

  const Eigen::Vector3d delta = desired - start;
  const double distance = delta.norm();
  if (distance < 1e-6) {
    clamped = false;
    target = desired;
    return true;
  }

  const double sample_step = std::max(1e-3, resolution_ * 0.5);
  const int sample_count =
    std::max(1, static_cast<int>(std::ceil(distance / sample_step)));
  int last_safe_sample = 0;
  for (int sample = 1; sample <= sample_count; ++sample) {
    const double fraction =
      static_cast<double>(sample) / static_cast<double>(sample_count);
    const Eigen::Vector3d candidate = start + fraction * delta;
    if (!isObservedWithClearance(candidate, clearance_m)) {
      break;
    }
    last_safe_sample = sample;
  }

  if (last_safe_sample == sample_count) {
    clamped = false;
    target = desired;
    return true;
  }

  const double last_safe_distance =
    distance * static_cast<double>(last_safe_sample) /
    static_cast<double>(sample_count);
  const double target_distance = std::max(
    0.0, last_safe_distance - std::max(0.0, frontier_margin_m));
  target = start + delta.normalized() * target_distance;
  return true;
}

void ObservedSpaceGrid::observedPoints(
  int stride_voxels,
  std::vector<Eigen::Vector3d> & points) const
{
  points.clear();
  if (!configured_) {
    return;
  }

  const int stride = std::max(1, stride_voxels);
  points.reserve(active_voxel_addresses_.size());
  for (const int voxel_address : active_voxel_addresses_) {
    const Eigen::Vector3i index = addressToIndex(voxel_address);
    if (index.x() % stride == 0 &&
      index.y() % stride == 0 &&
      index.z() % stride == 0)
    {
      points.push_back(indexToPosition(index));
    }
  }
}

bool ObservedSpaceGrid::positionToIndex(
  const Eigen::Vector3d & position,
  Eigen::Vector3i & index) const
{
  if (!position.allFinite()) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    index(axis) = static_cast<int>(
      std::floor((position(axis) - origin_(axis)) * resolution_inv_));
  }
  return indexInMap(index);
}

bool ObservedSpaceGrid::indexInMap(
  const Eigen::Vector3i & index) const
{
  return
    index.x() >= 0 && index.x() < voxel_num_.x() &&
    index.y() >= 0 && index.y() < voxel_num_.y() &&
    index.z() >= 0 && index.z() < voxel_num_.z();
}

int ObservedSpaceGrid::address(const Eigen::Vector3i & index) const
{
  return
    index.x() * voxel_num_.y() * voxel_num_.z() +
    index.y() * voxel_num_.z() +
    index.z();
}

Eigen::Vector3d ObservedSpaceGrid::indexToPosition(
  const Eigen::Vector3i & index) const
{
  return
    (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
    resolution_ + origin_;
}

Eigen::Vector3i ObservedSpaceGrid::addressToIndex(
  int voxel_address) const
{
  const int yz_size = voxel_num_.y() * voxel_num_.z();
  const int x = voxel_address / yz_size;
  const int remainder = voxel_address % yz_size;
  const int y = remainder / voxel_num_.z();
  const int z = remainder % voxel_num_.z();
  return Eigen::Vector3i(x, y, z);
}

bool ObservedSpaceGrid::markScratchAddress(
  std::vector<std::uint64_t> & scratch_words,
  int voxel_address) const
{
  constexpr std::size_t kScratchWordBits =
    std::numeric_limits<std::uint64_t>::digits;
  const std::size_t unsigned_address =
    static_cast<std::size_t>(voxel_address);
  const std::size_t word_index =
    unsigned_address / kScratchWordBits;
  const std::uint64_t mask =
    std::uint64_t{1} << (unsigned_address % kScratchWordBits);
  auto & word = scratch_words[word_index];
  if ((word & mask) != 0) {
    return false;
  }
  word |= mask;
  return true;
}

bool ObservedSpaceGrid::scratchContains(
  const std::vector<std::uint64_t> & scratch_words,
  int voxel_address) const
{
  constexpr std::size_t kScratchWordBits =
    std::numeric_limits<std::uint64_t>::digits;
  const std::size_t unsigned_address =
    static_cast<std::size_t>(voxel_address);
  const std::uint64_t mask =
    std::uint64_t{1} << (unsigned_address % kScratchWordBits);
  return
    (scratch_words[unsigned_address / kScratchWordBits] & mask) != 0;
}

void ObservedSpaceGrid::clearScratchAddress(
  std::vector<std::uint64_t> & scratch_words,
  int voxel_address) const
{
  constexpr std::size_t kScratchWordBits =
    std::numeric_limits<std::uint64_t>::digits;
  const std::size_t unsigned_address =
    static_cast<std::size_t>(voxel_address);
  const std::uint64_t mask =
    std::uint64_t{1} << (unsigned_address % kScratchWordBits);
  scratch_words[unsigned_address / kScratchWordBits] &= ~mask;
}

void ObservedSpaceGrid::addDilatedVoxel(
  const Eigen::Vector3i & index,
  std::vector<int> & addresses)
{
  for (int x = -ray_dilation_voxels_; x <= ray_dilation_voxels_; ++x) {
    for (int y = -ray_dilation_voxels_; y <= ray_dilation_voxels_; ++y) {
      for (int z = -ray_dilation_voxels_; z <= ray_dilation_voxels_; ++z) {
        const Eigen::Vector3i dilated =
          index + Eigen::Vector3i(x, y, z);
        if (indexInMap(dilated)) {
          const int voxel_address = address(dilated);
          if (markScratchAddress(
              frame_voxel_scratch_, voxel_address))
          {
            addresses.push_back(voxel_address);
          }
        }
      }
    }
  }
}

void ObservedSpaceGrid::removeOldestFrame()
{
  if (frames_.empty()) {
    return;
  }

  for (const int voxel_address : frames_.front().voxel_addresses) {
    auto & count = observation_ref_count_.at(
      static_cast<std::size_t>(voxel_address));
    if (count == 0) {
      continue;
    }
    --count;
    if (count == 0) {
      active_voxel_addresses_.erase(voxel_address);
    }
  }
  frames_.pop_front();
}
