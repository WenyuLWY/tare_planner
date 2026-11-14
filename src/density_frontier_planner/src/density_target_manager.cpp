#include "density_frontier_planner/density_target_manager.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace density_frontier_planner {

DensityTargetManager::DensityTargetManager(const DensityParams& params)
    : params_(params) {}

void DensityTargetManager::RegisterFailure(DensityRecord& record, double now_sec) {
  ++record.failures;
  record.attempt_time_sec = -1.0;
  record.inside_service = false;
  const int exponent = std::min(std::max(record.failures - 1, 0), 5);
  const double backoff = params_.cooldown_base_sec * std::pow(2.0, exponent);
  record.cooldown_until_sec =
      now_sec + std::min(backoff, params_.cooldown_max_sec);
  if (record.failures >= params_.max_failures) {
    record.abandoned = true;
  }
}

void DensityTargetManager::Update(const std::vector<Point3>& points,
                                  double now_sec, const Point3& robot_pos,
                                  const OccupancyGrid2D& grid) {
  raw_point_count_ = static_cast<int>(points.size());
  const std::vector<Target> clusters = ClusterPoints(
      points, params_.anchor_cell_size, 1, TargetType::kDensity, 0);
  unique_target_count_ = static_cast<int>(clusters.size());

  std::unordered_set<uint64_t> seen;
  seen.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    seen.insert(cluster.id);
    auto it = records_.find(cluster.id);
    if (it == records_.end()) {
      DensityRecord record;
      record.id = cluster.id;
      record.position = cluster.position;
      record.support = cluster.support;
      record.observed_cycles = 1;
      record.first_seen_sec = now_sec;
      record.last_seen_sec = now_sec;
      records_.emplace(cluster.id, record);
      continue;
    }

    DensityRecord& record = it->second;
    const double alpha =
        std::min(1.0, std::max(0.0, params_.centroid_smoothing));
    record.position.x =
        (1.0 - alpha) * record.position.x + alpha * cluster.position.x;
    record.position.y =
        (1.0 - alpha) * record.position.y + alpha * cluster.position.y;
    record.position.z =
        (1.0 - alpha) * record.position.z + alpha * cluster.position.z;
    record.support = cluster.support;
    ++record.observed_cycles;
    record.missing_cycles = 0;
    record.last_seen_sec = now_sec;
  }

  for (auto it = records_.begin(); it != records_.end();) {
    DensityRecord& record = it->second;
    if (seen.count(record.id) == 0) {
      ++record.missing_cycles;
      const int forget_cycles = record.abandoned
                                    ? params_.abandoned_forget_cycles
                                    : params_.missing_cycles_to_forget;
      if (record.missing_cycles > forget_cycles) {
        it = records_.erase(it);
        continue;
      }
      record.inside_service = false;
    }
    ++it;
  }

  for (auto& entry : records_) {
    DensityRecord& record = entry.second;
    if (seen.count(record.id) == 0) continue;

    const double dist = Dist2D(robot_pos, record.position);
    const bool visible = grid.IsLineOfSight(robot_pos, record.position);
    if (!record.inside_service && dist <= params_.service_radius && visible) {
      record.inside_service = true;
      ++record.attempts;
      if (record.attempt_time_sec < 0.0) record.attempt_time_sec = now_sec;
    } else if (record.inside_service &&
               dist > params_.service_radius * params_.service_release_ratio) {
      record.inside_service = false;
    }

    // The mapping node keeps publishing this point after the robot has been
    // close enough to re-scan it: treat it as a failed service.
    if (record.attempt_time_sec >= 0.0 &&
        now_sec - record.attempt_time_sec >= params_.service_settle_sec) {
      RegisterFailure(record, now_sec);
    }
  }
}

std::vector<Target> DensityTargetManager::GetActiveTargets(
    const Point3& robot_pos, double now_sec) const {
  std::vector<Target> targets;
  targets.reserve(records_.size());
  for (const auto& entry : records_) {
    const DensityRecord& record = entry.second;
    if (record.abandoned) continue;
    if (record.observed_cycles < params_.min_observation_cycles) continue;
    if (record.support < params_.min_support_points) continue;
    if (now_sec < record.cooldown_until_sec) continue;

    const double dist = Dist2D(robot_pos, record.position);
    const double age_bonus =
        1.0 + std::min((now_sec - record.first_seen_sec) / 60.0, 1.0);
    const double failure_penalty =
        std::pow(1.0 + record.failures, params_.fail_penalty);
    const double weight = params_.weight * age_bonus *
                          std::min(record.support, 8) *
                          std::exp(-dist / params_.distance_scale) /
                          failure_penalty;
    if (weight <= 0.0) continue;

    Target target;
    target.id = record.id;
    target.position = record.position;
    target.support = record.support;
    target.type = TargetType::kDensity;
    target.weight = weight;
    targets.push_back(target);
  }

  std::sort(targets.begin(), targets.end(), [](const Target& a, const Target& b) {
    if (a.weight != b.weight) return a.weight > b.weight;
    return a.id < b.id;
  });
  if (static_cast<int>(targets.size()) > params_.max_targets) {
    targets.resize(params_.max_targets);
  }
  return targets;
}

void DensityTargetManager::NoteStall(const std::vector<uint64_t>& ids,
                                     double now_sec) {
  for (const uint64_t id : ids) {
    auto it = records_.find(id);
    if (it == records_.end() || it->second.abandoned) continue;
    RegisterFailure(it->second, now_sec);
  }
}

}  // namespace density_frontier_planner
