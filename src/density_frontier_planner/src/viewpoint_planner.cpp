#include "density_frontier_planner/viewpoint_planner.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>
#include <vector>

namespace density_frontier_planner {

ViewpointPlanner::ViewpointPlanner(const ViewpointParams& viewpoint_params,
                                   const PathParams& path_params)
    : viewpoint_params_(viewpoint_params), path_params_(path_params) {}

double ViewpointPlanner::OrderCost(const std::vector<Viewpoint>& viewpoints,
                                   const Point3& robot_pos) const {
  double cost = 0.0;
  Point3 from = robot_pos;
  for (const auto& viewpoint : viewpoints) {
    cost += Dist2D(from, viewpoint.position);
    from = viewpoint.position;
  }
  return cost;
}

PlanResult ViewpointPlanner::Plan(const Point3& robot_pos, double robot_yaw,
                                  const std::vector<Target>& targets,
                                  const OccupancyGrid2D& grid) const {
  PlanResult result;
  if (targets.empty()) return result;

  const int angle_bins = std::max(1, viewpoint_params_.angle_bins);
  const bool has_map = grid.HasUsableMap();

  // ---- 1. Candidate viewpoints around every target ----
  std::vector<Viewpoint> candidates;
  for (const auto& target : targets) {
    for (const double radius : viewpoint_params_.radii) {
      for (int a = 0; a < angle_bins; ++a) {
        const double angle = 2.0 * kPi * a / angle_bins;
        Point3 position(target.position.x + radius * std::cos(angle),
                        target.position.y + radius * std::sin(angle),
                        robot_pos.z + viewpoint_params_.height_above_ground);
        if (Dist2D(position, robot_pos) > viewpoint_params_.max_viewpoint_dist) {
          continue;
        }
        if (has_map && !grid.IsFree(position.x, position.y)) continue;
        if (viewpoint_params_.require_los &&
            !grid.IsLineOfSight(robot_pos, position)) {
          continue;
        }

        bool too_close = false;
        for (const auto& existing : candidates) {
          if (Dist2D(existing.position, position) <
              viewpoint_params_.min_separation) {
            too_close = true;
            break;
          }
        }
        if (too_close) continue;

        Viewpoint candidate;
        candidate.position = position;
        for (size_t i = 0; i < targets.size(); ++i) {
          const Target& other = targets[i];
          if (Dist3D(position, other.position) > viewpoint_params_.effective_range) {
            continue;
          }
          if (std::fabs(position.z - other.position.z) >
              viewpoint_params_.max_z_diff) {
            continue;
          }
          if (viewpoint_params_.require_los &&
              !grid.IsLineOfSight(position, other.position)) {
            continue;
          }
          candidate.covered.push_back(i);
          if (other.type == TargetType::kDensity) {
            candidate.covered_density_ids.push_back(other.id);
          }
        }
        if (candidate.covered.empty()) continue;
        candidates.push_back(std::move(candidate));
      }
    }
  }
  if (candidates.empty()) return result;

  // ---- 2. Greedy weighted set cover ----
  std::vector<bool> covered(targets.size(), false);
  std::vector<bool> used(candidates.size(), false);
  Point3 reference = robot_pos;
  double reference_yaw = robot_yaw;
  std::vector<Viewpoint> selected;

  const int max_selected = std::max(1, viewpoint_params_.max_selected);
  for (int k = 0; k < max_selected; ++k) {
    int best_index = -1;
    double best_score = viewpoint_params_.min_gain;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (used[i]) continue;
      double gain = 0.0;
      for (const size_t target_index : candidates[i].covered) {
        if (!covered[target_index]) gain += targets[target_index].weight;
      }
      if (gain <= 0.0) continue;

      const Point3& position = candidates[i].position;
      const double travel = Dist2D(reference, position);
      const double heading = std::atan2(position.y - reference.y,
                                        position.x - reference.x);
      const double score = gain -
                           viewpoint_params_.distance_penalty * travel -
                           viewpoint_params_.turn_penalty *
                               AngleDiff(heading, reference_yaw);
      if (score > best_score) {
        best_score = score;
        best_index = static_cast<int>(i);
      }
    }
    if (best_index < 0) break;

    used[best_index] = true;
    Viewpoint chosen = candidates[best_index];
    chosen.score = best_score;
    for (const size_t target_index : chosen.covered) covered[target_index] = true;
    reference_yaw = std::atan2(chosen.position.y - reference.y,
                               chosen.position.x - reference.x);
    reference = chosen.position;
    selected.push_back(std::move(chosen));
  }
  if (selected.empty()) return result;

  // ---- 3. 2-opt ordering ----
  for (int iteration = 0; iteration < 20; ++iteration) {
    bool improved = false;
    const double current_cost = OrderCost(selected, robot_pos);
    for (size_t i = 0; i < selected.size(); ++i) {
      for (size_t j = i + 1; j < selected.size(); ++j) {
        std::vector<Viewpoint> trial = selected;
        std::reverse(trial.begin() + i, trial.begin() + j + 1);
        if (OrderCost(trial, robot_pos) + 1e-6 < current_cost) {
          selected.swap(trial);
          improved = true;
        }
      }
    }
    if (!improved) break;
  }

  // ---- 4. A* path through the ordered viewpoints ----
  std::vector<Point3> dense_path;
  dense_path.push_back(robot_pos);
  Point3 from = robot_pos;
  for (const auto& viewpoint : selected) {
    std::vector<Point3> segment;
    grid.FindPath(from, viewpoint.position, segment);
    for (size_t i = 1; i < segment.size(); ++i) dense_path.push_back(segment[i]);
    from = viewpoint.position;
  }

  const double spacing = std::max(0.1, path_params_.path_point_spacing);
  result.path.push_back(dense_path.front());
  for (size_t i = 1; i < dense_path.size(); ++i) {
    if (Dist2D(result.path.back(), dense_path[i]) >= spacing) {
      result.path.push_back(dense_path[i]);
    }
  }
  if (Dist2D(result.path.back(), from) > 0.05) result.path.push_back(from);

  std::unordered_set<uint64_t> ids;
  for (const auto& viewpoint : selected) {
    for (const uint64_t id : viewpoint.covered_density_ids) ids.insert(id);
  }
  result.planned_density_ids.assign(ids.begin(), ids.end());
  result.viewpoints = std::move(selected);
  result.valid = result.path.size() >= 2;
  return result;
}

}  // namespace density_frontier_planner
