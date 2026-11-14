#ifndef DENSITY_FRONTIER_PLANNER_TYPES_H_
#define DENSITY_FRONTIER_PLANNER_TYPES_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace density_frontier_planner {

constexpr double kPi = 3.14159265358979323846;

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3() = default;
  Point3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}
};

inline double Dist2D(const Point3& a, const Point3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double Dist3D(const Point3& a, const Point3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double NormalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

inline double AngleDiff(double a, double b) {
  return std::fabs(NormalizeAngle(a - b));
}

// Pack signed grid indices into one collision-free unsigned key.
inline uint64_t MakeCellKey(int cx, int cy) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(cy));
}

inline int CellX(uint64_t key) {
  return static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
}

inline int CellY(uint64_t key) {
  return static_cast<int32_t>(static_cast<uint32_t>(key & 0xffffffffULL));
}

inline int PosToCellIndex(double value, double cell_size) {
  return static_cast<int>(std::floor(value / cell_size));
}

enum class TargetType : uint8_t { kDensity = 0, kFrontier = 1 };

struct Target {
  uint64_t id = 0;
  Point3 position;
  TargetType type = TargetType::kDensity;
  int support = 1;
  double weight = 0.0;
};

struct Viewpoint {
  Point3 position;
  std::vector<size_t> covered;
  std::vector<uint64_t> covered_density_ids;
  double gain = 0.0;
  double cost = 0.0;
  double score = 0.0;
};

struct PlanResult {
  std::vector<Viewpoint> viewpoints;
  std::vector<Point3> path;
  std::vector<uint64_t> planned_density_ids;
  bool valid = false;
};

struct TopicParams {
  std::string density_cloud = "/FilteringMappingNodelet/pcd_density_map";
  std::string odom = "/state_estimation_at_scan";
  std::string scan = "/registered_scan";
  std::string map_cloud = "/FilteringMappingNodelet/pcd_map";
  std::string occupancy_grid = "/msp/occupancy_grid";
  std::string frontier_cloud = "/sensor_coverage_planner/filtered_frontier_cloud";
  std::string waypoint = "/way_point";
  std::string path = "viewpoint_path";
  std::string move_base_goal = "/move_base_simple/goal";
  std::string start = "/start_exploration";
};

struct GridParams {
  double resolution = 0.3;
  int occupied_threshold = 65;
  double ray_max_range = 20.0;
  int ray_bins = 360;
  double obstacle_min_height = 0.25;  // relative to robot z
  double obstacle_max_height = 2.0;
  double prune_radius = 60.0;
  int max_cells = 200000;
};

struct FrontierParams {
  bool enable = true;
  bool use_external_cloud = false;
  int min_cluster_size = 6;
  int min_unknown_neighbors = 2;
  double max_range = 15.0;
  int max_targets = 12;
  double weight = 1.0;
  double distance_scale = 12.0;
  double cluster_cell_size = 0.6;  // only for an external frontier cloud
};

struct DensityParams {
  double anchor_cell_size = 1.5;
  int min_observation_cycles = 2;
  int missing_cycles_to_forget = 5;
  int abandoned_forget_cycles = 30;
  int min_support_points = 1;
  int max_targets = 24;
  double weight = 3.0;
  double distance_scale = 12.0;
  double service_radius = 2.5;
  double service_release_ratio = 1.5;
  double service_settle_sec = 4.0;
  int max_failures = 3;
  double cooldown_base_sec = 8.0;
  double cooldown_max_sec = 120.0;
  double fail_penalty = 2.0;
  double centroid_smoothing = 0.3;
};

struct ViewpointParams {
  std::vector<double> radii = {1.5, 3.0};
  int angle_bins = 12;
  double max_viewpoint_dist = 12.0;
  double effective_range = 5.0;
  double max_z_diff = 3.0;
  double min_separation = 0.8;
  int max_selected = 5;
  bool require_los = true;
  double distance_penalty = 0.35;
  double turn_penalty = 0.15;
  double min_gain = 0.02;
  double height_above_ground = 0.0;
};

struct PathParams {
  double lookahead_dist = 3.5;
  double goal_tolerance = 0.6;
  double path_point_spacing = 0.5;
  double replan_interval_sec = 1.0;      // planning timer period
  double max_replan_interval_sec = 5.0;  // force re-plan even if not arrived
  double stall_sec = 8.0;
  double stall_distance = 0.3;
  double waypoint_rate = 10.0;
  bool publish_move_base_goal = false;
};

struct Params {
  std::string world_frame = "map";
  bool autostart = true;
  double density_stale_sec = 5.0;
  double log_period_sec = 5.0;
  double initial_waypoint_dist = 8.0;  // cold-start waypoint, like tare
  int finish_hysteresis_cycles = 5;
  bool publish_grid_debug = false;
  double grid_debug_half_size = 20.0;

  TopicParams topics;
  GridParams grid;
  FrontierParams frontier;
  DensityParams density;
  ViewpointParams viewpoint;
  PathParams path;
};

// Decode x/y/z from a PointCloud2 without PCL. Tolerates extra fields
// (intensity, rgb, ...) and both FLOAT32 and FLOAT64 coordinates.
inline bool ExtractXYZ(const sensor_msgs::PointCloud2& msg, std::vector<Point3>& out) {
  int offsets[3] = {-1, -1, -1};
  uint8_t datatypes[3] = {0, 0, 0};
  for (const auto& field : msg.fields) {
    int axis = -1;
    if (field.name == "x") axis = 0;
    else if (field.name == "y") axis = 1;
    else if (field.name == "z") axis = 2;
    if (axis < 0) continue;
    offsets[axis] = static_cast<int>(field.offset);
    datatypes[axis] = field.datatype;
  }

  for (int i = 0; i < 3; ++i) {
    if (offsets[i] < 0) return false;
    if (datatypes[i] != sensor_msgs::PointField::FLOAT32 &&
        datatypes[i] != sensor_msgs::PointField::FLOAT64) {
      return false;
    }
  }
  if (msg.point_step == 0) return false;

  const size_t point_num = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  out.clear();
  if (point_num == 0) return true;  // a legitimately empty map
  if (msg.data.size() < point_num * msg.point_step) return false;
  const uint8_t* base = msg.data.data();
  out.reserve(point_num);

  for (size_t i = 0; i < point_num; ++i) {
    const uint8_t* ptr = base + i * msg.point_step;
    double v[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
      if (datatypes[k] == sensor_msgs::PointField::FLOAT32) {
        float value = 0.0f;
        std::memcpy(&value, ptr + offsets[k], sizeof(float));
        v[k] = static_cast<double>(value);
      } else {
        double value = 0.0;
        std::memcpy(&value, ptr + offsets[k], sizeof(double));
        v[k] = value;
      }
    }
    if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) continue;
    out.emplace_back(v[0], v[1], v[2]);
  }
  return true;
}

// Anchor-cell aggregation, shared by the density manager and external
// frontier clouds. Stable IDs are the cell keys themselves.
inline std::vector<Target> ClusterPoints(const std::vector<Point3>& points,
                                         double cell_size, int min_support,
                                         TargetType type, uint64_t id_salt) {
  struct Accumulator {
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    int count = 0;
  };
  std::unordered_map<uint64_t, Accumulator> bins;
  for (const auto& p : points) {
    const int cx = PosToCellIndex(p.x, cell_size);
    const int cy = PosToCellIndex(p.y, cell_size);
    auto& acc = bins[MakeCellKey(cx, cy)];
    acc.sx += p.x;
    acc.sy += p.y;
    acc.sz += p.z;
    ++acc.count;
  }

  std::vector<Target> targets;
  targets.reserve(bins.size());
  for (const auto& entry : bins) {
    const auto& acc = entry.second;
    if (acc.count < min_support) continue;
    Target target;
    target.id = entry.first ^ id_salt;
    target.position = Point3(acc.sx / acc.count, acc.sy / acc.count, acc.sz / acc.count);
    target.support = acc.count;
    target.type = type;
    targets.push_back(target);
  }
  return targets;
}

}  // namespace density_frontier_planner

#endif  // DENSITY_FRONTIER_PLANNER_TYPES_H_
