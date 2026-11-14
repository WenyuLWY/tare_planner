#include "density_frontier_planner/occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

namespace density_frontier_planner {
namespace {

struct AStarNode {
  int cx = 0;
  int cy = 0;
  double g = 0.0;
  double f = 0.0;
};

struct AStarCompare {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f > b.f;
  }
};

double Heuristic(int cx, int cy, int gx, int gy) {
  const double dx = std::abs(cx - gx);
  const double dy = std::abs(cy - gy);
  const double diag = std::min(dx, dy);
  return (dx + dy - 2.0 * diag) + std::sqrt(2.0) * diag;
}

}  // namespace

OccupancyGrid2D::OccupancyGrid2D(const GridParams& params) : params_(params) {}

uint8_t OccupancyGrid2D::Get(int cx, int cy) const {
  const auto it = cells_.find(MakeCellKey(cx, cy));
  return it == cells_.end() ? kUnknown : it->second;
}

void OccupancyGrid2D::Set(int cx, int cy, uint8_t state) {
  const uint64_t key = MakeCellKey(cx, cy);
  auto it = cells_.find(key);
  if (state == kUnknown) {
    if (it != cells_.end()) {
      if (it->second == kFree) --free_cell_count_;
      cells_.erase(it);
    }
    return;
  }
  if (it == cells_.end()) {
    cells_.emplace(key, state);
    if (state == kFree) ++free_cell_count_;
    return;
  }
  if (it->second == kFree && state != kFree) --free_cell_count_;
  if (it->second != kFree && state == kFree) ++free_cell_count_;
  it->second = state;
}

std::pair<int, int> OccupancyGrid2D::ToCell(const Point3& p) const {
  return {PosToCellIndex(p.x, params_.resolution),
          PosToCellIndex(p.y, params_.resolution)};
}

Point3 OccupancyGrid2D::ToPos(int cx, int cy, double z) const {
  return Point3((cx + 0.5) * params_.resolution,
                (cy + 0.5) * params_.resolution, z);
}

std::vector<std::pair<int, int>> OccupancyGrid2D::Bresenham(int x0, int y0,
                                                             int x1, int y1) const {
  std::vector<std::pair<int, int>> cells;
  const int max_steps =
      static_cast<int>(params_.ray_max_range / params_.resolution) * 3 + 16;
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    cells.emplace_back(x0, y0);
    if ((x0 == x1 && y0 == y1) || static_cast<int>(cells.size()) >= max_steps) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
  return cells;
}

void OccupancyGrid2D::TraceRay(const Point3& sensor_pos, const Point3& hit,
                               bool occupied) {
  const auto from = ToCell(sensor_pos);
  const auto to = ToCell(hit);
  const auto cells = Bresenham(from.first, from.second, to.first, to.second);
  for (size_t i = 0; i < cells.size(); ++i) {
    const bool is_last = (i + 1 == cells.size());
    Set(cells[i].first, cells[i].second,
        (is_last && occupied) ? kOccupied : kFree);
  }
}

void OccupancyGrid2D::UpdateFromOccupancyGrid(const nav_msgs::OccupancyGrid& msg) {
  const auto& info = msg.info;
  if (info.resolution <= 0.0 || info.width == 0 || info.height == 0) return;
  if (msg.data.size() < static_cast<size_t>(info.width) * info.height) return;

  const double res = info.resolution;
  const double yaw = 2.0 * std::atan2(info.origin.orientation.z,
                                      info.origin.orientation.w);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  for (uint32_t row = 0; row < info.height; ++row) {
    for (uint32_t col = 0; col < info.width; ++col) {
      const int8_t value = msg.data[static_cast<size_t>(row) * info.width + col];
      const double lx = (col + 0.5) * res;
      const double ly = (row + 0.5) * res;
      const double wx = info.origin.position.x + c * lx - s * ly;
      const double wy = info.origin.position.y + s * lx + c * ly;
      const int cx = PosToCellIndex(wx, params_.resolution);
      const int cy = PosToCellIndex(wy, params_.resolution);
      uint8_t state = kUnknown;
      if (value >= 0) {
        state = (value >= params_.occupied_threshold) ? kOccupied : kFree;
      }
      Set(cx, cy, state);
    }
  }
}

void OccupancyGrid2D::UpdateFromScan(const std::vector<Point3>& cloud,
                                     const Point3& sensor_pos) {
  const int bins = std::max(1, params_.ray_bins);
  struct Bin {
    double obstacle_range = std::numeric_limits<double>::infinity();
    Point3 obstacle_point;
    double ground_range = 0.0;
  };
  std::vector<Bin> ray_bins(bins);

  for (const auto& p : cloud) {
    const double dx = p.x - sensor_pos.x;
    const double dy = p.y - sensor_pos.y;
    const double range = std::sqrt(dx * dx + dy * dy);
    if (range < 0.3 || range > params_.ray_max_range) continue;

    const double dz = p.z - sensor_pos.z;
    const bool obstacle = (dz >= params_.obstacle_min_height &&
                           dz <= params_.obstacle_max_height);
    const double angle = std::atan2(dy, dx);
    int bin = static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * bins));
    bin = std::max(0, std::min(bins - 1, bin));
    Bin& slot = ray_bins[bin];
    if (obstacle) {
      if (range < slot.obstacle_range) {
        slot.obstacle_range = range;
        slot.obstacle_point = p;
      }
    } else {
      slot.ground_range = std::max(slot.ground_range, range);
    }
  }

  for (size_t i = 0; i < ray_bins.size(); ++i) {
    const Bin& slot = ray_bins[i];
    const bool has_obstacle = std::isfinite(slot.obstacle_range);
    const double free_range =
        has_obstacle ? slot.obstacle_range : slot.ground_range;
    if (free_range < 0.3) continue;

    const double angle =
        (static_cast<double>(i) + 0.5) / bins * 2.0 * kPi - kPi;
    const Point3 ray_end(sensor_pos.x + std::cos(angle) * free_range,
                         sensor_pos.y + std::sin(angle) * free_range,
                         sensor_pos.z);
    TraceRay(sensor_pos, ray_end, false);
    if (has_obstacle) {
      const auto cell = ToCell(slot.obstacle_point);
      Set(cell.first, cell.second, kOccupied);
    }
  }
}

void OccupancyGrid2D::UpdateFromMapCloud(const std::vector<Point3>& cloud,
                                         const Point3& robot_pos) {
  for (const auto& p : cloud) {
    const double dz = p.z - robot_pos.z;
    if (dz < params_.obstacle_min_height || dz > params_.obstacle_max_height) {
      continue;
    }
    const auto cell = ToCell(p);
    Set(cell.first, cell.second, kOccupied);
  }
}

bool OccupancyGrid2D::IsFree(double x, double y) const {
  return Get(PosToCellIndex(x, params_.resolution),
             PosToCellIndex(y, params_.resolution)) == kFree;
}

bool OccupancyGrid2D::IsLineOfSight(const Point3& from, const Point3& to) const {
  // Degrade to "no occlusion known" until enough FREE cells have accumulated.
  // Otherwise a cold start (a handful of ray-cast cells) rejects every
  // candidate viewpoint and the robot never receives its first waypoint.
  if (free_cell_count_ < 200) return true;
  const auto a = ToCell(from);
  const auto b = ToCell(to);
  const auto cells = Bresenham(a.first, a.second, b.first, b.second);
  for (size_t i = 1; i + 1 < cells.size(); ++i) {
    if (Get(cells[i].first, cells[i].second) != kFree) return false;
  }
  return true;
}

std::vector<Target> OccupancyGrid2D::ExtractFrontiers(
    const Point3& robot_pos, const FrontierParams& params) const {
  std::vector<Target> targets;
  if (!HasFreeCells()) return targets;

  const int kNeighbors[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  std::unordered_set<uint64_t> frontier_cells;
  for (const auto& entry : cells_) {
    if (entry.second != kFree) continue;
    const int cx = CellX(entry.first);
    const int cy = CellY(entry.first);
    if (Dist2D(ToPos(cx, cy, robot_pos.z), robot_pos) > params.max_range) continue;
    int unknown_neighbors = 0;
    for (const auto& n : kNeighbors) {
      if (Get(cx + n[0], cy + n[1]) == kUnknown) ++unknown_neighbors;
    }
    if (unknown_neighbors >= params.min_unknown_neighbors) {
      frontier_cells.insert(entry.first);
    }
  }

  const int kClusterNeighbors[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                                        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  std::unordered_set<uint64_t> visited;
  for (const auto& seed : frontier_cells) {
    if (visited.count(seed) != 0) continue;
    std::vector<uint64_t> cluster;
    std::queue<uint64_t> queue;
    queue.push(seed);
    visited.insert(seed);
    while (!queue.empty()) {
      const uint64_t key = queue.front();
      queue.pop();
      cluster.push_back(key);
      const int cx = CellX(key);
      const int cy = CellY(key);
      for (const auto& n : kClusterNeighbors) {
        const uint64_t next = MakeCellKey(cx + n[0], cy + n[1]);
        if (frontier_cells.count(next) == 0 || visited.count(next) != 0) continue;
        visited.insert(next);
        queue.push(next);
      }
    }
    if (static_cast<int>(cluster.size()) < params.min_cluster_size) continue;

    double sx = 0.0;
    double sy = 0.0;
    for (const uint64_t key : cluster) {
      const Point3 p = ToPos(CellX(key), CellY(key), robot_pos.z);
      sx += p.x;
      sy += p.y;
    }
    Target target;
    target.id = MakeCellKey(PosToCellIndex(sx / cluster.size(), params_.resolution),
                            PosToCellIndex(sy / cluster.size(), params_.resolution));
    target.position = Point3(sx / cluster.size(), sy / cluster.size(), robot_pos.z);
    target.support = static_cast<int>(cluster.size());
    target.type = TargetType::kFrontier;
    targets.push_back(target);
  }

  std::sort(targets.begin(), targets.end(), [](const Target& a, const Target& b) {
    if (a.support != b.support) return a.support > b.support;
    return a.id < b.id;
  });
  if (static_cast<int>(targets.size()) > params.max_targets) {
    targets.resize(params.max_targets);
  }
  return targets;
}

bool OccupancyGrid2D::FindPath(const Point3& start, const Point3& goal,
                               std::vector<Point3>& path) const {
  path.clear();
  if (!HasFreeCells()) {
    path = {start, goal};
    return true;  // straight-line fallback on a cold start
  }

  const auto s = ToCell(start);
  const auto g = ToCell(goal);
  if (s == g) {
    path = {start, goal};
    return true;
  }

  const int kNeighbors[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                                {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare> open;
  std::unordered_map<uint64_t, double> best_g;
  std::unordered_map<uint64_t, uint64_t> parent;

  const uint64_t start_key = MakeCellKey(s.first, s.second);
  const uint64_t goal_key = MakeCellKey(g.first, g.second);
  best_g[start_key] = 0.0;
  open.push(AStarNode{s.first, s.second, 0.0,
                      Heuristic(s.first, s.second, g.first, g.second)});

  constexpr int kMaxExpansions = 20000;
  int expansions = 0;
  bool found = false;
  while (!open.empty() && expansions < kMaxExpansions) {
    const AStarNode node = open.top();
    open.pop();
    ++expansions;
    const uint64_t key = MakeCellKey(node.cx, node.cy);
    const auto known = best_g.find(key);
    if (known == best_g.end() || node.g > known->second + 1e-9) continue;
    if (key == goal_key) {
      found = true;
      break;
    }
    for (const auto& n : kNeighbors) {
      const int nx = node.cx + n[0];
      const int ny = node.cy + n[1];
      const uint64_t next_key = MakeCellKey(nx, ny);
      const uint8_t state = Get(nx, ny);
      if (state == kOccupied) continue;
      if (state == kUnknown && next_key != goal_key) continue;
      if (n[0] != 0 && n[1] != 0) {
        if (Get(node.cx + n[0], node.cy) == kOccupied ||
            Get(node.cx, node.cy + n[1]) == kOccupied) {
          continue;
        }
      }
      const double step = (n[0] != 0 && n[1] != 0) ? std::sqrt(2.0) : 1.0;
      const double next_g = node.g + step;
      const auto it = best_g.find(next_key);
      if (it != best_g.end() && it->second <= next_g + 1e-9) continue;
      best_g[next_key] = next_g;
      parent[next_key] = key;
      open.push(AStarNode{nx, ny, next_g,
                          next_g + Heuristic(nx, ny, g.first, g.second)});
    }
  }

  if (!found) {
    path = {start, goal};
    return false;  // degraded segment, local_planner still avoids obstacles
  }

  std::vector<uint64_t> keys;
  uint64_t cursor = goal_key;
  while (cursor != start_key) {
    keys.push_back(cursor);
    const auto it = parent.find(cursor);
    if (it == parent.end()) {
      path = {start, goal};
      return false;
    }
    cursor = it->second;
  }
  keys.push_back(start_key);
  std::reverse(keys.begin(), keys.end());
  for (const uint64_t key : keys) {
    path.push_back(ToPos(CellX(key), CellY(key), start.z));
  }
  path.back() = goal;
  return true;
}

void OccupancyGrid2D::Prune(const Point3& center) {
  double radius = params_.prune_radius;
  if (cells_.size() > static_cast<size_t>(params_.max_cells)) {
    radius = std::max(5.0, radius * 0.5);
  }
  for (auto it = cells_.begin(); it != cells_.end();) {
    const Point3 p = ToPos(CellX(it->first), CellY(it->first), 0.0);
    if (Dist2D(p, center) > radius) {
      if (it->second == kFree) --free_cell_count_;
      it = cells_.erase(it);
    } else {
      ++it;
    }
  }
}

nav_msgs::OccupancyGrid OccupancyGrid2D::ToRosMessage(const std::string& frame,
                                                      const Point3& center,
                                                      double half_size) const {
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = frame;
  grid.info.resolution = static_cast<float>(params_.resolution);
  const int half_cells = std::min(
      200, std::max(1, static_cast<int>(half_size / params_.resolution)));
  const int width = 2 * half_cells + 1;
  const auto center_cell = ToCell(center);
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(width);
  grid.info.origin.position.x =
      (center_cell.first - half_cells + 0.5) * params_.resolution;
  grid.info.origin.position.y =
      (center_cell.second - half_cells + 0.5) * params_.resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(width) * width, -1);

  for (int row = 0; row < width; ++row) {
    for (int col = 0; col < width; ++col) {
      const uint8_t state =
          Get(center_cell.first - half_cells + col,
              center_cell.second - half_cells + row);
      int8_t value = -1;
      if (state == kFree) value = 0;
      else if (state == kOccupied) value = 100;
      grid.data[static_cast<size_t>(row) * width + col] = value;
    }
  }
  return grid;
}

}  // namespace density_frontier_planner
