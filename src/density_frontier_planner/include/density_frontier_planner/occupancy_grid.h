#ifndef DENSITY_FRONTIER_PLANNER_OCCUPANCY_GRID_H_
#define DENSITY_FRONTIER_PLANNER_OCCUPANCY_GRID_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "density_frontier_planner/types.h"

namespace density_frontier_planner {

enum CellState : uint8_t {
  kUnknown = 0,
  kFree = 1,
  kOccupied = 2,
};

// A sparse, robot-centric three-state 2D grid.
// Only FREE and OCCUPIED cells are stored; anything absent is UNKNOWN.
// UNKNOWN is what makes classic (Yamauchi-style) frontier detection possible:
// a frontier is a FREE cell adjacent to UNKNOWN space.
class OccupancyGrid2D {
 public:
  explicit OccupancyGrid2D(const GridParams& params);

  // Preferred input: a three-state grid published by msp / move_base.
  // Cells inside the message are overwritten, -1 becomes UNKNOWN.
  void UpdateFromOccupancyGrid(const nav_msgs::OccupancyGrid& msg);

  // Fallback input: one registered scan, ray-cast into the grid.
  void UpdateFromScan(const std::vector<Point3>& cloud, const Point3& sensor_pos);

  // Supplementary input: global (downsampled) map, obstacles only.
  void UpdateFromMapCloud(const std::vector<Point3>& cloud, const Point3& robot_pos);

  std::vector<Target> ExtractFrontiers(const Point3& robot_pos,
                                       const FrontierParams& params) const;

  // A* on FREE cells. Falls back to a straight segment when the grid has no
  // observations yet, so the node still emits a waypoint on a cold start.
  bool FindPath(const Point3& start, const Point3& goal,
                std::vector<Point3>& path) const;

  bool IsLineOfSight(const Point3& from, const Point3& to) const;
  bool IsFree(double x, double y) const;
  bool HasFreeCells() const { return free_cell_count_ > 0; }
  bool HasUsableMap() const { return free_cell_count_ >= 200; }
  size_t CellCount() const { return cells_.size(); }

  void Prune(const Point3& center);

  nav_msgs::OccupancyGrid ToRosMessage(const std::string& frame,
                                       const Point3& center,
                                       double half_size) const;

 private:
  uint8_t Get(int cx, int cy) const;
  void Set(int cx, int cy, uint8_t state);
  std::pair<int, int> ToCell(const Point3& p) const;
  Point3 ToPos(int cx, int cy, double z) const;
  void TraceRay(const Point3& sensor_pos, const Point3& hit, bool occupied);
  std::vector<std::pair<int, int>> Bresenham(int x0, int y0, int x1, int y1) const;

  GridParams params_;
  std::unordered_map<uint64_t, uint8_t> cells_;
  size_t free_cell_count_ = 0;
};

}  // namespace density_frontier_planner

#endif  // DENSITY_FRONTIER_PLANNER_OCCUPANCY_GRID_H_
