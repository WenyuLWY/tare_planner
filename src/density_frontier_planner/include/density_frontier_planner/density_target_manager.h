#ifndef DENSITY_FRONTIER_PLANNER_DENSITY_TARGET_MANAGER_H_
#define DENSITY_FRONTIER_PLANNER_DENSITY_TARGET_MANAGER_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "density_frontier_planner/occupancy_grid.h"
#include "density_frontier_planner/types.h"

namespace density_frontier_planner {

struct DensityRecord {
  uint64_t id = 0;
  Point3 position;
  int support = 0;
  int observed_cycles = 0;
  int missing_cycles = 0;
  double first_seen_sec = 0.0;
  double last_seen_sec = 0.0;
  int attempts = 0;
  int failures = 0;
  bool inside_service = false;
  double attempt_time_sec = -1.0;  // pending service result, -1 = none
  double cooldown_until_sec = 0.0;
  bool abandoned = false;
};

// Two layers of de-duplication for the continuously re-published density map:
//   1) spatial: anchor-cell aggregation collapses repeated / nearby points;
//   2) temporal: a persistent record keyed by the anchor cell keeps its
//      observation, service and failure history across full-map refreshes.
// A target that survives several servicing attempts is declared un-coverable
// and stops driving the planner (exponential-backoff cooldown, then abandon).
class DensityTargetManager {
 public:
  explicit DensityTargetManager(const DensityParams& params);

  void Update(const std::vector<Point3>& points, double now_sec,
              const Point3& robot_pos, const OccupancyGrid2D& grid);

  std::vector<Target> GetActiveTargets(const Point3& robot_pos,
                                       double now_sec) const;

  // Called when the robot made no progress while following a plan.
  void NoteStall(const std::vector<uint64_t>& ids, double now_sec);

  const std::unordered_map<uint64_t, DensityRecord>& Records() const {
    return records_;
  }

  int RawPointCount() const { return raw_point_count_; }
  int UniqueTargetCount() const { return unique_target_count_; }

 private:
  void RegisterFailure(DensityRecord& record, double now_sec);

  DensityParams params_;
  std::unordered_map<uint64_t, DensityRecord> records_;
  int raw_point_count_ = 0;
  int unique_target_count_ = 0;
};

}  // namespace density_frontier_planner

#endif  // DENSITY_FRONTIER_PLANNER_DENSITY_TARGET_MANAGER_H_
