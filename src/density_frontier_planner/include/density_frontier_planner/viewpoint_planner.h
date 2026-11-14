#ifndef DENSITY_FRONTIER_PLANNER_VIEWPOINT_PLANNER_H_
#define DENSITY_FRONTIER_PLANNER_VIEWPOINT_PLANNER_H_

#include <vector>

#include "density_frontier_planner/occupancy_grid.h"
#include "density_frontier_planner/types.h"

namespace density_frontier_planner {

// Turns weighted density/frontier targets into a short ordered viewpoint
// sequence (greedy weighted set-cover + 2-opt) and then into a traversable
// path (A* on the three-state grid). No solver dependency: the whole planner
// is standard C++ so it builds anywhere roscpp builds.
class ViewpointPlanner {
 public:
  ViewpointPlanner(const ViewpointParams& viewpoint_params,
                   const PathParams& path_params);

  PlanResult Plan(const Point3& robot_pos, double robot_yaw,
                  const std::vector<Target>& targets,
                  const OccupancyGrid2D& grid) const;

 private:
  double OrderCost(const std::vector<Viewpoint>& viewpoints,
                   const Point3& robot_pos) const;

  ViewpointParams viewpoint_params_;
  PathParams path_params_;
};

}  // namespace density_frontier_planner

#endif  // DENSITY_FRONTIER_PLANNER_VIEWPOINT_PLANNER_H_
