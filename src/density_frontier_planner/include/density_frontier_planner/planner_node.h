#ifndef DENSITY_FRONTIER_PLANNER_PLANNER_NODE_H_
#define DENSITY_FRONTIER_PLANNER_PLANNER_NODE_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include "density_frontier_planner/density_target_manager.h"
#include "density_frontier_planner/occupancy_grid.h"
#include "density_frontier_planner/types.h"
#include "density_frontier_planner/viewpoint_planner.h"

namespace density_frontier_planner {

class PlannerNode {
 public:
  PlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 private:
  void LoadParams();

  void DensityCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void ScanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void MapCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void OccupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void FrontierCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void StartCallback(const std_msgs::BoolConstPtr& msg);

  void PlanningTimerCallback(const ros::TimerEvent& event);
  void WaypointTimerCallback(const ros::TimerEvent& event);

  void PublishWaypoint(const Point3& point);
  void PublishInitialWaypoint();
  void PublishPath(const std::vector<Point3>& path);
  void PublishVisualization(const std::vector<Target>& targets,
                            const PlanResult& plan);
  void PublishStatus(const std::vector<Target>& targets,
                     const PlanResult& plan, double now_sec);
  void SetFinished(bool finished);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  Params params_;

  std::unique_ptr<OccupancyGrid2D> grid_;
  std::unique_ptr<DensityTargetManager> density_manager_;
  std::unique_ptr<ViewpointPlanner> viewpoint_planner_;

  ros::Subscriber density_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_cloud_sub_;
  ros::Subscriber grid_sub_;
  ros::Subscriber frontier_sub_;
  ros::Subscriber start_sub_;

  ros::Publisher waypoint_pub_;
  ros::Publisher path_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher density_target_pub_;
  ros::Publisher frontier_target_pub_;
  ros::Publisher viewpoint_pub_;
  ros::Publisher grid_debug_pub_;
  ros::Publisher status_pub_;
  ros::Publisher finish_pub_;

  ros::Timer planning_timer_;
  ros::Timer waypoint_timer_;

  std::vector<Point3> density_points_;
  double last_density_sec_ = -1.0;
  std::vector<Point3> frontier_points_;
  double last_frontier_sec_ = -1.0;

  Point3 robot_pos_;
  double robot_yaw_ = 0.0;
  bool odom_received_ = false;
  bool started_ = false;
  bool external_grid_received_ = false;
  double last_scan_process_sec_ = -1.0;
  double last_map_process_sec_ = -1.0;
  double last_prune_sec_ = -1.0;

  std::vector<Point3> current_path_;
  std::vector<uint64_t> planned_density_ids_;
  size_t path_index_ = 0;

  Point3 last_progress_pos_;
  double last_progress_sec_ = 0.0;
  bool finished_ = false;
  bool finish_published_ = false;
  bool replan_requested_ = true;
  int empty_target_cycles_ = 0;
  int plan_cycle_ = 0;
  double last_plan_sec_ = 0.0;
  double last_status_sec_ = -1.0;
};

}  // namespace density_frontier_planner

#endif  // DENSITY_FRONTIER_PLANNER_PLANNER_NODE_H_
