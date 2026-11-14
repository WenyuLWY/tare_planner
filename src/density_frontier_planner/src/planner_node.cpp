#include "density_frontier_planner/planner_node.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>

#include <sensor_msgs/PointField.h>

namespace density_frontier_planner {
namespace {

constexpr uint64_t kFrontierIdSalt = 0x9e3779b97f4a7c15ULL;

double GetYaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

sensor_msgs::PointCloud2 MakeXYZICloud(
    const std::vector<std::pair<Point3, float>>& points,
    const std::string& frame, const ros::Time& stamp) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  const char* kNames[4] = {"x", "y", "z", "intensity"};
  cloud.fields.resize(4);
  for (int i = 0; i < 4; ++i) {
    cloud.fields[i].name = kNames[i];
    cloud.fields[i].offset = static_cast<uint32_t>(i * 4);
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * points.size();
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const float values[4] = {static_cast<float>(points[i].first.x),
                             static_cast<float>(points[i].first.y),
                             static_cast<float>(points[i].first.z),
                             points[i].second};
    std::memcpy(cloud.data.data() + i * cloud.point_step, values, sizeof(values));
  }
  return cloud;
}

double PathLength(const std::vector<Point3>& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += Dist2D(path[i - 1], path[i]);
  }
  return length;
}

// Kept as a plain string parameter so the node has no XmlRpc / vector-param
// dependency at all.
std::vector<double> ParseRadii(const std::string& text,
                               const std::vector<double>& fallback) {
  std::vector<double> values;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    try {
      const double value = std::stod(token);
      if (std::isfinite(value) && value > 0.0) values.push_back(value);
    } catch (...) {
      // ignore malformed tokens
    }
  }
  if (values.empty()) return fallback;
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

}  // namespace

PlannerNode::PlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  LoadParams();

  grid_.reset(new OccupancyGrid2D(params_.grid));
  density_manager_.reset(new DensityTargetManager(params_.density));
  viewpoint_planner_.reset(
      new ViewpointPlanner(params_.viewpoint, params_.path));

  if (!params_.topics.density_cloud.empty()) {
    density_sub_ = nh_.subscribe(params_.topics.density_cloud, 1,
                                 &PlannerNode::DensityCallback, this);
  }
  if (!params_.topics.odom.empty()) {
    odom_sub_ = nh_.subscribe(params_.topics.odom, 5,
                              &PlannerNode::OdomCallback, this);
  }
  if (!params_.topics.scan.empty()) {
    scan_sub_ = nh_.subscribe(params_.topics.scan, 1,
                              &PlannerNode::ScanCallback, this);
  }
  if (!params_.topics.map_cloud.empty()) {
    map_cloud_sub_ = nh_.subscribe(params_.topics.map_cloud, 1,
                                   &PlannerNode::MapCloudCallback, this);
  }
  if (!params_.topics.occupancy_grid.empty()) {
    grid_sub_ = nh_.subscribe(params_.topics.occupancy_grid, 1,
                              &PlannerNode::OccupancyGridCallback, this);
  }
  if (!params_.topics.frontier_cloud.empty()) {
    frontier_sub_ = nh_.subscribe(params_.topics.frontier_cloud, 1,
                                  &PlannerNode::FrontierCloudCallback, this);
  }
  if (!params_.topics.start.empty()) {
    start_sub_ = nh_.subscribe(params_.topics.start, 1,
                               &PlannerNode::StartCallback, this);
  }

  waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      params_.topics.waypoint, 2);
  path_pub_ = nh_.advertise<nav_msgs::Path>(params_.topics.path, 1);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      params_.topics.move_base_goal, 1);
  density_target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "density_targets", 1);
  frontier_target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "frontier_targets", 1);
  viewpoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "viewpoint_markers", 1);
  grid_debug_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_debug", 1);
  status_pub_ = nh_.advertise<std_msgs::String>("status", 1);
  finish_pub_ = nh_.advertise<std_msgs::Bool>("exploration_finished", 1, true);

  const double replan_period = std::max(0.2, params_.path.replan_interval_sec);
  const double waypoint_period =
      1.0 / std::max(1.0, params_.path.waypoint_rate);
  planning_timer_ = nh_.createTimer(ros::Duration(replan_period),
                                    &PlannerNode::PlanningTimerCallback, this);
  waypoint_timer_ = nh_.createTimer(ros::Duration(waypoint_period),
                                    &PlannerNode::WaypointTimerCallback, this);
  started_ = params_.autostart;

  ROS_INFO("[dfp] node ready | density=%s odom=%s grid=%s frontier=%s waypoint=%s",
           params_.topics.density_cloud.c_str(), params_.topics.odom.c_str(),
           params_.topics.occupancy_grid.c_str(),
           params_.topics.frontier_cloud.c_str(),
           params_.topics.waypoint.c_str());
}

void PlannerNode::LoadParams() {
  auto d = [this](const char* name, double& value) {
    pnh_.param(name, value, value);
  };
  auto i = [this](const char* name, int& value) {
    pnh_.param(name, value, value);
  };
  auto b = [this](const char* name, bool& value) {
    pnh_.param(name, value, value);
  };
  auto s = [this](const char* name, std::string& value) {
    pnh_.param(name, value, value);
  };

  s("world_frame", params_.world_frame);
  b("autostart", params_.autostart);
  d("density_stale_sec", params_.density_stale_sec);
  d("log_period_sec", params_.log_period_sec);
  d("initial_waypoint_dist", params_.initial_waypoint_dist);
  i("finish_hysteresis_cycles", params_.finish_hysteresis_cycles);
  b("publish_grid_debug", params_.publish_grid_debug);
  d("grid_debug_half_size", params_.grid_debug_half_size);

  s("topics/density_cloud", params_.topics.density_cloud);
  s("topics/odom", params_.topics.odom);
  s("topics/scan", params_.topics.scan);
  s("topics/map_cloud", params_.topics.map_cloud);
  s("topics/occupancy_grid", params_.topics.occupancy_grid);
  s("topics/frontier_cloud", params_.topics.frontier_cloud);
  s("topics/waypoint", params_.topics.waypoint);
  s("topics/path", params_.topics.path);
  s("topics/move_base_goal", params_.topics.move_base_goal);
  s("topics/start", params_.topics.start);

  d("grid/resolution", params_.grid.resolution);
  i("grid/occupied_threshold", params_.grid.occupied_threshold);
  d("grid/ray_max_range", params_.grid.ray_max_range);
  i("grid/ray_bins", params_.grid.ray_bins);
  d("grid/obstacle_min_height", params_.grid.obstacle_min_height);
  d("grid/obstacle_max_height", params_.grid.obstacle_max_height);
  d("grid/prune_radius", params_.grid.prune_radius);
  i("grid/max_cells", params_.grid.max_cells);

  b("frontier/enable", params_.frontier.enable);
  b("frontier/use_external_cloud", params_.frontier.use_external_cloud);
  i("frontier/min_cluster_size", params_.frontier.min_cluster_size);
  i("frontier/min_unknown_neighbors", params_.frontier.min_unknown_neighbors);
  d("frontier/max_range", params_.frontier.max_range);
  i("frontier/max_targets", params_.frontier.max_targets);
  d("frontier/weight", params_.frontier.weight);
  d("frontier/distance_scale", params_.frontier.distance_scale);
  d("frontier/cluster_cell_size", params_.frontier.cluster_cell_size);

  d("density/anchor_cell_size", params_.density.anchor_cell_size);
  i("density/min_observation_cycles", params_.density.min_observation_cycles);
  i("density/missing_cycles_to_forget", params_.density.missing_cycles_to_forget);
  i("density/abandoned_forget_cycles", params_.density.abandoned_forget_cycles);
  i("density/min_support_points", params_.density.min_support_points);
  i("density/max_targets", params_.density.max_targets);
  d("density/weight", params_.density.weight);
  d("density/distance_scale", params_.density.distance_scale);
  d("density/service_radius", params_.density.service_radius);
  d("density/service_release_ratio", params_.density.service_release_ratio);
  d("density/service_settle_sec", params_.density.service_settle_sec);
  i("density/max_failures", params_.density.max_failures);
  d("density/cooldown_base_sec", params_.density.cooldown_base_sec);
  d("density/cooldown_max_sec", params_.density.cooldown_max_sec);
  d("density/fail_penalty", params_.density.fail_penalty);
  d("density/centroid_smoothing", params_.density.centroid_smoothing);

  std::string radii_text;
  pnh_.param("viewpoint/radii", radii_text, std::string("1.5,3.0"));
  params_.viewpoint.radii =
      ParseRadii(radii_text, params_.viewpoint.radii);
  i("viewpoint/angle_bins", params_.viewpoint.angle_bins);
  d("viewpoint/max_viewpoint_dist", params_.viewpoint.max_viewpoint_dist);
  d("viewpoint/effective_range", params_.viewpoint.effective_range);
  d("viewpoint/max_z_diff", params_.viewpoint.max_z_diff);
  d("viewpoint/min_separation", params_.viewpoint.min_separation);
  i("viewpoint/max_selected", params_.viewpoint.max_selected);
  b("viewpoint/require_los", params_.viewpoint.require_los);
  d("viewpoint/distance_penalty", params_.viewpoint.distance_penalty);
  d("viewpoint/turn_penalty", params_.viewpoint.turn_penalty);
  d("viewpoint/min_gain", params_.viewpoint.min_gain);
  d("viewpoint/height_above_ground", params_.viewpoint.height_above_ground);

  d("path/lookahead_dist", params_.path.lookahead_dist);
  d("path/goal_tolerance", params_.path.goal_tolerance);
  d("path/path_point_spacing", params_.path.path_point_spacing);
  d("path/replan_interval_sec", params_.path.replan_interval_sec);
  d("path/max_replan_interval_sec", params_.path.max_replan_interval_sec);
  d("path/stall_sec", params_.path.stall_sec);
  d("path/stall_distance", params_.path.stall_distance);
  d("path/waypoint_rate", params_.path.waypoint_rate);
  b("path/publish_move_base_goal", params_.path.publish_move_base_goal);
}

void PlannerNode::DensityCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  std::vector<Point3> points;
  if (!ExtractXYZ(*msg, points)) {
    ROS_WARN_THROTTLE(5.0, "[dfp] density cloud has no FLOAT32/64 x,y,z fields");
    return;
  }
  density_points_.swap(points);
  last_density_sec_ = ros::Time::now().toSec();
}

void PlannerNode::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  robot_pos_ = Point3(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  robot_yaw_ = GetYaw(msg->pose.pose.orientation);
  if (!odom_received_) {
    odom_received_ = true;
    last_progress_pos_ = robot_pos_;
    last_progress_sec_ = ros::Time::now().toSec();
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != params_.world_frame) {
      ROS_WARN_ONCE("[dfp] odom frame is '%s' but density/frontier maps use "
                    "'%s'. msp treats both as the same global frame; add a "
                    "static map->odom TF if they actually differ.",
                    msg->header.frame_id.c_str(), params_.world_frame.c_str());
    }
  }
}

void PlannerNode::ScanCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (external_grid_received_ || !odom_received_) return;
  const double now = ros::Time::now().toSec();
  if (last_scan_process_sec_ >= 0.0 && now - last_scan_process_sec_ < 0.2) return;
  last_scan_process_sec_ = now;
  std::vector<Point3> cloud;
  if (!ExtractXYZ(*msg, cloud)) return;
  grid_->UpdateFromScan(cloud, robot_pos_);
}

void PlannerNode::MapCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (external_grid_received_ || !odom_received_) return;
  const double now = ros::Time::now().toSec();
  if (last_map_process_sec_ >= 0.0 && now - last_map_process_sec_ < 1.0) return;
  last_map_process_sec_ = now;
  std::vector<Point3> cloud;
  if (!ExtractXYZ(*msg, cloud)) return;
  grid_->UpdateFromMapCloud(cloud, robot_pos_);
}

void PlannerNode::OccupancyGridCallback(
    const nav_msgs::OccupancyGridConstPtr& msg) {
  external_grid_received_ = true;
  grid_->UpdateFromOccupancyGrid(*msg);
}

void PlannerNode::FrontierCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  std::vector<Point3> points;
  if (!ExtractXYZ(*msg, points)) return;
  frontier_points_.swap(points);
  last_frontier_sec_ = ros::Time::now().toSec();
}

void PlannerNode::StartCallback(const std_msgs::BoolConstPtr& msg) {
  started_ = msg->data;
  if (started_ && finished_) {
    finished_ = false;
    finish_published_ = false;
    empty_target_cycles_ = 0;
    replan_requested_ = true;
    ROS_INFO("[dfp] restarted by %s", params_.topics.start.c_str());
  }
}

void PlannerNode::PublishWaypoint(const Point3& point) {
  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = params_.world_frame;
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = point.x;
  waypoint.point.y = point.y;
  waypoint.point.z = point.z;
  waypoint_pub_.publish(waypoint);
}

void PlannerNode::PublishInitialWaypoint() {
  const Point3 goal(
      robot_pos_.x + std::cos(robot_yaw_) * params_.initial_waypoint_dist,
      robot_pos_.y + std::sin(robot_yaw_) * params_.initial_waypoint_dist,
      robot_pos_.z);
  PublishWaypoint(goal);
}

void PlannerNode::PublishPath(const std::vector<Point3>& path) {
  nav_msgs::Path msg;
  msg.header.frame_id = params_.world_frame;
  msg.header.stamp = ros::Time::now();
  msg.poses.reserve(path.size());
  for (const auto& p : path) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = p.z;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  path_pub_.publish(msg);

  if (params_.path.publish_move_base_goal && !path.empty()) {
    geometry_msgs::PoseStamped goal;
    goal.header = msg.header;
    goal.pose.position.x = path.back().x;
    goal.pose.position.y = path.back().y;
    goal.pose.position.z = path.back().z;
    goal.pose.orientation.w = 1.0;
    goal_pub_.publish(goal);
  }
}

void PlannerNode::PublishVisualization(const std::vector<Target>& targets,
                                       const PlanResult& plan) {
  const ros::Time stamp = ros::Time::now();
  const double now = stamp.toSec();

  std::vector<std::pair<Point3, float>> density_vis;
  density_vis.reserve(density_manager_->Records().size());
  for (const auto& entry : density_manager_->Records()) {
    const DensityRecord& record = entry.second;
    float state = 0.0f;  // active
    if (record.abandoned) state = 2.0f;
    else if (now < record.cooldown_until_sec) state = 1.0f;
    else if (record.observed_cycles < params_.density.min_observation_cycles) {
      state = 3.0f;
    }
    density_vis.emplace_back(record.position, state);
  }
  density_target_pub_.publish(
      MakeXYZICloud(density_vis, params_.world_frame, stamp));

  std::vector<std::pair<Point3, float>> frontier_vis;
  for (const auto& target : targets) {
    if (target.type != TargetType::kFrontier) continue;
    frontier_vis.emplace_back(target.position,
                              static_cast<float>(target.weight));
  }
  frontier_target_pub_.publish(
      MakeXYZICloud(frontier_vis, params_.world_frame, stamp));

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker clear;
  clear.header.frame_id = params_.world_frame;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int marker_id = 0;
  for (const auto& viewpoint : plan.viewpoints) {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = params_.world_frame;
    sphere.header.stamp = stamp;
    sphere.ns = "viewpoints";
    sphere.id = marker_id++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x = viewpoint.position.x;
    sphere.pose.position.y = viewpoint.position.y;
    sphere.pose.position.z = viewpoint.position.z;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.45;
    sphere.scale.y = 0.45;
    sphere.scale.z = 0.45;
    sphere.color.r = 0.1f;
    sphere.color.g = 0.8f;
    sphere.color.b = 0.9f;
    sphere.color.a = 0.9f;
    markers.markers.push_back(sphere);
  }

  visualization_msgs::Marker line;
  line.header.frame_id = params_.world_frame;
  line.header.stamp = stamp;
  line.ns = "viewpoint_path";
  line.id = marker_id++;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;
  line.scale.x = 0.12;
  line.color.r = 1.0f;
  line.color.g = 0.6f;
  line.color.b = 0.1f;
  line.color.a = 0.9f;
  line.pose.orientation.w = 1.0;
  for (const auto& p : plan.path) {
    geometry_msgs::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    line.points.push_back(point);
  }
  markers.markers.push_back(line);
  viewpoint_pub_.publish(markers);

  if (params_.publish_grid_debug) {
    nav_msgs::OccupancyGrid grid_msg = grid_->ToRosMessage(
        params_.world_frame, robot_pos_, params_.grid_debug_half_size);
    grid_msg.header.stamp = stamp;
    grid_debug_pub_.publish(grid_msg);
  }
}

void PlannerNode::PublishStatus(const std::vector<Target>& targets,
                                const PlanResult& plan, double now_sec) {
  int density_count = 0;
  int frontier_count = 0;
  int abandoned_count = 0;
  for (const auto& target : targets) {
    if (target.type == TargetType::kDensity) ++density_count;
    else ++frontier_count;
  }
  for (const auto& entry : density_manager_->Records()) {
    if (entry.second.abandoned) ++abandoned_count;
  }

  std::ostringstream oss;
  oss << "cycle=" << plan_cycle_
      << " density_raw=" << density_manager_->RawPointCount()
      << " density_unique=" << density_manager_->UniqueTargetCount()
      << " active_density=" << density_count
      << " abandoned=" << abandoned_count
      << " frontier=" << frontier_count
      << " viewpoints=" << plan.viewpoints.size()
      << " path_m=" << std::fixed << std::setprecision(1)
      << PathLength(plan.path)
      << " grid_cells=" << grid_->CellCount()
      << " grid=" << (external_grid_received_ ? "external" : "raycast")
      << " finished=" << (finished_ ? 1 : 0);
  std_msgs::String msg;
  msg.data = oss.str();
  status_pub_.publish(msg);
  if (last_status_sec_ < 0.0 ||
      now_sec - last_status_sec_ >= params_.log_period_sec) {
    ROS_INFO("[dfp] %s", msg.data.c_str());
    last_status_sec_ = now_sec;
  }
}

void PlannerNode::SetFinished(bool finished) {
  finished_ = finished;
  if (!finished) {
    finish_published_ = false;
    return;
  }
  current_path_.clear();
  planned_density_ids_.clear();
  PublishWaypoint(robot_pos_);  // stop in place, same as tare's reset behavior
  if (!finish_published_) {
    std_msgs::Bool msg;
    msg.data = true;
    finish_pub_.publish(msg);
    finish_published_ = true;
  }
  ROS_INFO("[dfp] exploration finished: no active density targets or frontiers");
}

void PlannerNode::PlanningTimerCallback(const ros::TimerEvent& /*event*/) {
  const double now = ros::Time::now().toSec();
  if (!started_ || !odom_received_) return;

  if (last_prune_sec_ < 0.0 || now - last_prune_sec_ > 5.0) {
    grid_->Prune(robot_pos_);
    last_prune_sec_ = now;
  }
  if (last_density_sec_ < 0.0) {
    ROS_WARN_THROTTLE(params_.log_period_sec,
                      "[dfp] waiting for the first density map on %s; sending "
                      "cold-start waypoint",
                      params_.topics.density_cloud.c_str());
    return;
  }
  // msp only re-publishes when its map changes. A quiet map is normal, so keep
  // planning from the last frame instead of freezing (the give-up policy still
  // handles never-disappearing points).
  if (now - last_density_sec_ > params_.density_stale_sec) {
    ROS_WARN_THROTTLE(params_.log_period_sec,
                      "[dfp] density map is stale (%.1fs); reusing last frame",
                      now - last_density_sec_);
  }

  density_manager_->Update(density_points_, now, robot_pos_, *grid_);
  std::vector<Target> density_targets =
      density_manager_->GetActiveTargets(robot_pos_, now);

  std::vector<Target> frontier_targets;
  if (params_.frontier.enable) {
    const bool external_available =
        params_.frontier.use_external_cloud && !frontier_points_.empty() &&
        now - last_frontier_sec_ < params_.density_stale_sec;
    if (external_available) {
      frontier_targets = ClusterPoints(
          frontier_points_, params_.frontier.cluster_cell_size,
          params_.frontier.min_cluster_size, TargetType::kFrontier,
          kFrontierIdSalt);
      std::sort(frontier_targets.begin(), frontier_targets.end(),
                [](const Target& a, const Target& b) {
                  if (a.support != b.support) return a.support > b.support;
                  return a.id < b.id;
                });
      if (static_cast<int>(frontier_targets.size()) >
          params_.frontier.max_targets) {
        frontier_targets.resize(params_.frontier.max_targets);
      }
    } else {
      frontier_targets = grid_->ExtractFrontiers(robot_pos_, params_.frontier);
    }
    for (auto& target : frontier_targets) {
      const double dist = Dist2D(robot_pos_, target.position);
      target.weight = params_.frontier.weight * std::min(target.support, 30) *
                      std::exp(-dist / params_.frontier.distance_scale);
    }
  }

  std::vector<Target> targets = density_targets;
  targets.insert(targets.end(), frontier_targets.begin(), frontier_targets.end());

  if (Dist2D(robot_pos_, last_progress_pos_) > params_.path.stall_distance) {
    last_progress_pos_ = robot_pos_;
    last_progress_sec_ = now;
  } else if (!current_path_.empty() &&
             now - last_progress_sec_ > params_.path.stall_sec) {
    density_manager_->NoteStall(planned_density_ids_, now);
    current_path_.clear();
    replan_requested_ = true;
    last_progress_sec_ = now;
    ROS_WARN("[dfp] no progress for %.1fs, penalizing %zu planned targets",
             params_.path.stall_sec, planned_density_ids_.size());
  }

  if (targets.empty()) {
    ++empty_target_cycles_;
    if (!finished_ && empty_target_cycles_ >= params_.finish_hysteresis_cycles) {
      SetFinished(true);
    }
    PublishVisualization(targets, PlanResult());
    return;
  }
  empty_target_cycles_ = 0;
  if (finished_) {
    PublishVisualization(targets, PlanResult());
    return;  // latched finish; /start_exploration restarts it
  }

  const bool arrived =
      current_path_.empty() || path_index_ >= current_path_.size() ||
      Dist2D(robot_pos_, current_path_.back()) <= params_.path.goal_tolerance;
  const bool periodic =
      now - last_plan_sec_ >= params_.path.max_replan_interval_sec;
  if (!replan_requested_ && !arrived && !periodic) return;

  PlanResult plan =
      viewpoint_planner_->Plan(robot_pos_, robot_yaw_, targets, *grid_);
  if (plan.valid) {
    current_path_ = plan.path;
    path_index_ = 0;
    planned_density_ids_ = plan.planned_density_ids;
    PublishPath(plan.path);
  } else {
    current_path_.clear();
    // A target that admits no reachable viewpoint is just as un-coverable as
    // one that survives a re-scan. Penalize it, otherwise the planner can stall
    // forever on a target behind a wall.
    std::vector<uint64_t> unreachable_ids;
    for (const auto& target : targets) {
      if (target.type == TargetType::kDensity) {
        unreachable_ids.push_back(target.id);
      }
    }
    density_manager_->NoteStall(unreachable_ids, now);
    ROS_WARN_THROTTLE(params_.log_period_sec,
                      "[dfp] no viewpoint could be generated for %zu targets, "
                      "penalizing %zu density targets",
                      targets.size(), unreachable_ids.size());
  }
  replan_requested_ = false;
  last_plan_sec_ = now;
  ++plan_cycle_;
  PublishVisualization(targets, plan);
  PublishStatus(targets, plan, now);
}

void PlannerNode::WaypointTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!started_ || !odom_received_ || finished_) return;

  if (last_density_sec_ < 0.0) {
    PublishInitialWaypoint();
    return;
  }
  if (current_path_.empty()) return;

  while (path_index_ < current_path_.size() &&
         Dist2D(robot_pos_, current_path_[path_index_]) <=
             params_.path.goal_tolerance) {
    ++path_index_;
  }
  if (path_index_ >= current_path_.size()) {
    replan_requested_ = true;
    return;
  }

  // Geometric lookahead (not arc length): the waypoint always stays ahead of
  // the robot, so the local_planner never receives a goal behind itself.
  Point3 waypoint = current_path_.back();
  for (size_t i = path_index_; i < current_path_.size(); ++i) {
    if (Dist2D(robot_pos_, current_path_[i]) >= params_.path.lookahead_dist) {
      waypoint = current_path_[i];
      break;
    }
  }
  PublishWaypoint(waypoint);
}

}  // namespace density_frontier_planner
