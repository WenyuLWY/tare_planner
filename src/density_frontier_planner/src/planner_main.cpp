#include <ros/ros.h>

#include "density_frontier_planner/planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "density_frontier_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  density_frontier_planner::PlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
