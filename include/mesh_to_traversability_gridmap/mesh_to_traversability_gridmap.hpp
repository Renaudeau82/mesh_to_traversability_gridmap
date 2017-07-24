#ifndef MESH_TO_GRID_MAP_LOOP_H
#define MESH_TO_GRID_MAP_LOOP_H

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_msgs/PolygonMesh.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <std_srvs/Empty.h>

namespace mesh_to_traversability {

constexpr double kDefaultGridMapResolution = 0.05;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = false;
constexpr bool kDefaultLoop = false;
static double kDefaultZThreshold = 10.0;

class MeshToGridMapConverter {

public:
  MeshToGridMapConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);

  // Service client
  ros::ServiceClient caller_;

  bool loop_;

private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Datacallback
  void meshCallback(const pcl_msgs::PolygonMesh& mesh);

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber mesh_sub_;

  // Publishers
  ros::Publisher grid_map_pub_;
  image_transport::Publisher traversability_pub_;

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;

  // Control Parameters
  bool latch_grid_map_pub_;
  bool verbose_;
  double z_threshold_;

};

} // namespace mesh_to_grid_map

#endif //MESH_TO_GRID_MAP_SAVE_H
