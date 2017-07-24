#include <ros/ros.h>
#include <mesh_to_traversability_gridmap/mesh_to_traversability_gridmap.hpp>

// Standard C++ entry point
int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "mesh_to_traversability_gridmap_node");
  ros::NodeHandle nh; 
  ros::NodeHandle private_nh("~");

  // Creating the object to do the work.
  mesh_to_traversability::MeshToGridMapConverter mesh_to_traversability_gridmap(nh, private_nh);

  // automatic start after 10 sec
  ros::Rate rate(0.1);
  rate.sleep();
  rate.sleep();
  if(mesh_to_traversability_gridmap.loop_)
  {
      std_srvs::Empty srv;
      mesh_to_traversability_gridmap.caller_.call(srv);
  }

  ros::spin();
  return 0;
}
