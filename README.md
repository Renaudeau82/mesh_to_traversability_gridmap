# mesh_to_traversability_gridmap #
ROS package containing 1 node :

*  **mesh_to_traversability_gridmap_node :** 

## Dependences ##

* Opencv, pcl, voxblox

## Usage ##

* **roslaunch mesh_to_traversability_gridmap.launch :** 

Launch rosbag (you have to use yours) 

dense_stereo (you have to use your calibration file .yaml)

mesh_to_traversability_gridmap_node

## Parameters ##

* **grid_map_resolution :** resolution of the gridmap bild from mesh

* **z_threshold :** the value you want to threshold the altitude of the traversability

* **loop :** if false, you need to use service call /voxblox/generate_mesh to trigger the node

