/*--
 *      This node compute the traversability from the polygon_mesh from voxblox
 *      Using opencv tools on gridMap
 *
 *      Subscrib :
 *          voxblox_node/pcl_mesh PolygonMesh from voxblox reconstruction
 *
 *      Publish :
 *          /image_traversability the binary image of Traversability
 *          /free_space_grid_map the finale gridMap with traversability + normals +...
 *
 *      Client :
 *          /Caller: to call the service /voxblox/generate_mesh
 *
 *      Parameters :
 *          verbose: to show steps, time consuming, image processed
 *          grid_map_resolution: resolution of the gridMap (m/pixels)
 *          z_threshold: The value of the threshold on traverasbility altitude
 *          loop: if you want the node to call for another polygonMesh from voxblox when the computation finish
 *
 *      Approach :
 *          1) convert PolygonMesh into GridMap
 *          2) compute normals using opencv tools : gradiant + orientation
 *          3) compute traversability by thresholding point cloud on slope and altitude
 *          4) filtering traversability images
 */
#include <mesh_to_traversability_gridmap/mesh_to_traversability_gridmap.hpp>

namespace mesh_to_traversability {

MeshToGridMapConverter::MeshToGridMapConverter(ros::NodeHandle nh,
                                               ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      grid_map_resolution_(kDefaultGridMapResolution),
      layer_name_(kDefaultLayerName),
      latch_grid_map_pub_(kDefaultLatchGridMapPub),
      verbose_(kDefaultVerbose),
      z_threshold_(kDefaultZThreshold),
      loop_(kDefaultLoop)
{
    // Initial interaction with ROS
    subscribeToTopics();
    advertiseTopics();
    getParametersFromRos();
    // init service client
    caller_ = nh_.serviceClient<std_srvs::Empty>("/ibis/voxblox_node/generate_mesh");
}

void MeshToGridMapConverter::subscribeToTopics() {
    mesh_sub_ = nh_.subscribe("mesh", 10, &MeshToGridMapConverter::meshCallback, this);
}

void MeshToGridMapConverter::advertiseTopics() {
    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/free_space_grid_map", 1, latch_grid_map_pub_);
    image_transport::ImageTransport it(nh_);
    traversability_pub_ = it.advertise("/image_traversability", 1);
}

void MeshToGridMapConverter::getParametersFromRos() {
    nh_private_.param("grid_map_resolution", grid_map_resolution_, grid_map_resolution_);
    nh_private_.param("layer_name", layer_name_, layer_name_);
    nh_private_.param("latch_grid_map_pub", latch_grid_map_pub_, latch_grid_map_pub_);
    nh_private_.param("z_threshold", z_threshold_, z_threshold_);
    nh_private_.param("loop", loop_, loop_);
    nh_private_.param("verbose", verbose_, verbose_);
}

void MeshToGridMapConverter::meshCallback(const pcl_msgs::PolygonMesh& mesh_msg)
{
    if(mesh_msg.cloud.width<10) return; // empty mesh
    if (verbose_)
    {
        ROS_INFO("Mesh received, starting conversion.");
    }
    ros::Time time0, time1, time2;
    double duration;
    time0 = ros::Time::now();

    /// convertion mesh_PCL en grid_map
    // Converting from message to an object
    pcl::PolygonMesh polygon_mesh;
    pcl_conversions::toPCL(mesh_msg, polygon_mesh);
    // Creating the grid map
    grid_map::GridMap map;
    map.setFrameId(mesh_msg.header.frame_id);
    // Creating the converter
    grid_map::GridMapPclConverter grid_map_pcl_converter;
    grid_map_pcl_converter.initializeFromPolygonMesh(polygon_mesh, grid_map_resolution_, map);
    const std::string layer_name(layer_name_);
    grid_map_pcl_converter.addLayerFromPolygonMesh(polygon_mesh, layer_name, map);
    // Printing some debug info about the mesh and the map
    time1 = ros::Time::now();
    duration = time1.toSec() - time0.toSec();
    if (verbose_)
    {
        ROS_INFO_STREAM("Number of polygons: " << polygon_mesh.polygons.size());
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",map.getLength().x(), map.getLength().y(), map.getSize()(0),map.getSize()(1));
        ROS_INFO_STREAM(duration<<"sec");
    }

    /// pre treatment of grid_map
    if(verbose_) ROS_INFO_STREAM("Treatment of grid_map");
    time1 = ros::Time::now();
    map.add("normal_x", 0.0);
    map.add("normal_y", 0.0);
    map.add("normal_z", 0.0);
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        map.getPosition(*it, position);
        if(map.at("elevation", *it) < -0.60)
        {
            map.at("elevation", *it) = -0.60;
        }
    }

    /// Use Opencv to compute normals
    cv::Mat mapImage;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map, "elevation", CV_16UC1, -0.60, z_threshold_, mapImage);
    cv::medianBlur(mapImage,mapImage,3);
    //cv::imshow("mapImage",mapImage);
    // gradiant
    int scale = 1;  int delta = 0;  int ddepth = CV_32F;
    cv::Mat grad_x, grad_y;
    // Gradient X et Y
    cv::Sobel( mapImage, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
    cv::Sobel( mapImage, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
    cv::Mat gradiant = cv::Mat::zeros(mapImage.rows, mapImage.cols, CV_32F);
    cv::magnitude(grad_x,grad_y,gradiant);
    cv::normalize(gradiant, gradiant,0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    int amplify_magnitude = 20;
    for(unsigned int i=0;i<grad_x.rows;i++)
    {
        for(unsigned int j=0;j<grad_y.cols;j++)
        {
            gradiant.at<uchar>(i,j) = gradiant.at<uchar>(i,j)*amplify_magnitude;
        }
    }
    // smoothing of the gradiant
    cv::GaussianBlur(gradiant,gradiant,cv::Size(5,5),0,0);
    //cv::imshow("gradiant",gradiant);
    // orientation
    cv::Mat orientation = cv::Mat::zeros(mapImage.rows, mapImage.cols, CV_32F);
    grad_y.convertTo(grad_y,CV_32F);
    grad_x.convertTo(grad_x,CV_32F);
    cv::phase(grad_x, grad_y, orientation,true);
    cv::normalize(orientation, orientation, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    // create new layers to store the data
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(gradiant, "gradiant", map, 0, 255);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(orientation, "orientation", map, 0, 255);
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        grid_map::Position position;
        map.getPosition(*it, position);
        Eigen::Vector3d normal(sin(map.at("gradiant", *it)/255.0) * sin(map.at("orientation", *it)*2*M_PI/255.0) , sin(map.at("gradiant", *it)/255.0) * cos(map.at("orientation", *it)*2*M_PI/255.0) , 1-map.at("gradiant", *it)/255.0);
        normal.normalize();
        map.at("normal_x", *it) = normal.x();
        map.at("normal_y", *it) = normal.y();
        map.at("normal_z", *it) = normal.z();
    }
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if(verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Computation of traversability
    if(verbose_) ROS_INFO_STREAM("Traversability of grid_map");
    time1 = ros::Time::now();
    map.add("traversability");
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        if(map.at("elevation", *it) > -0.6 && map.at("elevation", *it) < z_threshold_ && map.at("normal_z", *it) > 0.9)
            map.at("traversability", *it) = 255.0;
        else
            map.at("traversability", *it) = 0.0;
    }

    /// compute traversability as an image
    cv::Mat traversability;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "traversability", CV_8UC1, 0, 255, traversability);
    //cv::imshow("traversability_image_raw",traversability);
    // median filter to remove small black pixels
    cv::medianBlur(traversability,traversability,3);
    //cv::imshow("Output median", traversability);
    int element_size = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size + 1, 2*element_size+1 ), cv::Point( element_size, element_size ) );
    cv::morphologyEx( traversability, traversability, cv::MORPH_ERODE, element );
    //cv::imshow("Output erode", traversability);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(traversability, "traversability", map, 0, 255);
    // transform matrix to put into right convention
    cv::Mat traversability_img = cv::Mat(traversability.cols,traversability.rows,CV_8UC1);
    for(unsigned int i = 0; i < traversability.cols;i++)
    {
        for(unsigned int j = 0; j < traversability.rows;j++)
        {
            traversability_img.at<char>(i,j) = traversability.at<char>(j,traversability.cols-i-1);
        }
    }
    //cv::imshow("traversability", traversability);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if(verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Publish grid map
    map.setTimestamp(mesh_msg.header.stamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    grid_map_pub_.publish(message);
    if (verbose_)
    {
        ROS_INFO("Published a grid map message. \n");
    }

    /// publish image traversabiity
    sensor_msgs::ImagePtr msgPublish;
    msgPublish = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_img).toImageMsg();
    traversability_pub_.publish(msgPublish);
    if (verbose_) cv::waitKey(50);

    /// Closing the loop by calling for another mesh
    if(loop_)
    {
        std_srvs::Empty srv;
        caller_.call(srv);
    }
}

}  // namespace mesh_to_grid_map
