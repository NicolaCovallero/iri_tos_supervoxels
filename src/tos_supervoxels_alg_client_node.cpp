#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tabletop_object_detection_alg.h"

// srv
#include "iri_tabletop_object_detection/iri_tabletop_object_detection_service.h"

// parameters msg
#include "iri_tabletop_object_detection/parameters.h"


iri_tabletop_object_detection::parameters opt;
bool use_parameters;

ros::ServiceClient client;
ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2 cloud_msg)
{
  std::cout << "Received point cloud\n";
  iri_tabletop_object_detection::iri_tabletop_object_detection_service srv;
  srv.request.point_cloud = cloud_msg;
  srv.request.parameters = opt;
  srv.request.use_parameters = use_parameters;

  if(client.call(srv))
  {
    //creating new point cloud for debugging
    sensor_msgs::PointCloud2 detected_objects_msg;
    pcl::PointCloud<pcl::PointXYZRGB> detected_objects_cloud;
    for (int i = 0; i < srv.response.objects.points.size(); ++i)
    {
      // convert it to pcl
      pcl::PointCloud<pcl::PointXYZRGB> tmp;
      pcl::fromROSMsg(srv.response.objects.points[i],tmp);

      //we now construct manually the point cloud
      //choose a random color for the object
      float r,g,b;
      r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      for (int h = 0; h < tmp.points.size(); ++h)
      {
        pcl::PointXYZRGB temp_point;
        temp_point.x = tmp.points.at(h).x;
        temp_point.y = tmp.points.at(h).y;
        temp_point.z = tmp.points.at(h).z;
        temp_point.r = r*255;
        temp_point.g = g*255;
        temp_point.b = b*255;
        detected_objects_cloud.points.push_back(temp_point);
      }
    }

    detected_objects_cloud.width = detected_objects_cloud.points.size();
    detected_objects_cloud.height = 1;
    detected_objects_cloud.is_dense = true;

    std::cout << "detected_objects_cloud.points.size() " << detected_objects_cloud.points.size() << "\n";
    pcl::toROSMsg(detected_objects_cloud,detected_objects_msg);
    std::cout << "detected_objects_msg.data.size() " << detected_objects_msg.data.size() << "\n";

    detected_objects_msg.header.seq = 1;
    detected_objects_msg.header.frame_id = cloud_msg.header.frame_id;
    detected_objects_msg.header.stamp = ros::Time::now();
    std::cout << "frame of point cloud: " << detected_objects_msg.header.frame_id << "\n";
    pub.publish(detected_objects_msg);
  }
  else
    ROS_ERROR("Failed to call service /iri_tabletop_object_detection_alg/detect_tabletop_objects");

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tabletop_object_detection_client_node");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(1);

  //supervoxels parameters
  bool disable_transform;
  double voxel_resolution;
  double seed_resolution;
  double color_importance;
  double spatial_importance;
  double normal_importance;

  // LCCPSegmentation parameters
  double concavity_tolerance_threshold;
  double smoothness_threshold;
  int min_segment_size;
  bool use_extended_convexity;
  bool use_sanity_criterion;
      
  // Others parameters
  double zmin;
  double zmax;
  int th_points; 

  //parsing the input

  // if the use_parameters variable is set to false the algorithm will works with the default values,
  // that are the ones defined in its header file and should be the same of the ones
  // specified in this file.
  n.param("use_parameters",use_parameters,USE_PARAMETERS);

  n.param("disable_transform",disable_transform,DISABLE_TRANSFORM);
  n.param("voxel_resolution",voxel_resolution,VOXEL_RESOLUTION);
  n.param("seed_resolution",seed_resolution,SEED_RESOLUTION);
  n.param("color_importance",color_importance,COLOR_IMPORTANCE);
  n.param("spatial_importance",spatial_importance,SPATIAL_IMPORTANCE);
  n.param("normal_importance",normal_importance,NORMAL_IMPORTANCE);

  n.param("concavity_tolerance_threshold",concavity_tolerance_threshold,CONCAVITY_TOLERANCE_THRESHOLD);
  n.param("smoothness_threshold",smoothness_threshold,SMOOTHNESS_THRESHOLD);
  n.param("min_segment_size",min_segment_size,MIN_SEGMENT_SIZE);
  n.param("use_extended_convexity",use_extended_convexity,USE_EXTENDED_CONVEXITY);
  n.param("use_sanity_criterion",use_sanity_criterion,USE_SANITY_CRITERION);

  n.param("zmin",zmin,ZMIN);
  n.param("zmax",zmax,ZMAX);
  n.param("th_points",th_points,TH_POINTS);

  // building the parameters message
  opt.disable_transform = disable_transform;
  opt.voxel_resolution = voxel_resolution;
  opt.seed_resolution = seed_resolution;
  opt.color_importance = color_importance;
  opt.spatial_importance = spatial_importance;
  opt.normal_importance = normal_importance;
  opt.concavity_tolerance_threshold = concavity_tolerance_threshold;
  opt.smoothness_threshold = smoothness_threshold;
  opt.min_segment_size = min_segment_size;
  opt.use_extended_convexity = use_extended_convexity;
  opt.use_sanity_criterion = use_sanity_criterion;
  opt.zmin = zmin;
  opt.zmax =  zmax;
  opt.th_points = th_points; 

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);

  pub = n.advertise<sensor_msgs::PointCloud2>("/detected_objects/points", 1);

  client = n.serviceClient<iri_tabletop_object_detection::iri_tabletop_object_detection_service>("/iri_tabletop_object_detection_alg/detect_tabletop_objects");

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}