#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tos_supervoxels_alg.h"

// srv
#include "iri_tos_supervoxels/object_segmentation.h"

// msg
#include "iri_tos_supervoxels/segmented_objects.h"
#include "iri_tos_supervoxels/plane_coefficients.h"

ros::ServiceClient client;
ros::Publisher pub,pub_segmented_objs;
std::string service_name;

void callback(const sensor_msgs::PointCloud2 cloud_msg)
{
  std::cout << "Received point cloud\n";
  iri_tos_supervoxels::object_segmentation srv;
  srv.request.point_cloud = cloud_msg;

  // call the service to segment the table otp objects
  if(client.call(srv))
  {
    //creating new point cloud for debugging with random color for each segmented object
    sensor_msgs::PointCloud2 segmented_objects_msg;
    pcl::PointCloud<pcl::PointXYZRGB> segmented_objects_cloud;

    //iri_tos_supervoxels::segmented_objects segmented_objects_msg;

    for (int i = 0; i < srv.response.objects.objects.size(); ++i)
    {
      // convert it to pcl
      pcl::PointCloud<pcl::PointXYZRGB> tmp;
      pcl::fromROSMsg(srv.response.objects.objects[i],tmp);

      //we now construct manually the point cloud
      //choose a random color for each segmented object
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
        segmented_objects_cloud.points.push_back(temp_point);
      }

      //segmented_objects_msg.objects.push_back(srv.response.objects[i]);
    }

    segmented_objects_cloud.width = segmented_objects_cloud.points.size();
    segmented_objects_cloud.height = 1;
    segmented_objects_cloud.is_dense = true;

    //std::cout << "segmented_objects_cloud.points.size() " << segmented_objects_cloud.points.size() << "\n";
    pcl::toROSMsg(segmented_objects_cloud,segmented_objects_msg);
    //std::cout << "segmented_objects_msg.data.size() " << segmented_objects_msg.data.size() << "\n";

    segmented_objects_msg.header.seq = 1;
    segmented_objects_msg.header.frame_id = cloud_msg.header.frame_id;
    segmented_objects_msg.header.stamp = ros::Time::now();
    //std::cout << "frame of point cloud: " << segmented_objects_msg.header.frame_id << "\n";
    pub.publish(segmented_objects_msg);

    pub_segmented_objs.publish(srv.response.objects);
  }
  else
    ROS_ERROR("Failed to call service %s",service_name.c_str());

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tos_supervoxels_client_node");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(1);

  std::string input_topic;
  n.param("service_name",service_name,std::string("/iri_tos_supervoxels_alg/object_segmentation"));
  n.param("input_topic",input_topic,std::string("/camera/depth_registered/points"));

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, callback);

  pub = n.advertise<sensor_msgs::PointCloud2>("/segmented_objects/points", 1);
  pub_segmented_objs = n.advertise<iri_tos_supervoxels::segmented_objects>("/segmented_objects",1);

  client = n.serviceClient<iri_tos_supervoxels::object_segmentation>(service_name);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}