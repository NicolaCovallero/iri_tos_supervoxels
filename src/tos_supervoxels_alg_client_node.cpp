#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tos_supervoxels_alg.h"

// srv
#include "iri_tos_supervoxels/object_segmentation.h"

ros::ServiceClient client;
ros::Publisher pub;
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
    sensor_msgs::PointCloud2 detected_objects_msg;
    pcl::PointCloud<pcl::PointXYZRGB> detected_objects_cloud;
    for (int i = 0; i < srv.response.objects.size(); ++i)
    {
      // convert it to pcl
      pcl::PointCloud<pcl::PointXYZRGB> tmp;
      pcl::fromROSMsg(srv.response.objects[i],tmp);

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
        detected_objects_cloud.points.push_back(temp_point);
      }
    }

    detected_objects_cloud.width = detected_objects_cloud.points.size();
    detected_objects_cloud.height = 1;
    detected_objects_cloud.is_dense = true;

    //std::cout << "detected_objects_cloud.points.size() " << detected_objects_cloud.points.size() << "\n";
    pcl::toROSMsg(detected_objects_cloud,detected_objects_msg);
    //std::cout << "detected_objects_msg.data.size() " << detected_objects_msg.data.size() << "\n";

    detected_objects_msg.header.seq = 1;
    detected_objects_msg.header.frame_id = cloud_msg.header.frame_id;
    detected_objects_msg.header.stamp = ros::Time::now();
    //std::cout << "frame of point cloud: " << detected_objects_msg.header.frame_id << "\n";
    pub.publish(detected_objects_msg);
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
  n.param("input_topic",input_topic,std::string("/camera/depth/points"));

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, callback);

  pub = n.advertise<sensor_msgs::PointCloud2>("/detected_objects/points", 1);

  client = n.serviceClient<iri_tos_supervoxels::object_segmentation>(service_name);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}