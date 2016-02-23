#include "tos_supervoxels_alg_node.h"

TosSupervoxelsAlgNode::TosSupervoxelsAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TosSupervoxelsAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10;//in [Hz]

  this->public_node_handle_.param("disable_transform",alg_.params.disable_transform,DISABLE_TRANSFORM);
  this->public_node_handle_.param("voxel_resolution",alg_.params.voxel_resolution,VOXEL_RESOLUTION);
  this->public_node_handle_.param("seed_resolution",alg_.params.seed_resolution,SEED_RESOLUTION);
  this->public_node_handle_.param("color_importance",alg_.params.color_importance,COLOR_IMPORTANCE);
  this->public_node_handle_.param("spatial_importance",alg_.params.spatial_importance,SPATIAL_IMPORTANCE);
  this->public_node_handle_.param("normal_importance",alg_.params.normal_importance,NORMAL_IMPORTANCE);

  this->public_node_handle_.param("concavity_tolerance_threshold",alg_.params.concavity_tolerance_threshold,CONCAVITY_TOLERANCE_THRESHOLD);
  this->public_node_handle_.param("smoothness_threshold",alg_.params.smoothness_threshold,SMOOTHNESS_THRESHOLD);
  this->public_node_handle_.param("min_segment_size",alg_.params.min_segment_size,MIN_SEGMENT_SIZE);
  this->public_node_handle_.param("use_extended_convexity",alg_.params.use_extended_convexity,USE_EXTENDED_CONVEXITY);
  this->public_node_handle_.param("use_sanity_criterion",alg_.params.use_sanity_criterion,USE_SANITY_CRITERION);

  this->public_node_handle_.param("zmin",alg_.params.zmin,ZMIN);
  this->public_node_handle_.param("zmax",alg_.params.zmax,ZMAX);
  this->public_node_handle_.param("th_points",alg_.params.th_points,TH_POINTS);

  std::string service_name;
  this->public_node_handle_.param("service_name",service_name,std::string("object_segmentation"));

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->object_segmentation_server_ = this->public_node_handle_.advertiseService(service_name, &TosSupervoxelsAlgNode::object_segmentationCallback, this);
  pthread_mutex_init(&this->object_segmentation_mutex_,NULL);

  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TosSupervoxelsAlgNode::~TosSupervoxelsAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->object_segmentation_mutex_);
}

void TosSupervoxelsAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool TosSupervoxelsAlgNode::object_segmentationCallback(iri_tos_supervoxels::object_segmentation::Request &req, iri_tos_supervoxels::object_segmentation::Response &res)
{
  ROS_INFO("TosSupervoxelsAlgNode::object_segmentationCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->object_segmentation_mutex_enter();

  ROS_INFO("TosSupervoxelsAlgNode::object_segmentationCallback: Processing New Request!");

  // convert the point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg(req.point_cloud,cloud);

  // initialize the class 
  alg_.init(cloud,alg_.params);

  // print parameters
  alg_.print_parameters();

  // segment tabletop objects
  alg_.segment();
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > objects;
  objects = alg_.get_segmented_objects_simple();
  
  // free the memory of the algorithm
  alg_.reset();

  //create msg
  std::vector<sensor_msgs::PointCloud2> objects_msg;
  for (uint i = 0; i < objects.size(); ++i)
  {
    sensor_msgs::PointCloud2 object_msg;
    pcl::toROSMsg(objects[i], object_msg);
    object_msg.header.seq = i;
    object_msg.header.frame_id = req.point_cloud.header.frame_id;
    object_msg.header.stamp = ros::Time::now();
    objects_msg.push_back(object_msg);
  }
  res.objects = objects_msg; 

  //unlock previously blocked shared variables
  this->object_segmentation_mutex_exit();
  this->alg_.unlock();

  return true;

  return true;
}

void TosSupervoxelsAlgNode::object_segmentation_mutex_enter(void)
{
  pthread_mutex_lock(&this->object_segmentation_mutex_);
}

void TosSupervoxelsAlgNode::object_segmentation_mutex_exit(void)
{
  pthread_mutex_unlock(&this->object_segmentation_mutex_);
}


/*  [action callbacks] */

/*  [action requests] */

void TosSupervoxelsAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TosSupervoxelsAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TosSupervoxelsAlgNode>(argc, argv, "tos_supervoxels_alg_node");
}
