#include "tos_supervoxels_alg.h"

TosSupervoxelsAlgorithm::TosSupervoxelsAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TosSupervoxelsAlgorithm::~TosSupervoxelsAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TosSupervoxelsAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TosSupervoxelsAlgorithm Public API
void TosSupervoxelsAlgorithm::init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud,
          tos_supervoxels_parameters &opt)
{
  this->obj_segment.init(input_cloud,opt);
}

void TosSupervoxelsAlgorithm::init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud)
{
  this->obj_segment.init(input_cloud);
}

/*void TosSupervoxelsAlgorithm::init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud,
          iri_tos_supervoxels::parameters &opt)
{
  this->obj_segment.init(input_cloud);

  this->obj_segment.set_disable_transform(opt.disable_transform);
  this->obj_segment.set_voxel_resolution(opt.voxel_resolution);
  this->obj_segment.set_seed_resolution(opt.seed_resolution);
  this->obj_segment.set_color_importance(opt.color_importance);
  this->obj_segment.set_spatial_importance(opt.spatial_importance);
  this->obj_segment.set_normal_importance(opt.normal_importance);  

  this->obj_segment.set_concavity_tolerance_threshold(opt.concavity_tolerance_threshold);
  this->obj_segment.set_smoothness_threshold(opt.smoothness_threshold);
  this->obj_segment.set_min_segment_size(opt.min_segment_size);
  this->obj_segment.set_use_extended_convexity(opt.use_extended_convexity);
  this->obj_segment.set_use_sanity_criterion(opt.use_sanity_criterion);
 
  this->obj_segment.set_zmin(opt.zmin);
  this->obj_segment.set_zmax(opt.zmax);
  this->obj_segment.set_th_points(opt.th_points);
}*/

void TosSupervoxelsAlgorithm::reset()
{
  this->obj_segment.reset();
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr TosSupervoxelsAlgorithm::get_input_cloud()
{
  return this->obj_segment.get_input_cloud();
}


tos_supervoxels_parameters TosSupervoxelsAlgorithm::get_default_parameters()
{
  return this->obj_segment.get_default_parameters();
}

/*iri_tos_supervoxels::parameters TosSupervoxelsAlgorithm::get_default_parameters_msg()
{
  iri_tos_supervoxels::parameters p;
  
  tos_supervoxels_parameters opt = this->obj_segment.get_default_parameters();

  p.disable_transform = opt.disable_transform;
  p.voxel_resolution = opt.voxel_resolution;
  p.seed_resolution = opt.seed_resolution;
  p.color_importance = opt.color_importance;
  p.spatial_importance = opt.spatial_importance;
  p.normal_importance = opt.normal_importance;

  // LCCPSegmentation Stuff
  p.concavity_tolerance_threshold = opt.concavity_tolerance_threshold;
  p.smoothness_threshold = opt.smoothness_threshold;
  p.min_segment_size = opt.min_segment_size; 
  p.use_extended_convexity = opt.use_extended_convexity;
  p.use_sanity_criterion = opt.use_sanity_criterion;

  p.zmin = opt.zmin;//meters
  p.zmax = opt.zmax;//meters

  p.th_points = opt.th_points;

  return p;
}*/


void TosSupervoxelsAlgorithm::segment()
{
  this->obj_segment.segment();
}

std::vector<Object> TosSupervoxelsAlgorithm::get_segmented_objects()
{
  return this->obj_segment.get_segmented_objects();
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > TosSupervoxelsAlgorithm::get_segmented_objects_simple()
{
  return this->obj_segment.get_segmented_objects_simple();
}

void TosSupervoxelsAlgorithm::print_parameters()
{
  this->obj_segment.print_parameters();
}

pcl::PointCloud<pcl::PointXYZL> TosSupervoxelsAlgorithm::get_labeled_voxel_cloud()
{
  return this->obj_segment.get_labeled_voxel_cloud();
}

std::multimap<uint32_t, uint32_t> TosSupervoxelsAlgorithm::get_supervoxel_adjacency()
{
  return this->obj_segment.get_supervoxel_adjacency();
}

std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> TosSupervoxelsAlgorithm::get_supervoxel_clusters()
{
  return this->obj_segment.get_supervoxel_clusters();
}

pcl::PointCloud<pcl::PointNormal> TosSupervoxelsAlgorithm::get_sv_normal_cloud()
{
  return this->obj_segment.get_sv_normal_cloud();
}

void TosSupervoxelsAlgorithm::set_disable_transform(bool disable_transform_in)
{
  this->obj_segment.set_disable_transform(disable_transform_in);
}
    
void TosSupervoxelsAlgorithm::set_voxel_resolution(double voxel_resolution_in)
{
  this->obj_segment.set_voxel_resolution(voxel_resolution_in);
}

void TosSupervoxelsAlgorithm::set_seed_resolution(double seed_resolution_in)
{
  this->obj_segment.set_seed_resolution(seed_resolution_in);
}

void TosSupervoxelsAlgorithm::set_color_importance(double color_importance_in)
{
  this->obj_segment.set_color_importance(color_importance_in);
}

void TosSupervoxelsAlgorithm::set_spatial_importance(double spatial_importance_in)
{
  this->obj_segment.set_spatial_importance(spatial_importance_in);
}

void TosSupervoxelsAlgorithm::set_normal_importance(double normal_importance_in)
{
  this->obj_segment.set_normal_importance(normal_importance_in);
}
    
void TosSupervoxelsAlgorithm::set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in)
{
  this->obj_segment.set_concavity_tolerance_threshold(concavity_tolerance_threshold_in);
}

void TosSupervoxelsAlgorithm::set_smoothness_threshold(double smoothness_threshold_in)
{
  this->obj_segment.set_smoothness_threshold(smoothness_threshold_in);
}

void TosSupervoxelsAlgorithm::set_min_segment_size(int min_segment_size_in)
{
  this->obj_segment.set_min_segment_size(min_segment_size_in);
}

void TosSupervoxelsAlgorithm::set_use_extended_convexity(bool use_extended_convexity_in)
{
  this->obj_segment.set_use_extended_convexity(use_extended_convexity_in);
}

void TosSupervoxelsAlgorithm::set_use_sanity_criterion(bool use_sanity_criterion_in)
{
  this->obj_segment.set_use_sanity_criterion(use_sanity_criterion_in);
}

void TosSupervoxelsAlgorithm::set_zmin(double zmin_in)
{
  this->obj_segment.set_zmin(zmin_in);
}

void TosSupervoxelsAlgorithm::set_zmax(double zmax_in)
{
  this->obj_segment.set_zmax(zmax_in);
}

void TosSupervoxelsAlgorithm::set_th_points(int th_points_in)
{
  this->obj_segment.set_th_points(th_points_in);
}

bool TosSupervoxelsAlgorithm::get_disable_transform()
{
  return this->obj_segment.get_disable_transform();
}

double TosSupervoxelsAlgorithm::get_voxel_resolution()
{
  return this->obj_segment.get_voxel_resolution();
}

double TosSupervoxelsAlgorithm::get_seed_resolution()
{
  return this->get_seed_resolution();
}

double TosSupervoxelsAlgorithm::get_color_importance()
{
  return this->get_color_importance();
}

double TosSupervoxelsAlgorithm::get_spatial_importance()
{
  return this->obj_segment.get_spatial_importance();
}

double TosSupervoxelsAlgorithm::get_normal_importance()
{
  return this->obj_segment.get_normal_importance();
}

double TosSupervoxelsAlgorithm::get_concavity_tolerance_threshold()
{
  return this->obj_segment.get_concavity_tolerance_threshold();
}

double TosSupervoxelsAlgorithm::get_smoothness_threshold()
{
  return this->obj_segment.get_smoothness_threshold();
}

int TosSupervoxelsAlgorithm::get_min_segment_size()
{
  return this->obj_segment.get_min_segment_size();
}

bool TosSupervoxelsAlgorithm::get_use_extended_convexity()
{
  return this->obj_segment.get_use_extended_convexity();
}

bool TosSupervoxelsAlgorithm::get_use_sanity_criterion()
{
  return this->obj_segment.get_use_sanity_criterion();
}

double TosSupervoxelsAlgorithm::get_zmin()
{
  return this->obj_segment.get_zmin();
}

double TosSupervoxelsAlgorithm::get_zmax()
{
  return this->obj_segment.get_zmax();
}

int TosSupervoxelsAlgorithm::get_th_points()
{
  return this->obj_segment.get_th_points();
}

// TosSupervoxelsAlgorithm Public API
