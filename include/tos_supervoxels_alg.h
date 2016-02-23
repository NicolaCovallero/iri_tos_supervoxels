// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _tos_supervoxels_alg_h_
#define _tos_supervoxels_alg_h_

#include <iri_tos_supervoxels/TosSupervoxelsConfig.h>
#include "tos_supervoxels.h"
//include tos_supervoxels_alg main library

// -------- Default Parameters --------
// supervoxels parameters 
const bool DISABLE_TRANSFORM = true; /**< default value of disable_transform for supervoxel algorithm*/
const double VOXEL_RESOLUTION = 0.0075f;/**< default value of voxel_resolution for supervoxel algorithm*/
const double SEED_RESOLUTION = 0.03f; /**< default value of seed_resolution for supervoxel algorithm*/
const double COLOR_IMPORTANCE = 0.0f;/**< default value of color_importance for supervoxel algorithm*/
const double SPATIAL_IMPORTANCE = 1.0f;/**< default value of spatial_importance for supervoxel algorithm*/
const double NORMAL_IMPORTANCE = 4.0f;/**< default value of normal_importance for supervoxel algorithm*/

// LCCPSegmentation parameters
const double CONCAVITY_TOLERANCE_THRESHOLD = 10;/**< default value of concavity_tolerance_threshold for lccp algorithm*/
const double SMOOTHNESS_THRESHOLD = 0.1f;/**< default value of smoothness_threshold for lccp algorithm*/
const int MIN_SEGMENT_SIZE = 3;/**< default value of min_segment_size for lccp algorithm*/
const bool USE_EXTENDED_CONVEXITY = false;/**< default value of use_extended_convexity for lccp algorithm*/
const bool USE_SANITY_CRITERION = true;/**< default value of use_sanity_criterion for lccp algorithm*/

// Others parameters
const double ZMIN = 0.02;/**<  Default value of the minimum distance for object detection on the table - used inside detectedObjectsTable() */
const double ZMAX = 2.; /**<  Default value of the maxmimum distance for object detection on the table - used inside detectedObjectsTable() */
const int TH_POINTS = 400; /**< Default value of the threshold of minimum points required to consider a cluster as valid */

const bool USE_PARAMETERS = false;
//-------------------

/**
 * \brief IRI ROS Specific Driver Class - Tabletop Object Segmentation by Supervoxels convexity
 *
 * Class to segmented tabletop objects, the table has to be
 * the bigger plane in the scene. 
 */
class TosSupervoxelsAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the TosSupervoxelsConfig. All driver implementations
    * will then use the same variable type Config.
    */
    pthread_mutex_t access_;    

    // private attributes and methods
    /**
    * \brief tos_supervoxels object
    */
    tos_supervoxels obj_segment;

  public:

    /** \brief algorithm parameters
    */
    tos_supervoxels_parameters params;

   /**
    * \brief define config type
    *
    * Define a Config type with the TosSupervoxelsConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_tos_supervoxels::TosSupervoxelsConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    TosSupervoxelsAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { pthread_mutex_lock(&this->access_); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { pthread_mutex_unlock(&this->access_); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) 
    { 
      if(pthread_mutex_trylock(&this->access_)==0)
        return true;
      else
        return false;
    };

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(Config& config, uint32_t level=0);

    // here define all tos_supervoxels_alg interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~TosSupervoxelsAlgorithm(void);

    /*! \brief Class initializer
    *
    * \param input_cloud input cloud to segment 
    * \param opt parameters for the algorithm 
    */
    void init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud,
              tos_supervoxels_parameters &opt);

    /*! \brief Class initializer, with default parameters
    *
    * \param input_cloud input cloud to segment 
    */
    void init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud);

    /*! \brief reset all the publis members 
    * reset all the public members.
    */
    void reset();

    /*! \brief returns input cloud
    */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_input_cloud();

    /*! \brief get the default parameters of the algorithm
    */  
    tos_supervoxels_parameters get_default_parameters(); 

    /*! \brief Detect and segment the objects on the table
    */
    void segment();

    /*! \brief Get the segmented objects as a vector of the variable Object
    */
    std::vector<Object> get_segmented_objects();

    /*! \brief Get the segmented objects as a vector of point clouds
    */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > get_segmented_objects_simple();

    /*! \brief Print the parameters of the algorithm in the shell
    */
    void print_parameters();

    /*! \brief returns labeld voxel cloud
    */
    pcl::PointCloud<pcl::PointXYZL> get_labeled_voxel_cloud();

    /*! \brief returns supervoxel_adjacency map
    */
    std::multimap<uint32_t, uint32_t> get_supervoxel_adjacency();

    /*! \brief returns supervoxel_clusters
    */
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> get_supervoxel_clusters();

    /*! \brief returns normals point cloud of the supervoxels
    */
    pcl::PointCloud<pcl::PointNormal> get_sv_normal_cloud();

    // functions to set and get the parameters of the algorithm

    void set_disable_transform(bool disable_transform_in);
    void set_voxel_resolution(double voxel_resolution_in);
    void set_seed_resolution(double seed_resolution_in);
    void set_color_importance(double color_importance_in);
    void set_spatial_importance(double spatial_importance_in);
    void set_normal_importance(double normal_importance_in);
    
    void set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in);
    void set_smoothness_threshold(double smoothness_threshold_in);
    void set_min_segment_size(int min_segment_size_in);
    void set_use_extended_convexity(bool use_extended_convexity_in);
    void set_use_sanity_criterion(bool use_sanity_criterion_in);

    void set_zmin(double zmin_in);
    void set_zmax(double zmax_in);
    void set_th_points(int th_points_in);

    bool get_disable_transform();
    double get_voxel_resolution();
    double get_seed_resolution();
    double get_color_importance();
    double get_spatial_importance();
    double get_normal_importance();
    
    double get_concavity_tolerance_threshold();
    double get_smoothness_threshold();
    int get_min_segment_size();
    bool get_use_extended_convexity();
    bool get_use_sanity_criterion();

    double get_zmin();
    double get_zmax();
    int get_th_points();
};

#endif
