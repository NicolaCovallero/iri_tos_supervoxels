# iri_tos_supervoxels

**ROS package for Tabletop Object Segmentation by Supervoxels convexity**

This package provides a service node which segment the tabletop objects of a point cloud, it can work well in cluttered scenes.

*How it works:*
1 - Tabletop objects detection
2 - Segmentation of tabletop objects by means of [LCCP](http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html) segmentation algorithm. 

