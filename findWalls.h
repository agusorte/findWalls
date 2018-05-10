/*
 * findWalls.h
 *
 *  Created on: Mar, 2018
 *  Author: Agustin Ortega
 *  Email: aortega.jim@gmail.com
 */

#pragma once
// c++

//PCL
#include <pcl/point_types.h>

//filters
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include "pcl/filters/project_inliers.h"
//visualization 
#include <pcl/visualization/pcl_visualizer.h>

//segmentation
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree_flann.h"


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//convexhull
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"

// opencv

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>



namespace project_canvas {

using namespace pcl;
using namespace std;
using namespace cv;

class findWalls{

public:

  /*!
  * construtor
  */

  findWalls(const string& str_);

   /*!
  * compute planes and segmentation, and corners
  */
  void compute();

   /*!
  * compute planes and segmentation, and corners
  */
  void setClusterSize(int size_){
  	size_cluster = size_;
  }

 
private:

  //! original pointcloud
  PointCloud<PointXYZ>::Ptr cloud;

  //! bounding boxes or corners
  vector<PointCloud<PointXYZ>::Ptr> boundingBoxes;

  //! colored point cloud
  PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;

  //! cluster or planes

  vector <PointIndices> clusters;

  // ! size of the cluster
  int size_cluster;
  // ! segmentation of planes based in region growing
  void segmentation();
  // ! compute the bounding box of each plane
  void computeBoundingBox(const PointCloud<PointXYZ>::Ptr &  cloud, PointCloud<PointXYZ>::Ptr &  cloudBB);
  // ! visualize results
  void visualize();
  // ! print cluster and plane length.
  void printResults();

};


}