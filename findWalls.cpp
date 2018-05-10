#include "findWalls.h"

/*
 * findWalls.cpp
 *
 *  Created on: Mars, 2018
 *  Author: Agustin Ortega
 *  Email: aortega.jim@gmail.com
 */

namespace project_canvas {

findWalls::findWalls(const string &str_){



  cloud =  PointCloud<pcl::PointXYZ>::Ptr(new PointCloud<pcl::PointXYZ>);

  if ( io::loadPCDFile <pcl::PointXYZ> (str_, *cloud) == -1){
    std::cout << "Cloud reading failed." << std::endl;
    return;
  }

 

}


void findWalls::segmentation(){
  

  //int size_cluster = 50;
  search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (size_cluster);
  normal_estimator.compute (*normals);

  // IndicesPtr indices (new std::vector <int>);
  // PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // pass.filter (*indices);

  RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (size_cluster);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (0.3);

  
  reg.extract (clusters);

 

  // Create the filtering object
  ExtractIndices<PointXYZ> extract;


  boundingBoxes.clear();
  for (size_t i = 0; i < clusters.size(); i++){

    PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<pcl::PointXYZ>::Ptr cloud_bb (new pcl::PointCloud<pcl::PointXYZ>);

    IndicesPtr indx (new std::vector <int>(clusters[i].indices));

  	extract.setInputCloud (cloud);
    extract.setIndices (indx);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    computeBoundingBox(cloud_p, cloud_bb);

    boundingBoxes.push_back(cloud_bb);


  }

  // colored point cloud
  colored_cloud = reg.getColoredCloud ();


}

void findWalls::printResults(){

  cout << "Number of planes is equal to " << boundingBoxes.size () << endl;

  size_t indx = 0;
  for (auto i:boundingBoxes){
  	cout<<"Cluster "<<indx<<" with size "<<clusters[indx].indices.size()<<endl;
  	Eigen::Vector3f p1,p2, p3;

  	p1 = Eigen::Vector3f( i->points[0].x,i->points[0].y, i->points[0].z);
  	p2 = Eigen::Vector3f( i->points[1].x,i->points[1].y, i->points[1].z);
  	p3 = Eigen::Vector3f( i->points[3].x,i->points[3].y, i->points[3].z);

  	cout<<"dimentions  hight: "<<(p1-p2).norm()<<" mts"<<endl;
  	cout<<"dimentions  widht: "<<(p1-p3).norm()<<" mts"<<endl;
  	indx++;
  }

  
  
}


void findWalls::visualize(){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud );
  viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud , rgb, "sample cloud");

  for (auto i:boundingBoxes){
  	viewer->addPointCloud<pcl::PointXYZ>(i, "polygon"+to_string(i)); 
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5, "polygon"+to_string(i),0);

  	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "chessboard"+to_string(i));
  
  	viewer->addPolygon<pcl::PointXYZ>(i, 0.0, 1.0, 0.0,"square"+to_string(i),0);
    viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "square"+to_string(i));
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


void findWalls::compute(){

  // segment point cloud based in region growing approach
  segmentation ();

  // print results
  printResults();

  // visualize results 
  visualize ();

  

}

void findWalls::computeBoundingBox(const PointCloud<PointXYZ>::Ptr& cloud, PointCloud<PointXYZ>::Ptr &  cloudBB){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proj(new PointCloud<pcl::PointXYZ>);
  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  PointIndices::Ptr inliers (new pcl::PointIndices);


  // compute plane

  SACSegmentation<PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.1);


  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0){
  	cerr << "Could not estimate a planar model for the given the pointcloud."<<endl;
    return;
   }


  // project to plane
  ProjectInliers<PointXYZ> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_proj);    


  //convex hull
  PointCloud<PointXYZ>::Ptr cloud_hull (new PointCloud<PointXYZ>);
  ConvexHull<PointXYZ> chull;
  chull.setInputCloud (cloud_proj);
  chull.reconstruct (*cloud_hull);

  //now create a minimum bounding box

  Eigen::Vector3f plane_normal;
  plane_normal.x() = coefficients->values[0];
  plane_normal.y() = coefficients->values[1];
  plane_normal.z() = coefficients->values[2];
  // compute an orthogonal normal to the plane normal
  Eigen::Vector3f v = plane_normal.unitOrthogonal();
  // take the cross product of the two normals to get
  // a thirds normal, on the plane
  Eigen::Vector3f u = plane_normal.cross(v); 
   
  CvMat* points_mat = cvCreateMat(cloud_hull->points.size(), 2, CV_32F );

  vector<cv::Point2f> points; 
  Eigen::Vector3f p0(cloud_hull->points[0].x,
                         cloud_hull->points[0].y,
                         cloud_hull->points[0].z);; 
  for(int ii=0; ii<cloud_hull->points.size(); ii++){

    Eigen::Vector3f p3d(cloud_hull->points[ii].x,
                         cloud_hull->points[ii].y,
                         cloud_hull->points[ii].z);

    // subtract all 3D points with a point in the plane
    // this will move the origin of the 3D coordinate system
    // onto the plane
    p3d = p3d - p0;

    cv::Point2f p2d;
    p2d.x = p3d.dot(u);
    p2d.y = p3d.dot(v);
    points.push_back(p2d); 
  } 

  Eigen::Vector4f  centroid;
  compute3DCentroid (*cloud_proj, centroid);
   

  cv::Mat points_mat2(points); 


  RotatedRect rrect = minAreaRect(points_mat2); 
  
  cv::Point2f rrPts[4];
  rrect.points(rrPts);

  //store the table top bounding points in a vector
  for(size_t ii=0; ii<4; ii++)
  {
	Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0); 
       
    PointCloud<PointXYZ> PointAux;
        
    PointAux.points.resize(1);
    PointAux.points[0].x=pbbx[0];
    PointAux.points[0].y=pbbx[1];
    PointAux.points[0].z=pbbx[2];
        
    cloudBB->push_back (PointAux.points[0]);
      
  } 


}


}
