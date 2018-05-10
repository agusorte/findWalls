
#include <iostream>
#include "findWalls.h"


using namespace std;

/*
 *  main.cpp 
 *
 *  Created on: Mar, 2018
 *  Author: Agustin Ortega
 *  Email: aortega.jim@gmail.com
 */

int main(int argc, char** argv) {
  
  string cloud_str;
  int cluster_size = 30;

  if(argc == 2){
    cloud_str = argv[1];

  }
  else if(argc == 3){
  	cloud_str = argv[1];
    cluster_size = atoi(argv[2]); 

  }
  else{
  	 cout<<"try ./find_walls <point_cloud> <cluster_size>"<<endl;
     cout<<"try ./find_walls point_cloud.pcd 50"<<endl;
  }

  cout<<"Opening file "<<cloud_str<<endl;
  cout<<"cluster size "<<cluster_size<<endl;


  // we create the object 
  project_canvas::findWalls find_walls(cloud_str);
  
  // put the size we select by planes
  find_walls.setClusterSize(cluster_size);

  // compute results and print/render
  find_walls.compute();
  

  return 0;
}