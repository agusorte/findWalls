# findWalls


#Overview
	this program find the wall based in region growing approach and compute bounding boxes to compute the corners,
	the dimentions of the wall are equal to the lengh of the bounding box

#Code 

	important files are 
		-findWall.h findWall.cpp 



#Dependences

	- gcc > 5.4
	- make > 3
	-opencv > 3
	-PCL > 1.8
	-boost > 5
	-Eigen > 3

#Building

	$unzip code.zip

	$cd canvas

	$mkdir build

	$cd build

	$cmake ..
	
	make 

#Run
	./find_Walls ../room_scan2.pcd 1000
 
	or

	./find_Walls ../room_scan2.pcd 

#Output

   will be the number of planes, points in each plane and dimentions( mts)
   Example:
   ./find_walls ../room_scan2.pcd 1000

	Opening file ../room_scan2.pcd
	cluster size 1000
	Number of planes is equal to 11
	Cluster 0 with size 45762
	dimentions  hight: 5.27515 mts
	dimentions  widht: 8.71977 mts
	Cluster 1 with size 11112
	dimentions  hight: 5.47792 mts
	dimentions  widht: 9.84019 mts
	Cluster 2 with size 5008
	dimentions  hight: 3.26552 mts
	dimentions  widht: 3.76665 mts
	Cluster 3 with size 2122
	dimentions  hight: 0.654565 mts
	dimentions  widht: 0.632684 mts
	Cluster 4 with size 1940
	dimentions  hight: 6.63023 mts
	dimentions  widht: 3.16047 mts
	Cluster 5 with size 1456
	dimentions  hight: 6.25151 mts
	dimentions  widht: 2.84903 mts
	Cluster 6 with size 1472
	dimentions  hight: 3.46671 mts
	dimentions  widht: 2.74877 mts
	Cluster 7 with size 1174
	dimentions  hight: 1.51538 mts
	dimentions  widht: 0.193601 mts
	Cluster 8 with size 7994
	dimentions  hight: 0.193232 mts
	dimentions  widht: 0.181492 mts
	Cluster 9 with size 2208
	dimentions  hight: 0.0934116 mts
	dimentions  widht: 0.0895963 mts
	Cluster 10 with size 2028
	dimentions  hight: 0.0496079 mts
	dimentions  widht: 0.061831 mts




#Inputs

	there are tow inputs the pointcloud (just pcd format) and cluster size (default 30)

	Suggestion or comment email to Agustin Ortega aortega.jim@gmail.com
