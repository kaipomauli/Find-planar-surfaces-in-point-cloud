#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <conio.h>
#include <ctime>
#include <CubicVOI.h>


class vizualizeCloud {

private:

	static void viewerPsycho(pcl::visualization::PCLVisualizer &viewer);
	static boost::function1<void, pcl::visualization::PCLVisualizer> viewerOneOff(pcl::visualization::PCLVisualizer &viewer);
	
	
public:
	
	int vizualize(char* pcdFileName, bool Color_Cloud);
	void vizualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr,CubicVOI cubeVOI);
	void vizualizeCloud::vizualizePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, CubicVOI cubeVOI,Eigen::Affine3f planeCentroidRot);
	void vizualizeCloud::vizualizePlaneBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, CubicVOI cubeVOI,CubicVOI boundingBox, Eigen::Affine3f planeCentroidRot);
	void vizualizeCloud::vizualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CubicVOI cubeVOI);

};


