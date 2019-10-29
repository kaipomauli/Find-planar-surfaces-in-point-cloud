#pragma once
#include <write_pcd.h>


void write_pcd(float *arrayXdata, float *arrayYdata, float *arrayZdata,int width,int height,char* outputPCD,bool binaryPCD)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width = width;
	cloud.height = height;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	for (size_t i = 0; i < cloud.size(); i++)
	{
		cloud.points[i].x = arrayXdata[i];
		cloud.points[i].y = arrayYdata[i];
		cloud.points[i].z = arrayZdata[i];
		if (abs(cloud.points[i].x) >= 0.99*FLT_MAX)
			cloud.points[i].x = NAN;
		if (abs(cloud.points[i].y) >= 0.99*FLT_MAX)
			cloud.points[i].y = NAN;
		if (abs(cloud.points[i].z) >= 0.99*FLT_MAX)
			cloud.points[i].z = NAN;
		
	}
	std::clock_t start = std::clock();

	pcl::io::savePCDFile(outputPCD, cloud,binaryPCD);
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	safeCout("Writing Binary PCD file took " << duration << " seconds." << std::endl);
	//start = std::clock();
	//safeCout("Writing ASCII PCD file, please wait..." << std::endl);
	//pcl::io::savePCDFileASCII("debug_pcd.pcd", cloud);
	//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	//safeCout("Writing ASCII PCD file took " << duration << " seconds." << std::endl);
	
}
void write_RGB_pcd(float *arrayXdata, float *arrayYdata, float *arrayZdata, uint8_t *RGBR, uint8_t *RGBG, uint8_t *RGBB, int width, int height, char* outputPCD, bool binaryPCD)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// Fill in the cloud data
	cloud.width = width;
	cloud.height = height;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	for (size_t i = 0; i < cloud.size(); i++)
	{
		cloud.points[i].x = arrayXdata[i];
		cloud.points[i].y = arrayYdata[i];
		cloud.points[i].z = arrayZdata[i];
		if (abs(cloud.points[i].x) >= 0.99*FLT_MAX)
			cloud.points[i].x = NAN;
		if (abs(cloud.points[i].y) >= 0.99*FLT_MAX)
			cloud.points[i].y = NAN;
		if (abs(cloud.points[i].z) >= 0.99*FLT_MAX)
			cloud.points[i].z = NAN;
		cloud.points[i].r = RGBR[i];
		cloud.points[i].g = RGBG[i];
		cloud.points[i].b = RGBB[i];
	}
	std::clock_t start = std::clock();

	pcl::io::savePCDFile(outputPCD, cloud, binaryPCD);
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	safeCout("Writing Color PCD file took " << duration << " seconds." << std::endl);
	//start = std::clock();
	//safeCout("Writing ASCII PCD file, please wait..." << std::endl);
	//pcl::io::savePCDFileASCII("debug_pcd.pcd", cloud);
	//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	//safeCout("Writing ASCII PCD file took " << duration << " seconds." << std::endl);

}
void write_color_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, char* outputPCD, bool binaryPCD)
{
	
	std::clock_t start = std::clock();

	pcl::io::savePCDFile(outputPCD, *cloud_color, binaryPCD);
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	safeCout("Writing Color PCD file took " << duration << " seconds." << std::endl);
	//start = std::clock();
	//safeCout("Writing ASCII PCD file, please wait..." << std::endl);
	//pcl::io::savePCDFileASCII("debug_pcd.pcd", cloud);
	//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	//safeCout("Writing ASCII PCD file took " << duration << " seconds." << std::endl);

}