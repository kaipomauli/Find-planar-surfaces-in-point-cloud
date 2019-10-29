// main.cpp : Defines the entry point for the console application.
//

#include "write_pcd.h"
#include <pcl/visualization/cloud_viewer.h>
#include "stdafx.h"
#include <iostream>
#include <conio.h>
#include <stdlib.h>
#include "ErrorHandling.h"
#include "ThreadSafePrint.h"
#include <ctime>
#include <fstream>  // std::ifstream
#include <iostream> // std::cout
#include "vizualize.h"
#include <cstdint>
#include <string.h>
#include <CubicVOI.h>
#include <colorSubset.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <colorSubset.h>
#include <segObject.h>
#include <findPlane.h>
#include <limits>


using namespace std;
using namespace icon;



void tryCameraOperation(ErrorCode IconErrorCode)
{
	try
	{
		CHECK_ERROR(IconErrorCode);
	}
	catch (icon_exception &e)
	{
		cerr << e.what() << endl;
		cerr << "Press any key to exit..." << endl;
		_getch();
		exit(-1);
	}
}

int main(int argc, char** argv)
{
		int returnCode;
		std::clock_t start = std::clock();
		if (argc<4)
		{
			safeCout("Not enough arguments to exe file." << endl << "Press a key to exit..." << endl);
			_getch();
			return -1;
		}
		if (strcmp(argv[1], "True") == 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudReduced(new pcl::PointCloud<pcl::PointXYZRGB>);
			Eigen::Affine3f* planePose = new Eigen::Affine3f;
			CubicVOI* planeBB = new CubicVOI;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
			MyColor* planeColor = new MyColor(255, 255, 0);
			//CubicVOI cubeVOI(600, 350, 200, 200, 1000, 410, 1, 0, 0, 0); //enveloppe
			CubicVOI cubeVOI(370, 220, 220, -260, 30, 1170, 1, 0, 0, 0); //Boxes
			vizualizeCloud viz;

			
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *cloudReduced) == -1) //* load the file
			{
				PCL_ERROR("Load PCD: Could not read file");
				return (-1);
			}
			while (cloudReduced->points.size() > 1000)
			{
				findPlaneRedCld(cloudReduced, inliers, planePose, planeBB);

				planeIndices->indices = inliers->indices;
				segObject pointsFromPlane(planeIndices, planeColor);
				colorSubset(cloudReduced, pointsFromPlane);
				viz.vizualizePlaneBox(cloudReduced, cubeVOI, *planeBB, *planePose);
				filterFromVOI(inliers, cloudReduced, *planeBB);
				inliers.reset(new pcl::PointIndices);

			}
			
			return 1;
		}
		bool Color_Cloud = false;
		bool  binaryPCD = false;
		if (strcmp(argv[3], "True") == 0)
			binaryPCD = true;
		if (strcmp(argv[4], "True") == 0)
			Color_Cloud = true;
			
		char* pcdFile=argv[2];
		std::string icon_buffer_fileName = argv[1];

		createConsoleMutex();
		IconBuffer*buffer = IconBuffer::loadBuffer(icon_buffer_fileName.c_str());
		if(buffer==NULL)
		{ 
			safeCout("Failed getting the buffer" << endl << "Press a key to exit..." << endl);
			_getch();
			return -1;
		}
		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		
		safeCout("Managed to get buffer " << duration << " sec." << endl);
		const unsigned int numberOfScans = buffer->getHeight();

		safeCout("numberOfScans " << numberOfScans << endl);

		const DataFormat*format = buffer->getDataFormat();
		const Component*component = format->getNamedComponent("Cir 1");
		const SubComponent*subcomponentZ = component->getNamedSubComponent("Z");
		const SubComponent*subcomponentX = component->getNamedSubComponent("X");
		const SubComponent*subcomponentY = component->getNamedSubComponent("Y");
		const unsigned int numberOfColumns = subcomponentX->getWidth();
		safeCout("numberOfColumns " << numberOfColumns << endl);
		
		const float *Xdata;
		const float *Ydata;
		const float *Zdata;
		
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		

		cloud_color_ptr->width = numberOfColumns;
		cloud_color_ptr->height = numberOfScans;
		cloud_color_ptr->is_dense = false;
		cloud_color_ptr->points.resize(cloud_color_ptr->width*cloud_color_ptr->height);

		vizualizeCloud viz;

	
		std::vector<int> myindices;
		
		
		std::clock_t startNans = std::clock();
		for (unsigned int scan = 0; scan<numberOfScans; scan++)
		{
			buffer->getReadPointer("Cir 1", "X", scan, Xdata);
			buffer->getReadPointer("Cir 1", "Y", scan, Ydata);
			buffer->getReadPointer("Cir 1", "Z", scan, Zdata);
			for (unsigned int col = 0; col < numberOfColumns; col++)
			{
				cloud_color_ptr->points[scan*numberOfColumns + col].x = *(Xdata + col);
				cloud_color_ptr->points[scan*numberOfColumns + col].y = *(Ydata + col);
				cloud_color_ptr->points[scan*numberOfColumns + col].z = *(Zdata + col);
				if (abs(cloud_color_ptr->points[scan*numberOfColumns + col].x) >= 0.99*FLT_MAX)
					//std::cerr << "Big number= " << abs(cloud_color_ptr->points[scan*numberOfColumns + col].x) << std::endl;
					cloud_color_ptr->points[scan*numberOfColumns + col].x = NAN;
				if (abs(cloud_color_ptr->points[scan*numberOfColumns + col].y) >= 0.99*FLT_MAX)
					cloud_color_ptr->points[scan*numberOfColumns + col].y = NAN;
				if (abs(cloud_color_ptr->points[scan*numberOfColumns + col].z) >= 0.99*FLT_MAX)
					cloud_color_ptr->points[scan*numberOfColumns + col].z = NAN;

				
			}
			
		}

		

		//std::replace_if(cloud_color_ptr->points[0], cloud_color_ptr->points[numberOfScans*numberOfColumns], std::bind(std::isgreaterequal<float,float>(), std::placeholders::_1, ), NAN);
		duration = (std::clock() - startNans) / (double)CLOCKS_PER_SEC;
		std::cerr << "Replacing NaNs took " << duration << " sec." << std::endl;

		std::cerr << "Cloud size before removing NaNs " << cloud_color_ptr->size() << " points." << std::endl;
		std::clock_t startRemNans = std::clock();
		pcl::removeNaNFromPointCloud(*cloud_color_ptr, *cloud_color_ptr, myindices);
		duration = (std::clock() - startRemNans) / (double)CLOCKS_PER_SEC;
		std::cerr << "Removing NaNs took " << duration << " sec." << std::endl;
		std::cerr << "Cloud size after removing NaNs " << cloud_color_ptr->size() << " points." << std::endl;

		//CubicVOI cubeVOI(1000, 500,500, -250, 0, 100, 1,0, 0, 0);
		//CubicVOI cubeVOI(600, 350,200, 200, 1000, 410, 1,0, 0, 0);  //Envelopes
		CubicVOI cubeVOI(370, 220, 220, -260, 30, 1170, 1, 0, 0, 0); //Boxes
		
		pcl::PointIndices::Ptr voi_indices(new pcl::PointIndices);
		

		/*pcl::PointIndices::Ptr inputIndices2(new pcl::PointIndices);
		pcl::PointIndices::Ptr voi_indices2(new pcl::PointIndices);
		MyColor* voiColor2 = new MyColor(0, 0, 255);
		*/
		CubicVOI cubeVOI2(1000, 500, 500, 0, 0, 0, 1, 0, 0, 0);
		Eigen::VectorXf  modelcoef;
		//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		MyColor* voiColor =new MyColor(0,255,0);
		MyColor* planeColor = new MyColor(255, 255, 0);
		MyColor* hullColor = new MyColor(255, 0, 0);
		CubicVOI* planeBB=new CubicVOI;
		
		std::clock_t startExtr = std::clock();


		extractIndicesFromVOI(voi_indices, cloud_color_ptr, cubeVOI);
		duration = (std::clock() - startExtr) / (double)CLOCKS_PER_SEC;
		std::cerr << "Extracting voi indices took " << duration << " sec." << std::endl;

		

		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		std::cerr << "Entering findplane after " << duration << " sec." << std::endl;

		while (voi_indices->indices.size() > 1000)
		{
		//extractIndicesFromVOI(inputIndices2, cloud_color_ptr, cubeVOI2);
		//segObject pointsFromVOI2(inputIndices2, voiColor2);
		segObject pointsFromVOI(voi_indices, voiColor);

		pcl::PointIndices::Ptr hullIndicesPtr(new pcl::PointIndices);
		Eigen::Affine3f* planePose=new Eigen::Affine3f;
		//viz.vizualize(cloud_color_ptr, cubeVOI);

		

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			findPlane(cloud_color_ptr, voi_indices, modelcoef, inliers, hullIndicesPtr, planePose, planeBB);
			

			segObject hull(hullIndicesPtr, hullColor);
			colorSubset(cloud_color_ptr, hull);

			
			pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
			planeIndices->indices = inliers->indices;
			segObject pointsFromPlane(planeIndices, planeColor);



			//myObjectsToSeg.push_back(pointsFromVOI);
			//myObjectsToSeg.push_back(pointsFromVOI2);
			colorSubset(cloud_color_ptr, pointsFromVOI);
			colorSubset(cloud_color_ptr, pointsFromPlane);

			//colorAllSubset(cloud_color_ptr, myObjectsToSeg);


			safeCout("Writing pcd file..." << endl);


			write_color_pcd(cloud_color_ptr, pcdFile, binaryPCD);

			viz.vizualizePlaneBox(cloud_color_ptr, cubeVOI, *planeBB, *planePose);
			
			filterFromVOI(voi_indices,cloud_color_ptr, *planeBB);
			
			extractIndicesFromVOI(voi_indices, cloud_color_ptr, cubeVOI);
			
		}
		

		
		/* if(Color_Cloud)
			write_RGB_pcd(arrayOfXData, arrayOfYData, arrayOfZData, RGBDataR, RGBDataG, RGBDataB, numberOfColumns, numberOfScans, pcdFile, binaryPCD);
		else
			write_pcd(arrayOfXData, arrayOfYData, arrayOfZData, numberOfColumns, numberOfScans,pcdFile,binaryPCD); */
	
		delete buffer,voiColor,planeColor,hullColor;

		//returnCode = vizualize(pcdFile,Color_Cloud);
		//viz.vizualize(cloud_color_ptr, cubeVOI);
		
		
		returnCode = 0;
		if (returnCode == 0)
		{
			safeCout("Vizualization succeeded." << endl << "Press a key to exit..." << endl);
			_getch();
			return 0;
		}
		else
		{
			safeCout("Vizualization failed." << endl << "Press a key to exit..." << endl);
			_getch();
			return -1;
		}
		


}