#pragma once
#include <vizualize.h>
int user_data;

void vizualizeCloud::vizualizePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, CubicVOI cubeVOI, Eigen::Affine3f planeCentroidRot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 0.5, 1.0);
	bool drawnCube = viewer->addCube(cubeVOI.trans(), cubeVOI.rot(), cubeVOI.sizeX(), cubeVOI.sizeY(), cubeVOI.sizeZ(), "myVoi");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "myVoi");
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer->addSphere(o, 0.25, "sphere", 0);
	viewer->addCoordinateSystem(500.0);
	viewer->addCoordinateSystem(200, planeCentroidRot);
	
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_color_ptr);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

void vizualizeCloud::vizualizePlaneBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, CubicVOI cubeVOI,CubicVOI boundingBox, Eigen::Affine3f planeCentroidRot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 0.5, 1.0);
	bool drawnCube = viewer->addCube(cubeVOI.trans(), cubeVOI.rot(), cubeVOI.sizeX(), cubeVOI.sizeY(), cubeVOI.sizeZ(), "myVoi");
	bool drawnBB = viewer->addCube(boundingBox.trans(), boundingBox.rot(), boundingBox.sizeX(), boundingBox.sizeY(), boundingBox.sizeZ(), "myBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "myVoi");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "myBB");
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer->addSphere(o, 0.25, "sphere", 0);
	viewer->addCoordinateSystem(500.0);
	viewer->addCoordinateSystem(200, planeCentroidRot);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_color_ptr);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

void vizualizeCloud::vizualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CubicVOI cubeVOI)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 0.5, 1.0);
	bool drawnCube = viewer->addCube(cubeVOI.trans(), cubeVOI.rot(), cubeVOI.sizeX(), cubeVOI.sizeY(), cubeVOI.sizeZ(), "myVoi");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "myVoi");
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer->addSphere(o, 0.25, "sphere", 0);
	viewer->addCoordinateSystem(100.0);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color_ptr);
	viewer->addPointCloud<pcl::PointXYZ>(cloud);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

void vizualizeCloud::vizualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, CubicVOI cubeVOI)
{
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 0.5, 1.0);
	bool drawnCube = viewer->addCube(cubeVOI.trans(), cubeVOI.rot(), cubeVOI.sizeX(), cubeVOI.sizeY(), cubeVOI.sizeZ(), "myVoi");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "myVoi");
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer->addSphere(o, 0.25, "sphere", 0);
	viewer->addCoordinateSystem(100.0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_color_ptr);
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
}
int vizualizeCloud::vizualize(char* pcdFileName, bool Color_Cloud)
{
	if (Color_Cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::clock_t start = std::clock();

		pcl::io::loadPCDFile(pcdFileName, *cloud);
		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		std::cout << "Loading PCD file took " << duration << " seconds." << std::endl;

		pcl::visualization::CloudViewer viewer("Cloud Viewer");

		//blocks until the cloud is actually rendered
		viewer.showCloud(cloud);

		//use the following functions to get access to the underlying more advanced/powerful
		//PCLVisualizer

		//This will only get called once
		viewer.runOnVisualizationThreadOnce(vizualizeCloud::viewerOneOff);

		//This will get called once per visualization iteration
		viewer.runOnVisualizationThread(vizualizeCloud::viewerPsycho);
		while (!viewer.wasStopped())
		{
			//you can also do cool processing here
			//FIXME: Note that this is running in a separate thread from viewerPsycho
			//and you should guard against race conditions yourself...
			user_data++;
		}
		return 0;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		std::clock_t start = std::clock();

		pcl::io::loadPCDFile(pcdFileName, *cloud);
		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		std::cout << "Loading PCD file took " << duration << " seconds." << std::endl;

		pcl::visualization::CloudViewer viewer("Cloud Viewer");

		//blocks until the cloud is actually rendered
		viewer.showCloud(cloud);

		//use the following functions to get access to the underlying more advanced/powerful
		//PCLVisualizer



		//This will only get called once
		viewer.runOnVisualizationThreadOnce(vizualizeCloud::viewerOneOff);

		//This will get called once per visualization iteration
		viewer.runOnVisualizationThread(vizualizeCloud::viewerPsycho);
		while (!viewer.wasStopped())
		{
			//you can also do cool processing here
			//FIXME: Note that this is running in a separate thread from viewerPsycho
			//and you should guard against race conditions yourself...
			user_data++;
		}
		return 0;
	}
}

boost::function1<void, pcl::visualization::PCLVisualizer> vizualizeCloud::viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{

	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "viewerOneOff" << std::endl << "Continuing..." << std::endl;
	viewer.addCoordinateSystem(100.0);
	
	

	
	//viewer.setRepresentationToWireframeForAllActors();
	//_getch();
	return 0;
}

void vizualizeCloud::viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
	static unsigned count = 0;


	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}
