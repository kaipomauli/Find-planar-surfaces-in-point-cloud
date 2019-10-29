#include <colorSubset.h>


void extractIndicesFromVOI(pcl::PointIndicesPtr indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CubicVOI voi)
{
	pcl::PointIndicesPtr tempIndices(new pcl::PointIndices);
	//
	//pcl::PCLPointCloud2 outputCloud2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *tempCloud);
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGBToSave(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::PCLPointCloud2::Ptr cloud2_ptr_filtered(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud2_ptr(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *cloud2_ptr);
	safeCout("Input cloud size " << cloud2_ptr->height*cloud2_ptr->width<< " points." << std::endl);

	pcl::CropBox<pcl::PCLPointCloud2> boxCrop(true);
	
	Eigen::Vector4f minPoint;
	minPoint[0] = -float(voi.sizeX())/2;  // define minimum point x
	minPoint[1] = -float(voi.sizeY()) / 2;  // define minimum point y
	minPoint[2] = -float(voi.sizeZ()) / 2; // define minimum point z
	minPoint[3] = 0.0f;
	Eigen::Vector4f maxPoint;
	maxPoint[0] = float(voi.sizeX() / 2);  // define max point x
	maxPoint[1] = float(voi.sizeY() / 2);  // define max point y
	maxPoint[2] = float(voi.sizeZ() / 2);   // define max point z
	maxPoint[3] = 0.0f;
	boxCrop.setMin(minPoint);
	boxCrop.setMax(maxPoint);
	Eigen::Matrix3f m;
	m = voi.rot().toRotationMatrix();
	Eigen::Vector3f eulerAngles = m.eulerAngles(0, 1, 2);
	boxCrop.setInputCloud(cloud2_ptr);
	boxCrop.setTranslation(voi.trans());
	boxCrop.setRotation(eulerAngles);
	
	std::clock_t startbxcrop = std::clock();
	boxCrop.filter(indices->indices);
	double duration = (std::clock() - startbxcrop) / (double)CLOCKS_PER_SEC;
	std::cerr << "Boxcropping took " << duration << " sec." << std::endl;
	std::cerr << "VOI PointCloud has: " << indices->indices.size() << " data points." << std::endl;
	boxCrop.filter(*cloud2_ptr_filtered);
	pcl::fromPCLPointCloud2(*cloud2_ptr_filtered, *cloudRGBToSave);
	write_color_pcd(cloudRGBToSave, "reducedCloud.pcd", true);
	/*

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(tempCloud);
	pass.setIndices(tempIndices);
	std::cerr << "Filtering Missing data. Before passthrough, VOI PointCloud has: " << tempCloud->points.size() << " data points." << std::endl;

	pass.setFilterFieldName("z");
	pass.setFilterLimits(-100, 2000);
	pass.filter(indices->indices);
	std::cerr << "After passthrough, VOI PointCloud has: " << indices->indices.size() << " data points." << std::endl;
	*/
	
	return;
}

//Modify pointcloud to one without the recent Plane candidate
void filterFromVOI(pcl::PointIndicesPtr indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CubicVOI voi)
{
	pcl::PCLPointCloud2::Ptr cloud2_ptr(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *cloud2_ptr);
	std::cerr << "Input PointCloud has: " << cloud->points.size() << " data points." << std::endl;
	safeCout("Input VOI size " << indices->indices.size() << " points." << std::endl);
	
	pcl::CropBox<pcl::PCLPointCloud2> boxCrop(true);

	Eigen::Vector4f minPoint;
	minPoint[0] = -float(voi.sizeX()) / 2;  // define minimum point x
	minPoint[1] = -float(voi.sizeY()) / 2;  // define minimum point y
	minPoint[2] = -float(voi.sizeZ()) / 2; // define minimum point z
	minPoint[3] = 0.0f;
	Eigen::Vector4f maxPoint;
	maxPoint[0] = float(voi.sizeX() / 2);  // define max point x
	maxPoint[1] = float(voi.sizeY() / 2);  // define max point y
	maxPoint[2] = float(voi.sizeZ() / 2);   // define max point z
	maxPoint[3] = 0.0f;
	boxCrop.setMin(minPoint);
	boxCrop.setMax(maxPoint);
	boxCrop.setInputCloud(cloud2_ptr);
	
	/*Eigen::Matrix3f m;
	m = voi.rot().toRotationMatrix();
	Eigen::Vector3f eulerAngles = m.eulerAngles(0, 1, 2);
	
	boxCrop.setTranslation(voi.trans()); //Here we are setting the translation and rotation of the box with respect to world.
	Eigen::Vector3f newRot(eulerAngles[2], eulerAngles[1], eulerAngles[0] );
	boxCrop.setRotation(newRot);
	*/
	Eigen::Translation3f boxTrans(voi.trans());
	Eigen::Affine3f boxInWorld = boxTrans*voi.rot();
	Eigen::Affine3f worldInBox = boxInWorld.inverse();
	boxCrop.setTransform(worldInBox); //This setTransform does not apply on the box but on the point cloud fed to the box.Thus we need to feed with the location of the world in the Box coordinate system.
	boxCrop.setNegative(true);
	std::clock_t startbxcrop = std::clock();
	
	boxCrop.filter(*cloud2_ptr);
	pcl::fromPCLPointCloud2(*cloud2_ptr, *cloud);
	
	
	//pcl::PointIndices::Ptr remIndicesPtr(new pcl::PointIndices);
	//boxCrop.getRemovedIndices(*remIndicesPtr);
	//removeIndices(cloud, remIndicesPtr); 

	double duration = (std::clock() - startbxcrop) / (double)CLOCKS_PER_SEC;
	std::cerr << "Filtering the points inside the box took... " << duration << " sec." << std::endl;
	
	std::cerr << "New PointCloud has: " << cloud->points.size() << " data points." << std::endl;
	return;
}
//Filter point cloud from input indices
void removeIndices(pcl::PointIndices::Ptr InputIndices, pcl::PointIndices::Ptr remIndices)
{
	
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	/*
	extract.setIndices(remIndices);
	extract.setNegative(true);
	extract.filter(*cloud);
	pcl::copyPointCloud(*cloud,*cloud_color_ptr);

*/
	return;
}

 void colorSubset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, segObject objectToColor)
{
			
	for each (int i in objectToColor.indices->indices) {
		color_cloud->points[i].r = objectToColor.color->red();
		color_cloud->points[i].g = objectToColor.color->green();
		color_cloud->points[i].b = objectToColor.color->blue();
		
	}


	
}
 void colorAllSubset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, std::vector<segObject>& objectToColor)
 {

	 for each (segObject i in objectToColor) {
		 for each (int j in i.indices->indices) {
			 color_cloud->points[j].r = i.color->red();
			 color_cloud->points[j].g = i.color->green();
			 color_cloud->points[j].b = i.color->blue();

		 }
	 }


 }