#include <findPlane.h>
void findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, pcl::PointIndices::Ptr voi_Indices, Eigen::VectorXf  modelcoef, pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr hullinliers, Eigen::Affine3f* planePose, CubicVOI*boundingBox) {



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hullcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr hullcloudGrey(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*cloud_color_ptr, *cloud);






	//SEGMENTATION USING PCL::RANDOMSAMPLECONSENSUS

	// Create a shared plane model pointer directly
	//	pcl::SampleConsensusModelCone<pcl::PointXYZ,pcl::PointXYZ>::Ptr model (new	pcl::SampleConsensusModelCone<pcl::PointXYZ,pcl::PointXYZ>(cloudFromVOi));
		//pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr plane_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_color_ptr));
	pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr plane_model(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(cloud_color_ptr));
	plane_model->setIndices(voi_Indices->indices);
	plane_model->setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
	float epsAngle = 15.0f / 180 * M_PI;
	plane_model->setEpsAngle(epsAngle);




	Eigen::Vector4f centroid;
	//model->setMinMaxOpeningAngle(80, 120);
	//plane_model->selectWithinDistance(modelcoef, 2, inliers->indices);
	pcl::PolygonMesh concaveHullMesh;
	std::vector<pcl::Vertices> concavehullVertices;
	pcl::PointIndices hullIndices;
	// Create the RANSAC object
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac(plane_model, 1.5);



	//SEGMENTATION USING pcl::SACSegmentation (NOT WORKING WHEN TRIED)

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setIndices(voi_Indices);
	seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
	seg.setEpsAngle(15.0f / 180 * M_PI);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(5);
	seg.setMaxIterations(100);
	seg.setInputCloud(cloud);

	bool result = false;

	std::cerr << "Starting to calculate plane model." << std::endl;
	std::clock_t start = std::clock();
	seg.segment(*inliers, *coefficients);
	
	//perform the segmenation step

	if (inliers != NULL)
		result = true;




	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	std::cerr << "Calculating plane model took " << duration << " sec." << std::endl;
	if (result)
	{

		Eigen::Vector3f zvector(0.0f, 0.0f, 1.0f);
		Eigen::Vector3f planevector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		float angle_rad = acos(zvector.dot(planevector));
		float angle_degrees = angle_rad * 180 / M_PI;
		std::cerr << "Plane angle: " << angle_degrees << " degrees." << std::endl;
		//Reduce point cloud to biggest cluster
		// Creating the KdTree object for the search method of the extraction
		std::cerr << "Creating kdTree..." << std::endl;
		std::clock_t startkdTree = std::clock();
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud(cloud);
		duration = (std::clock() - startkdTree) / (double)CLOCKS_PER_SEC;

		std::cerr << "Creating kdTree took " << duration << " sec." << std::endl;

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(2); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(inliers->indices.size());
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.setIndices(inliers);
		std::cerr << "Extracting clusters..." << std::endl;
		std::clock_t startClus = std::clock();
		ec.extract(cluster_indices);
		duration = (std::clock() - startClus) / (double)CLOCKS_PER_SEC;

		std::cerr << "Extracting clusters took " << duration << " sec." << std::endl;
		//inliers = NULL;
		inliers->header = cluster_indices[0].header;
		inliers->indices = cluster_indices[0].indices;


		//REDUCE POINT CLOUD TO FOUND PLANE POINTS

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		std::cerr << "Extracting cloud from biggest cluster..." << std::endl;
		std::clock_t startCloudExtr = std::clock();
		extract.filter(*cloud);
		duration = (std::clock() - startCloudExtr) / (double)CLOCKS_PER_SEC;

		std::cerr << "Extracting cloud took " << duration << " sec." << std::endl;
		

		//COMPUTE POINT CLOUD CENTROID
		
		std::cerr << "Computing cloud centroid..." << std::endl;
		std::clock_t startCentroid = std::clock();

		pcl::compute3DCentroid(*cloud, centroid);

		duration = (std::clock() - startCentroid) / (double)CLOCKS_PER_SEC;

		std::cerr << "Computing centroid took " << duration << " sec." << std::endl;


		//COMPUTE PLANE TRANSFORMATION

		int method = 4;

		std::cerr << "Computing plane transformation with method " <<method << "."<< std::endl;
		std::clock_t startPlaneTransf = std::clock();

		transf planT = getPlaneTransf(cloud, method, centroid, coefficients);

		duration = (std::clock() - startPlaneTransf) / (double)CLOCKS_PER_SEC;
		std::cerr << "Computing plane transformation took " << duration << " sec." << std::endl;




		*planePose = planT.trans*planT.rot;
		boundingBox->setRot(planT.rot);
		Eigen::Vector3f transl(planT.trans.x(), planT.trans.y(), planT.trans.z());
		boundingBox->setTrans(transl);
		//boundingBox->setRot(quat);
		//boundingBox->setTrans(poseBB);
		boundingBox->setSizeX(planT.sizeX);
		boundingBox->setSizeY(planT.sizeY);
		boundingBox->setSizeZ(planT.sizeZ);
		std::cerr << "BoundingBox Size: " << boundingBox->sizeX() << " mm x " << boundingBox->sizeY()<< " mm. " << std::endl;





		std::cerr << "Plane coefficients: Normal X " << coefficients->values[0] << " Normal Y " << coefficients->values[1] << " Normal Z " << coefficients->values[2] << " Constant: " << coefficients->values[3] << std::endl;
		std::cerr << "Plane inliers: " << inliers->indices.size() << " data points." << std::endl;
		
		std::cerr << "Concave Hull: " << hullIndices.indices.size() << " points." << std::endl;
		std::cerr << "Centroid: X " << centroid[0] << " Y " << centroid[1] << " Z " << centroid[2] << std::endl;

	}
	else
		std::cerr << "Failed to calculate plane model." << std::endl;

	if (inliers == NULL)
	{
		PCL_ERROR("Could not estimate a cone model for the given dataset.");
		return;
	}






}
void findPlaneRedCld(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudReduced, pcl::PointIndices::Ptr inliers, Eigen::Affine3f* planePose, CubicVOI*boundingBox) {



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	

	pcl::copyPointCloud(*cloudReduced, *cloud);

	
	//SEGMENTATION USING pcl::SACSegmentation 

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
	seg.setEpsAngle(15.0f / 180 * M_PI);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.7);
	seg.setMaxIterations(100);
	seg.setInputCloud(cloud);

	bool result = false;

	std::cerr << "Starting to calculate plane model." << std::endl;
	std::clock_t start = std::clock();
	seg.segment(*inliers, *coefficients);

	//perform the segmenation step

	if (inliers != NULL)
		result = true;


	bool sketchPlaneDistances = true;

	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	std::cerr << "Calculating plane model took " << duration << " sec." << std::endl;
	if (result)
	{
		std::vector<float> histogram;
		if (sketchPlaneDistances)
		{
			
			for (auto iter = inliers->indices.begin(); iter != inliers->indices.end(); ++iter)
			{
				Eigen::Vector3f pointP(cloudReduced->points[*iter].x, cloudReduced->points[*iter].y, cloudReduced->points[*iter].z);
				float A =coefficients->values[0];
				float B= coefficients->values[1];
				float C= coefficients->values[2];
				float D= coefficients->values[3];
				float x1 = pointP[0];
				float y1 = pointP[1];
				float z1 = pointP[2];

				float signedDistance = (A*x1 + B*y1 + C*z1 + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));  //Distance of PointP from the plane
				histogram.push_back(*iter);
				histogram.push_back(signedDistance);
			}
		}

		Eigen::Vector3f zvector(0.0f, 0.0f, 1.0f);
		Eigen::Vector3f planevector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		float angle_rad = acos(zvector.dot(planevector));
		float angle_degrees = angle_rad * 180 / M_PI;
		std::cerr << "Plane angle: " << angle_degrees << " degrees." << std::endl;
		//Reduce point cloud to biggest cluster
		// Creating the KdTree object for the search method of the extraction
		std::cerr << "Creating kdTree..." << std::endl;
		std::clock_t startkdTree = std::clock();
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud(cloud);
		duration = (std::clock() - startkdTree) / (double)CLOCKS_PER_SEC;

		std::cerr << "Creating kdTree took " << duration << " sec." << std::endl;

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(2); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(inliers->indices.size());
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.setIndices(inliers);
		std::cerr << "Extracting clusters..." << std::endl;
		std::clock_t startClus = std::clock();
		ec.extract(cluster_indices);
		duration = (std::clock() - startClus) / (double)CLOCKS_PER_SEC;

		std::cerr << "Extracting clusters took " << duration << " sec." << std::endl;
		//inliers = NULL;
		inliers->header = cluster_indices[0].header;
		inliers->indices = cluster_indices[0].indices;


		//REDUCE POINT CLOUD TO FOUND PLANE POINTS

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		std::cerr << "Extracting cloud from biggest cluster..." << std::endl;
		std::clock_t startCloudExtr = std::clock();
		extract.filter(*cloud);
		duration = (std::clock() - startCloudExtr) / (double)CLOCKS_PER_SEC;

		std::cerr << "Extracting cloud took " << duration << " sec." << std::endl;


		//COMPUTE PLANE TRANSFORMATION

		int method = 4;
		Eigen::Vector4f centroid;
		std::cerr << "Computing plane transformation with method " << method << "." << std::endl;
		std::clock_t startPlaneTransf = std::clock();

		transf planT = getPlaneTransf(cloud, method, centroid, coefficients);

		duration = (std::clock() - startPlaneTransf) / (double)CLOCKS_PER_SEC;
		std::cerr << "Computing plane transformation took " << duration << " sec." << std::endl;




		*planePose = planT.trans*planT.rot;
		boundingBox->setRot(planT.rot);
		Eigen::Vector3f transl(planT.trans.x(), planT.trans.y(), planT.trans.z());
		boundingBox->setTrans(transl);
		//boundingBox->setRot(quat);
		//boundingBox->setTrans(poseBB);
		boundingBox->setSizeX(planT.sizeX);
		boundingBox->setSizeY(planT.sizeY);
		boundingBox->setSizeZ(planT.sizeZ);
		std::cerr << "BoundingBox Size: " << boundingBox->sizeX() << " mm x " << boundingBox->sizeY() << " mm. " << std::endl;
		std::cerr << "Plane coefficients: Normal X " << coefficients->values[0] << " Normal Y " << coefficients->values[1] << " Normal Z " << coefficients->values[2] << " Constant: " << coefficients->values[3] << std::endl;
		std::cerr << "Plane inliers: " << inliers->indices.size() << " data points." << std::endl;
		

	}
	else
		std::cerr << "Failed to calculate plane model." << std::endl;

	if (inliers == NULL)
	{
		PCL_ERROR("Could not estimate a cone model for the given dataset.");
		return;
	}






}
struct transf getPlaneTransf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int method, Eigen::Vector4f centroid, pcl::ModelCoefficients::Ptr coefficients) {
	transf planeTransf;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;

	if (method == 1) //
	{
		//USE Nicola Fioraio method for minimum bounding box

		// Compute principal directions


		Eigen::Matrix3f covariance;
		pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
		/// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
		///    the signs are different and the box doesn't get correctly oriented in some cases.
		/*
		// Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(cloudSegmented);
		pca.project(*cloudSegmented, *cloudPCAprojection);
		std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
		std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
		// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.



		//These eigenvectors are used to transform the point cloud to the origin point(0, 0, 0) such that the eigenvectors correspond to the axes of the space.
		//The minimum point, maximum point, and the middle of the diagonal between these two points are calculated for the transformed cloud(also referred to as
		//the projected cloud when using PCL's PCA interface, or reference cloud by Nicola).

		// Transform the original cloud to the origin where the principal components correspond to the axes.
		*/
		Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
		projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
		projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * centroid.head<3>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
		// Get the minimum and maximum points of the transformed cloud.
		//pcl::PointXYZ minPoint, maxPoint;
		pcl::getMinMax3D(*cloudPointsProjected, min_point_OBB, max_point_OBB);
		const Eigen::Vector3f meanDiagonal = 0.5f*(max_point_OBB.getVector3fMap() + min_point_OBB.getVector3fMap());
		//Finally, the quaternion is calculated using the eigenvectors(which determines how the final box gets rotated), 
		//and the transform to put the box in correct location is calculated.The minimum and maximum points are used to determine the box width, height, and depth.

		// Final transform
		const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations
		const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + centroid.head<3>();
		planeTransf.trans = Eigen::Translation3f(bboxTransform[0], bboxTransform[1], bboxTransform[2]);

		planeTransf.rot = bboxQuaternion;
		planeTransf.sizeX = max_point_OBB.x - min_point_OBB.x;
		planeTransf.sizeY = max_point_OBB.y - min_point_OBB.y;
		planeTransf.sizeZ = max_point_OBB.z - min_point_OBB.z;
	}
	else if (method == 2)
	{
		//BY KNOWING THE PLANE EQUATION DERIVATE ITS 6DOF TRANSFORMATION

		Eigen::Vector3f zvector(0.0f, 0.0f, 1.0f);
		Eigen::Vector3f planevector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		float angle_rad = acos(zvector.dot(planevector));
		float angle_degrees = angle_rad * 180 / M_PI;
		if (angle_degrees > 120)
			angle_degrees = abs(angle_degrees - 180);
		Eigen::Vector3f transformVector;
		Eigen::Vector3f rotation_vector = planevector.cross(zvector);
		float length = sqrt(rotation_vector[0] * rotation_vector[0] + rotation_vector[1] * rotation_vector[1] + rotation_vector[2] * rotation_vector[2]);
		transformVector(0) = rotation_vector[0] / length;
		transformVector(1) = rotation_vector[1] / length;
		transformVector(2) = rotation_vector[2] / length;

	}
	else if (method == 3)
	{

		//USE PCL::MOMENTOFINERTIA FEATURE EXTRACTOR TO GET THE OBB (ORIENTED BOUNDING BOX) OF THE HULL POINT CLOUD
		feature_extractor.setInputCloud(cloud);

		feature_extractor.compute();
		bool gotOBB = feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		// DEPENDING ON IF OBB HAS SUCCEEDED, RETURN ITS COORDINATES

		if (gotOBB == false)
		{
			std::cerr << "Computing of bounding box failed ";

		}
		else
		{


			//Eigen::AngleAxis<float>myangleAxis(rotational_matrix_OBB);
			//Eigen::Vector3f planevector(coefficients->values[0], coefficients->values[1], coefficients->values[3]);
			Eigen::Vector3f ea = rotational_matrix_OBB.eulerAngles(0, 1, 2);
			Eigen::Quaternionf quat2(rotational_matrix_OBB);
			Eigen::Quaternionf quat;
			Eigen::Vector3f poseBB(position_OBB.x, position_OBB.y, position_OBB.z);
			float angle_rad = ea[0];
			float angle_radY = ea[1];
			float angle_degrees = angle_rad * 180 / M_PI;
			float angle_degreesY = angle_radY * 180 / M_PI;
			if ((abs(angle_degrees) > 120) || (abs(angle_degreesY) > 120)) //Keep the bounding box pose rotated with Z pointing up
			{
				if (angle_rad < 0)
					ea[2] = angle_rad + M_PI;
				else
					ea[2] = angle_rad - M_PI;
				Eigen::AngleAxisf rollAngle(ea[0], Eigen::Vector3f::UnitX());
				Eigen::AngleAxisf yawAngle(ea[1], Eigen::Vector3f::UnitY());
				Eigen::AngleAxisf pitchAngle(ea[2], Eigen::Vector3f::UnitZ());
				Eigen::AngleAxis<float>myAxis(rotational_matrix_OBB);

				Eigen::Vector3f axis = rotational_matrix_OBB*Eigen::Vector3f::UnitX();

				//axis.reverse();
				Eigen::AngleAxis<float>myNewAxis(-M_PI, Eigen::Vector3f::UnitX());


				quat = rotational_matrix_OBB.cast<float>();

				planeTransf.trans = Eigen::Translation3f(poseBB[0], poseBB[1], poseBB[2]);
				planeTransf.rot = quat*myNewAxis;
			}
			else
			{
				quat = rotational_matrix_OBB.cast<float>();

				planeTransf.trans = Eigen::Translation3f(poseBB[0], poseBB[1], poseBB[2]);
				planeTransf.rot = quat;
			}

			//quat = rotational_matrix_OBB.cast<float>();

			planeTransf.sizeX = max_point_OBB.x - min_point_OBB.x;
			planeTransf.sizeY = max_point_OBB.y - min_point_OBB.y;
			planeTransf.sizeZ = max_point_OBB.z - min_point_OBB.z;

		}

	}

	else if (method == 4) {
		std::vector<Eigen::Vector3f> bbox;
		to3D outputProjection = projectCloudTo2D(cloud, coefficients);
	
	//	viewPointsAsImage(points);
		std::vector<cv::Point2f> convexHull;
		cv::convexHull(outputProjection.points, convexHull, false, true);
	//	viewPointsAsImage(convexHull);
		cv::Mat convexHull_Mat(convexHull);
	
		//cv::Mat convexHull_Mat(convexHull);
		cv::Mat points_mat =pointsToBGRMat(outputProjection.points,false);
		cv::Mat points_GrayscaleMat;
		cv::Mat contoursMat = points_mat;
		std::vector<std::vector<cv::Point>> contours;

		int biggestContourIndex  = 0;
		int biggestContourArea = 0;
		std::vector<double> areas;
		std::vector<cv::Vec4i> hierarchy;
		cv::cvtColor(points_mat, points_GrayscaleMat, cv::COLOR_BGR2GRAY);
		cv::findContours(points_GrayscaleMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		cv::RNG rng(12345);
		for (int i = 0; i < contours.size(); i++)
		{
			areas.push_back(cv::contourArea(contours[i]));
		}
		for (int i = 0; i < areas.size(); i++)
		{
			if (areas[i] > biggestContourArea)
			{
				biggestContourArea = areas[i];
				biggestContourIndex = i;
			}
				
		}
		cv::Mat biggestCountourMat=pointsToBGRMat(contours[biggestContourIndex], false);
		std::cerr << "The contour area is " << biggestContourArea << " square pixels" << std::endl;
		

		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::drawContours(contoursMat, contours, biggestContourIndex, color, 2, 8, hierarchy, 0);

		
		cv::imshow("Contours", contoursMat);
		cv::waitKey(0);
		// Set up the detector with default parameters.

		//BLOB SEARCH
		
		//cv::SimpleBlobDetector::Params params;
		//params.minThreshold = 10;
		//params.blobColor = 255;
		//params.filterByConvexity = false;
		//params.minConvexity = 0.5;
		//params.filterByArea = false;
		//params.minArea = 10;
		//params.filterByInertia = false;
		//cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	
		//// Detect blobs.
		//std::vector<cv::KeyPoint> keypoints;
		//
		//detector->detect(points_GrayscaleMat, keypoints);

		//// Draw detected blobs as red circles.
		//// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
		//cv::Mat im_with_keypoints;
		//drawKeypoints(points_mat, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		//// Show blobs
		//cv::imshow("keypoints", im_with_keypoints);
		//cv::waitKey(0);
	




		/*cv::Mat points_mat_closed;
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 50));
		cv::morphologyEx(points_mat, points_mat_closed, cv::MORPH_CLOSE, kernel);*/

		//cv::RotatedRect rrect = cv::minAreaRect(contours[biggestContourIndex]);
		cv::RotatedRect rrect = cv::minAreaRect(outputProjection.points);
		float rectArea = rrect.size.area();
		std::cerr << "The rectangle area: " << rectArea << " square pixels" << std::endl;
		std::cerr << "Percentage cover is " << biggestContourArea/rectArea*100 << " percent. " << std::endl;
		cv::Point2f rrPts[4];
		rrect.points(rrPts);

		

		//vizualize2D(points_mat, rrPts, rrect);//Vizualize 2D projection and selected four points rectangle
		vizualize2DinOne(outputProjection.points, rrPts, rrect);
		
		
		std::vector<pcl::PointXYZ> points3D(5);
		//reproject the 4 rectangle points + the center and store into a vector of 3D points
		for (unsigned int ii = 0; ii < 4; ii++)
		{
			Eigen::Vector3f pbbx(rrPts[ii].x*outputProjection.u + rrPts[ii].y*outputProjection.v + outputProjection.p0);
			points3D[ii].x = pbbx[0];
			points3D[ii].y = pbbx[1];
			points3D[ii].z = pbbx[2];
			bbox.push_back(pbbx);
		}



		//The longest distance should be the X direction. The Z axis should always be pointing up.

		//Calculate the distance between point 1 and 2, and point 1 and 4. OpenCV returns the points in a anticlockwise fashion.
		float distanceX = pcl::euclideanDistance(points3D[0], points3D[1]);
		float distanceY = pcl::euclideanDistance(points3D[0], points3D[3]);

		//According to which distance is greatest, assign a 3D vector that goes from point 1 to point 2 or 4 (X). The Y vector will be chosen
		//According to the direction that makes the Z point upwards
		Eigen::Vector3f XDir;
		Eigen::Vector3f YDir;
		if (distanceX < distanceY)
		{
			float temp = distanceX;
			distanceX = distanceY;
			distanceY = temp;
			XDir = bbox[3] - bbox[0];
			YDir = bbox[1] - bbox[0];
		}
		else
		{
			XDir = bbox[1] - bbox[0];
			YDir = bbox[3] - bbox[0];
		}
		Eigen::Vector3f ZDir = XDir.cross(YDir);
		if (ZDir[2] < 0) //If the Z component of the the Z direction vector is negative, reverse Y and Z
		{
			YDir = -YDir;
			ZDir = XDir.cross(YDir);
		}
		Eigen::Vector3f center(rrect.center.x*outputProjection.u + rrect.center.y*outputProjection.v + outputProjection.p0);
		bbox.push_back(center);
		
		points3D[4].x =center[0];
		points3D[4].y = center[1];
		points3D[4].z = center[2];
		
		//Set the center of the found rectangle as the position of the box
		planeTransf.trans = Eigen::Translation3f(bbox[4][0], bbox[4][1], bbox[4][2]);
		planeTransf.sizeX = distanceX;
		planeTransf.sizeY = distanceY;
		planeTransf.sizeZ = 10; //Set a hard value for the box height

		//To find the pose of the new coordinate system, use the fact that we know the direction of the new X, Y directions.
		//the Z vector is found as just the cross product of the two.
		//The affine transform can be initialized using a linear transformation of the three axis, and this transform tells us where the plane is in the world.
		Eigen::Affine3f T= Eigen::Affine3f::Identity();
		T.linear() << XDir, YDir, ZDir;
		planeTransf.rot = T.rotation();
	}
	return planeTransf;
}
struct to3D projectCloudTo2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients) {
	// Project points onto the found plane
	struct to3D my3DInst;
	pcl::PointCloud<pcl::PointXYZ> projected_cloud;
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);

	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(projected_cloud);

	// store the plane parameters
	Eigen::Vector3f plane_normal;
	plane_normal.x() = coefficients->values[0];
	plane_normal.y() = coefficients->values[1];
	plane_normal.z() = coefficients->values[2];

	// compute an orthogonal normal to the plane normal
	Eigen::Vector3f v = plane_normal.unitOrthogonal();

	// take the cross product of the two normals to get
	// a thirds normal, on the plane
	Eigen::Vector3f u = plane_normal.cross(v);

	// project the 3D point onto a 2D plane
	std::vector<cv::Point2f> points;
	// choose a point on the plane (here, the first one)
	Eigen::Vector3f p0(projected_cloud.points[0].x,
		projected_cloud.points[0].y,
		projected_cloud.points[0].z);
	for (unsigned int ii = 0; ii < projected_cloud.points.size(); ii++)
	{
		Eigen::Vector3f p3d(projected_cloud.points[ii].x,
			projected_cloud.points[ii].y,
			projected_cloud.points[ii].z);

		// subtract all 3D points with the point in the plane
		// this will move the origin of the 3D coordinate system
		// onto the plane
		p3d = p3d - p0;

		cv::Point2f p2d;
		p2d.x = p3d.dot(u);
		p2d.y = p3d.dot(v);
		points.push_back(p2d);
	}
	my3DInst.points = points;
	my3DInst.p0 = p0;
	my3DInst.u = u;
	my3DInst.v = v;
	return my3DInst;
}
void vizualize2D (std::vector<cv::Point2f> points, cv::Point2f Selected_4 [4], cv::RotatedRect rrect)
{
	int offset =500;
	char window[] = "Drawing 1: Points";
	char window2[] = "Drawing 2: 4 Selected points";
	float w = 1;
	cv::Mat image = cv::Mat::zeros(ceil(points.size()/2), ceil(points.size() / 2), CV_8UC3);
	for (unsigned int ii = 0; ii < points.size(); ii++)
	{
		cv::Point center;
		center.x = points[ii].x+offset;
		center.y = points[ii].y+offset;
		cv::circle(image,	center,	w / 32.0,cv::Scalar(255,0,0),	-1,8);
	}

	cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
 	cv::imshow(window, image);
	cv::waitKey();
	cv::moveWindow(window, 0, 200);
	cv::Mat image2 = cv::Mat::zeros(ceil(points.size() / 2), ceil(points.size() / 2), CV_8UC3);
	cv::Point vertices[5];
	for (int i = 0; i < 4; ++i)
	{
		vertices[i].x = Selected_4[i].x+offset;
		vertices[i].y = Selected_4[i].y + offset;
		
	}
	cv::circle(image2, vertices[0], w, cv::Scalar(255, 0, 0), -1, 8); //First point red
	cv::circle(image2, vertices[1], w, cv::Scalar(0, 255, 0), -1, 8); //Sec point green
	cv::circle(image2, vertices[2], w, cv::Scalar(0, 0, 255), -1, 8);//Third point blue
	cv::circle(image2, vertices[3], w, cv::Scalar(255,255, 0), -1, 8);//Fourth point yellow

	vertices[4].x = rrect.center.x + offset;
	vertices[4].y = rrect.center.y + offset;
	cv::circle(image2, vertices[4], w / 32.0, cv::Scalar(255, 255, 255), -1, 8);
	//cv::fillConvexPoly(image,vertices,4,cv::Scalar(255,0,0));
	cv::namedWindow(window2, cv::WINDOW_AUTOSIZE);
	cv::imshow(window2, image2);
	cv::waitKey();
	cv::moveWindow(window2, 300, 200);


}
void vizualize2DinOne(std::vector<cv::Point2f> points, cv::Point2f Selected_4[4], cv::RotatedRect rrect)
{
	int offset = 500;
	char window[] = "Drawing 1: Points";
	char window2[] = "Drawing 2: 4 Selected points";
	float w = 1;
	cv::Scalar red(0, 0, 255);
	cv::Scalar green(0, 255, 0);
	cv::Scalar blue(255, 0, 0);
	cv::Scalar white(255, 255, 255);
	cv::Mat image = cv::Mat::zeros(20000, 20000, CV_8UC3);
	for (unsigned int ii = 0; ii < points.size(); ii++)
	{
		cv::Point center;
		center.x = points[ii].x + offset;
		center.y = points[ii].y + offset;
		cv::circle(image, center, w / 32.0, cv::Scalar(255, 0, 0), -1, 8);
	}

	
	cv::Point vertices[5];
	for (int i = 0; i < 4; ++i)
	{
		vertices[i].x = Selected_4[i].x + offset;
		vertices[i].y = Selected_4[i].y + offset;

	}
	cv::circle(image, vertices[0], w, red, -1, 8); //First point red
	cv::circle(image, vertices[1], w, green, -1, 8); //Sec point green
	cv::circle(image, vertices[2], w, blue, -1, 8);//Third point blue
	cv::circle(image, vertices[3], w, cv::Scalar(0, 255, 0), -1, 8);//Fourth point yellow

	for (int i = 0; i < 3; ++i)
	{
		cv::line(image, vertices[i], vertices[i + 1], green);
	}
	cv::line(image, vertices[3], vertices[0], green);

	vertices[4].x = rrect.center.x + offset;
	vertices[4].y = rrect.center.y + offset;
	cv::circle(image, vertices[4], w / 32.0, cv::Scalar(255, 255, 255), -1, 8);
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(90);
	
	bool ImageSaved =cv::imwrite("C:\\Users\\Asus\\Documents\\Utility_Programs\\RangerTraining\\Ranger Training\\Project\\findPlanes2Jan18\\Build\\Debug\\Boundingbox.jpeg", image,compression_params);
	if (!ImageSaved)
		std::cerr << "Debug 2D image could not be saved." << std::endl;
	else
		std::cerr << "Debug 2D image saved." << std::endl;
	//cv::fillConvexPoly(image,vertices,4,cv::Scalar(255,0,0));

	cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
	cv::imshow(window, image);
	cv::waitKey();


}
void vizualize2DinOne(cv::Mat image, cv::Point2f Selected_4[4], cv::RotatedRect rrect)
{
	int offset = 500;
	char window[] = "Drawing 1: Points";
	char window2[] = "Drawing 2: 4 Selected points";
	float w = 1;
	cv::Scalar red(0, 0, 255);
	cv::Scalar green(0, 255, 0);
	cv::Scalar blue(255, 0, 0);
	cv::Scalar white(255, 255, 255);
	
	cv::Point vertices[5];
	for (int i = 0; i < 4; ++i)
	{
		vertices[i].x = Selected_4[i].x + offset;
		vertices[i].y = Selected_4[i].y + offset;

	}
	cv::circle(image, vertices[0], w, red, -1, 8); //First point red
	cv::circle(image, vertices[1], w, green, -1, 8); //Sec point green
	cv::circle(image, vertices[2], w, blue, -1, 8);//Third point blue
	cv::circle(image, vertices[3], w, cv::Scalar(0, 255, 0), -1, 8);//Fourth point yellow

	for (int i = 0; i < 3; ++i)
	{
		cv::line(image, vertices[i], vertices[i + 1], green);
	}
	cv::line(image, vertices[3], vertices[0], green);

	vertices[4].x = rrect.center.x + offset;
	vertices[4].y = rrect.center.y + offset;
	cv::circle(image, vertices[4], w / 32.0, cv::Scalar(255, 255, 255), -1, 8);
	//cv::fillConvexPoly(image,vertices,4,cv::Scalar(255,0,0));
	cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
	cv::imshow(window, image);
	cv::waitKey();


}
void viewPointsAsImage(std::vector<cv::Point2f> points)
{
	int offset = 500;
	char window[] = "Points as Image";
	float w = 1;
	cv::Scalar red(0, 0, 255);
	cv::Scalar green(0, 255, 0);
	cv::Scalar blue(255, 0, 0);
	cv::Scalar white(255, 255, 255);
	cv::Mat image = cv::Mat::zeros(20000, 20000, CV_8UC3);
	for (unsigned int ii = 0; ii < points.size(); ii++)
	{
		cv::Point center;
		center.x = points[ii].x + offset;
		center.y = points[ii].y + offset;
		cv::circle(image, center, w /5, white, -1, 8);
	}


	cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
	cv::imshow(window, image);
	cv::waitKey();


}
cv::Mat pointsToBWMat(std::vector<cv::Point2f> points)
{
	cv::Mat image = cv::Mat::zeros(20000, 20000, CV_8UC1);
	for (unsigned int ii = 0; ii < points.size(); ii++)
	{
		cv::Point center;
		center.x = points[ii].x;
		center.y = points[ii].y;
		cv::circle(image, center,1, cv::Scalar(255), -1, 8);
	}
	return image;
}
cv::Mat pointsToBGRMat(std::vector<cv::Point2f> points,bool withOffset)
{
	cv::Mat image = cv::Mat::zeros(20000, 20000, CV_8UC3);
	if (withOffset)
	{
		for (unsigned int ii = 0; ii < points.size(); ii++)
		{
			cv::Point center;

			center.x = points[ii].x+500;
			center.y = points[ii].y+500;
			cv::circle(image, center, 1 , cv::Scalar(255, 255, 255), -1, 8);
		}
	}
	else
	{
		for (unsigned int ii = 0; ii < points.size(); ii++)
		{
			cv::Point center;

			center.x = points[ii].x;
			center.y = points[ii].y;
			cv::circle(image, center, 1, cv::Scalar(255, 255, 255), -1, 8);
		}
	}
	
	return image;
}
cv::Mat pointsToBGRMat(std::vector<cv::Point> points, bool withOffset)
{
	cv::Mat image = cv::Mat::zeros(20000, 20000, CV_8UC3);
	if (withOffset)
	{
		for (unsigned int ii = 0; ii < points.size(); ii++)
		{
			cv::Point center;

			center.x = points[ii].x + 500;
			center.y = points[ii].y + 500;
			cv::circle(image, center, 1, cv::Scalar(255, 255, 255), -1, 8);
		}
	}
	else
	{
		for (unsigned int ii = 0; ii < points.size(); ii++)
		{
			cv::Point center;

			center.x = points[ii].x;
			center.y = points[ii].y;
			cv::circle(image, center, 1, cv::Scalar(255, 255, 255), -1, 8);
		}
	}

	return image;
}