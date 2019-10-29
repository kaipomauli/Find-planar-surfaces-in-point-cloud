#pragma once
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <math.h>
#include <pcl/surface/concave_hull.h>
#include <CubicVOI.h>
#include <vizualize.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <math.h>

void findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, pcl::PointIndices::Ptr voi_Indices, Eigen::VectorXf  modelcoef, pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr hullinliers, Eigen::Affine3f* planePose,CubicVOI*boundingBox);
void findPlaneRedCld(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudReduced, pcl::PointIndices::Ptr inliers, Eigen::Affine3f* planePose, CubicVOI*boundingBox);
struct transf { Eigen::Translation3f trans;	Eigen::Quaternionf rot; int sizeX; int sizeY; int sizeZ; };
struct to3D  projectCloudTo2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients);
struct to3D { std::vector<cv::Point2f> points; Eigen::Vector3f p0; Eigen::Vector3f u; Eigen::Vector3f v; };
struct transf getPlaneTransf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int method, Eigen::Vector4f centroid, pcl::ModelCoefficients::Ptr coefficients);
void vizualize2D(std::vector<cv::Point2f> points, cv::Point2f Selected_4[4], cv::RotatedRect rrect);
void vizualize2DinOne(cv::Mat image, cv::Point2f Selected_4[4], cv::RotatedRect rrect);
void vizualize2DinOne(std::vector<cv::Point2f> points, cv::Point2f Selected_4[4], cv::RotatedRect rrect);
void viewPointsAsImage(std::vector<cv::Point2f> points);
cv::Mat pointsToBWMat(std::vector<cv::Point2f> points);
cv::Mat pointsToBGRMat(std::vector<cv::Point2f> points, bool withOffset = false);
cv::Mat pointsToBGRMat(std::vector<cv::Point> points, bool withOffset);