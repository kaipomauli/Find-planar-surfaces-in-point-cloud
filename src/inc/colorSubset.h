#pragma once
#include <pcl\filters\extract_indices.h>
#include <CubicVOI.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/conversions.h>
#include "ThreadSafePrint.h"
#include <segObject.h>
#include <pcl/filters/passthrough.h>
#include <write_pcd.h>

void extractIndicesFromVOI(pcl::PointIndicesPtr indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CubicVOI voi);
void filterFromVOI(pcl::PointIndicesPtr indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CubicVOI voi);
void removeIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr, pcl::PointIndices::Ptr remIndices);
void colorSubset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, segObject objectToColor);
void colorAllSubset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, std::vector<segObject>& objectToColor);