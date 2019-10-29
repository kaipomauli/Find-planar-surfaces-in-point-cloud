#pragma once
#include <pcl\filters\extract_indices.h>
#include <CubicVOI.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <MyColor.h>
class segObject {
public:
	MyColor* color;
	pcl::PointIndicesPtr indices;
	segObject(pcl::PointIndicesPtr input_indicesPtr, MyColor* input_color);
	segObject(pcl::PointIndices input_indices, MyColor* input_color);
};
