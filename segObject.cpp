#include <segObject.h>

segObject::segObject(pcl::PointIndicesPtr input_indicesPtr, MyColor* input_color) {
	indices = input_indicesPtr;
	color = input_color;
};
segObject::segObject(pcl::PointIndices input_indices, MyColor* input_color) {
	indices->indices = input_indices.indices;
	color = input_color;
};