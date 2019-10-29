#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ThreadSafePrint.h"
#include <conio.h>
#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <cstdint>

void write_pcd(float *arrayXdata, float *arrayYdata, float *arrayZdata, int width, int height, char* outputPCD, bool binaryPCD);
void write_RGB_pcd(float *arrayXdata, float *arrayYdata, float *arrayZdata, uint8_t *RGBR, uint8_t *RGBG, uint8_t *RGBB, int width, int height, char* outputPCD, bool binaryPCD);
void write_color_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, char* outputPCD, bool binaryPCD);