#pragma once
#include <pcl/io/io.h>

class CubicVOI
{
	
	Eigen::Vector3f cube_Translation;
	Eigen::Quaternionf cube_Rotation;
	unsigned int Xsize;
	unsigned int Ysize;
	unsigned int Zsize;
public:
	CubicVOI();
	CubicVOI(unsigned int sizeX_in, unsigned int sizeY_in, unsigned int sizeZ_in, float transX, float transY, float transZ, float qW, float qX, float qY, float qZ);
	Eigen::Vector3f	trans(void) { return cube_Translation; };
	Eigen::Quaternionf rot(void) { return cube_Rotation; };
	unsigned int sizeX(void) { return Xsize; };
	unsigned int sizeY(void) { return Ysize; };
	unsigned int sizeZ(void) { return Zsize; };
	void setSizeX(unsigned int sizeX2) { Xsize = sizeX2; }
	void setSizeY(unsigned int sizeY2) { Ysize = sizeY2; }
	void setSizeZ(unsigned int sizeZ2) { Zsize = sizeZ2; }
	void setTrans(Eigen::Vector3f trans2) { cube_Translation = trans2; }
	void setRot(Eigen::Quaternionf rot2) { cube_Rotation=rot2; }
};