#include "CubicVOI.h"

CubicVOI::CubicVOI() {
	Xsize = 0;
	Ysize = 0;
	Zsize = 0;
	cube_Translation << 0, 0, 0;
	cube_Rotation.w()=1;
	cube_Rotation.x() = 0;
	cube_Rotation.y() = 0;
	cube_Rotation.z()= 0;
	
}

CubicVOI::CubicVOI(unsigned int sizeX_in, unsigned int sizeY_in, unsigned int sizeZ_in, float transX, float transY, float transZ, float qW, float qX, float qY, float qZ) {
	Xsize = sizeX_in;
	Ysize = sizeY_in;
	Zsize = sizeZ_in;
	cube_Translation << transX, transY, transZ;
	cube_Rotation.w()=qW;
	cube_Rotation.x() = qX;
	cube_Rotation.y() = qY;
	cube_Rotation.z() = qZ;

	

}
