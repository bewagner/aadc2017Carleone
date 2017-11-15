#ifndef _UTIL_
#define _UTIL_

#include "stdafx.h"
//#include "../../include/camera_angle.h"
#include "camera_angle.h"

namespace Util{

	Point3f ComputeWorldCoordinate(float pPoint_x, float pPoint_y, float pDepthValue, int ImagecutHeightUp, int ImagecutWidthLeft);

};

#endif //Util	
