/*
 * Date 15.12.15 
 */

#include "Util.h"
#include "intrinsic_data.h"

#include <fstream>
 
namespace Util{

	/*
	 * Calculating world coordinate with depth image and intrinsic data in meters.
	 * This function rotates the system to the one of the car, which is parallel to the ground.
	 */
	Point3f ComputeWorldCoordinate(float pPoint_x, float pPoint_y, float pDepthValue, int ImagecutHeightUp, int ImagecutWidthLeft){
		
		Point3f point;

		point.x 	= (pPoint_x - c_x + ImagecutWidthLeft) * pDepthValue / (f_x*1000);
		point.y 	= ((pPoint_y - c_y + ImagecutHeightUp) * pDepthValue*cos(camera_angle_rad) / (f_y)+sin(camera_angle_rad)*pDepthValue)/1000;		
		point.z 	= -(sin(camera_angle_rad)*cos(camera_angle_rad)/f_y*pPoint_y*pDepthValue	-	(c_y*sin(camera_angle_rad)/f_y	+	cos(camera_angle_rad))*pDepthValue)/1000; 
	

		return point;

	}

}
