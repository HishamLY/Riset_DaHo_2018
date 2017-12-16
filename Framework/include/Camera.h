/*
 *   Camera.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "Vector2.h"

namespace Robot
{
class Camera
{
public:


  static const double VIEW_V_ANGLE = 46.0; //degree = 0.8028 rad
  static const double VIEW_H_ANGLE = 58.0; //degree = 1.012 rad

	static int WIDTH;
  static int HEIGHT;

//  static const int WIDTH = 352;//352
//  static const int HEIGHT = 288;//288

  /** Intrinsic camera parameters: axis skew is modelled as 0 (90° perfectly orthogonal XY)
  * and the same has been modeled for focal axis aspect ratio; distortion is considering
  * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
  */

  static const float focalLength = 385.54f;
  static const float focalLengthInv = 0.0026f; // (1/focalLength) used to speed up certain calculations
  static const float focalLenPow2 = 122500.0f;
  static const float focalLenPow4 = 15000000000.0f;

//  Camera();
//  ~Camera();
  // Default constructor.

/*
  Camera()
  {
    focalLength = 50;
    //opticalCenter.x = WIDTH/2; // unchecked
    //opticalCenter.y = HEIGHT/2; // unchecked
    focalLenPow2 = focalLength * focalLength;
    focalLenPow4 = focalLenPow2 * focalLenPow2;
    focalLengthInv = 1 / focalLength;
  }
*/
};
}

#endif
