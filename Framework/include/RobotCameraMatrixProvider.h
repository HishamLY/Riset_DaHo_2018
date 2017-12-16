/**
* @file RobotCameraMatrixProvider.h
* This file declares a class to calculate the position of the camera relative to the body for the Nao.
*/

#ifndef _ROBOTCAMERAMATRIXPROVIDER_H_
#define _ROBOTCAMERAMATRIXPROVIDER_H_


#include "CameraCalibration.h"
#include "RobotDimensions.h"
#include "JointData.h"
#include "CameraMatrix.h"
#include "Head.h"
#include "MotionStatus.h"
#include <stdio.h>

namespace Robot{
class RobotCameraMatrixProvider
{
public:
  void update(RobotCameraMatrix& robotCameraMatrix,   CameraCalibration& theCameraCalibration);
  void update(RobotCameraMatrixPrev& robotCameraMatrixPrev,   CameraCalibration& theCameraCalibration);

  //-----ADDITIONAL DECLARATION-------//

  RobotDimensions theRobotDimensions;
};
}
#endif // RobotCameraMatrixProvider_H
