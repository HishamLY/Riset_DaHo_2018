/**
* @file CameraMatrixProvider.h
* This file declares a class to calculate the position of the camera for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef CameraMatrixProvider_H
#define CameraMatrixProvider_H

#include "CameraCalibration.h"
#include "RobotDimensions.h"
//#include "FieldDimensions.h"
#include "JointData.h"
#include "Camera.h"
#include "CameraMatrix.h"
#include "TorsoMatrix.h"
#include "MotionStatus.h"

namespace Robot{
class CameraMatrixProvider
{
public:
  void update(CameraMatrixPrev& cameraMatrixPrev, TorsoMatrixPrev& theTorsoMatrixPrev, RobotCameraMatrixPrev& theRobotCameraMatrixPrev,CameraCalibration& theCameraCalibration);
  void update(CameraMatrix& cameraMatrix,TorsoMatrix& theTorsoMatrix, RobotCameraMatrix& theRobotCameraMatrix, CameraCalibration& theCameraCalibration);

  void camera2image(const Vector3<>& camera, Vector2<>& image) const;
  bool intersectLineWithCullPlane(const Vector3<>& lineBase,
                                  const Vector3<>& lineDir,
                                  Vector3<>& point) const;

  void drawFieldLines(const CameraMatrix& cameraMatrix) const;
};
}
#endif // CameraMatrixProvider_H
