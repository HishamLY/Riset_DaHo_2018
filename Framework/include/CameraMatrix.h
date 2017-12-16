/**
* @file CameraMatrix.h
* Declaration of CameraMatrix and RobotCameraMatrix representation.
* @author Colin Graf
*/

#ifndef _CAMERAMATRIX_H_
#define _CAMERAMATRIX_H_

#include "Pose3D.h"
#include "Image.h"
#include "RobotDimensions.h"
#include "CameraCalibration.h"

namespace Robot
{
// Matrix transformasi dari selakangan ke kamera
class RobotCameraMatrix : public Pose3D
{
public:
  RobotCameraMatrix() {}
  RobotCameraMatrix(const RobotDimensions& robotDimensions,const float headYaw,const float headPitch,const CameraCalibration& cameraCalibration,bool upperCamera);

  void draw(Image* img);   /** Draws the camera matrix. */
  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions,float headYaw,float headPitch,const CameraCalibration& cameraCalibration,bool upperCamera);
};


//Matrix transformasi dari tanah ( center between both feet) ke kamera.
class CameraMatrix : public Pose3D
{
public:
  bool isValid; /**< Matrix is only valid if motion was stable. */

  CameraMatrix(const Pose3D& pose): Pose3D(pose), isValid(true) {} /** Kind of copy-constructor. @param pose The other pose.*/
  CameraMatrix() : isValid(true) {}   /** Default constructor. */
  CameraMatrix(const Pose3D& torsoMatrix,const Pose3D& robotCameraMatrix,const CameraCalibration& cameraCalibration);

  void computeCameraMatrix(const Pose3D& torsoMatrix,const Pose3D& robotCameraMatrix,const CameraCalibration& cameraCalibration);
  void draw(Image* img);   /** Draws the camera matrix. */
};

class CameraMatrixPrev : public CameraMatrix {};
class RobotCameraMatrixPrev : public RobotCameraMatrix {};
}
#endif
