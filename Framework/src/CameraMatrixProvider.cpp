/**
* @file CameraMatrixProvider.cpp
* This file implements a class to calculate the position of the camera for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
* @author Colin Graf
*/

#include "CameraMatrixProvider.h"
using namespace Robot;

void CameraMatrixProvider::update(CameraMatrixPrev& cameraMatrixPrev, TorsoMatrixPrev& theTorsoMatrixPrev, RobotCameraMatrixPrev& theRobotCameraMatrixPrev, CameraCalibration& theCameraCalibration)
{
  //cameraMatrixPrev.computeCameraMatrix(theTorsoMatrixPrev, theRobotCameraMatrixPrev, theCameraCalibration);
  //cameraMatrixPrev.isValid = true;//theTorsoMatrixPrev.isValid && true;
}

void CameraMatrixProvider::update(CameraMatrix& cameraMatrix, TorsoMatrix& theTorsoMatrix, RobotCameraMatrix& theRobotCameraMatrix, CameraCalibration& theCameraCalibration)
{
  cameraMatrix.computeCameraMatrix(theTorsoMatrix, theRobotCameraMatrix, theCameraCalibration);
  cameraMatrix.isValid  = true; //theTorsoMatrix.isValid && (MotionStatus::FALLEN == STANDUP);
}

void CameraMatrixProvider::camera2image(const Vector3<>& camera, Vector2<>& image) const
{
  const float& scale(Camera::focalLength / camera.x);
  image.x = float(Camera::WIDTH/2 - scale * camera.y);
  image.y = float(Camera::HEIGHT/2 - scale * camera.z);
}

bool CameraMatrixProvider::intersectLineWithCullPlane(const Vector3<>& lineBase, const Vector3<>& lineDir,
    Vector3<>& point) const
{
  if(lineDir.x == 0.)
    return false;
  point = lineBase + lineDir * ((lineBase.x - 200.0f) / -lineDir.x);
  return true;
}

void CameraMatrixProvider::drawFieldLines(const CameraMatrix& cameraMatrix) const
{
 /*   Vector3<> start0C, start1C, end0C, end1C;
    Vector2<> start0I, start1I, end0I, end1I;

    const Pose2D& robotPoseInv(theRobotPose.invert());
    const Pose3D& cameraMatrixInv(cameraMatrix.invert());
    int halfFieldLinesWidth = theFieldDimensions.fieldLinesWidth / 2;

    for(unsigned int i = 0; i < theFieldDimensions.fieldLines.lines.size(); ++i)
    {
  Pose2D relativeLine(robotPoseInv);
  relativeLine.conc(theFieldDimensions.fieldLines.lines[i].corner);
  const Vector2<> start0(Pose2D(relativeLine).translate(0, (float) -halfFieldLinesWidth).translation);
  const Vector2<> end1(Pose2D(relativeLine).translate(theFieldDimensions.fieldLines.lines[i].length, (float) halfFieldLinesWidth).translation);

  start0C = cameraMatrixInv * Vector3<>(start0.x, start0.y, 0.); // field2camera
  end1C = cameraMatrixInv * Vector3<>(end1.x, end1.y, 0.); // field2camera

  if(start0C.x <= 200 && end1C.x <= 200)
    continue;

  const Vector2<>& start1(Pose2D(relativeLine).translate(0, (float) halfFieldLinesWidth).translation);
  const Vector2<>& end0(Pose2D(relativeLine).translate(theFieldDimensions.fieldLines.lines[i].length, (float) -halfFieldLinesWidth).translation);

  start1C = cameraMatrixInv * Vector3<>(start1.x, start1.y, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3<>(end0.x, end0.y, 0.); // field2camera

  if(start0C.x <= 200)
    intersectLineWithCullPlane(start0C, end0C - start0C, start0C);
  else if(end0C.x <= 200)
    intersectLineWithCullPlane(start0C, end0C - start0C, end0C);
  if(start1C.x <= 200)
    intersectLineWithCullPlane(start1C, end1C - start1C, start1C);
  else if(end1C.x <= 200)
    intersectLineWithCullPlane(start1C, end1C - start1C, end1C);

  camera2image(start0C, start0I);
  camera2image(end0C, end0I);
  camera2image(start1C, start1I);
  camera2image(end1C, end1I);

  //LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x, start0I.y, end0I.x, end0I.y, 0, Drawings::ps_solid, ColorRGBA(0,0,0));
  //LINE("module:CameraMatrixProvider:calibrationHelper", start1I.x, start1I.y, end1I.x, end1I.y, 0, Drawings::ps_solid, ColorRGBA(0,0,0));
    }

    start0C = cameraMatrixInv * Vector3<>(100, -11, 0.); // field2camera
    end0C = cameraMatrixInv * Vector3<>(0, -11, 0.); // field2camera
    camera2image(start0C, start0I);
    camera2image(end0C, end0I);

    //LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x, start0I.y, end0I.x, end0I.y, 0, Drawings::ps_solid, ColorClasses::blue);

    start0C = cameraMatrixInv * Vector3<>(100, 11, 0.); // field2camera
    end0C = cameraMatrixInv * Vector3<>(0, 11, 0.); // field2camera
    camera2image(start0C, start0I);
    camera2image(end0C, end0I);

    //LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x, start0I.y, end0I.x, end0I.y, 0, Drawings::ps_solid, ColorClasses::blue);

    start0C = cameraMatrixInv * Vector3<>(110, 1000, 0.); // field2camera
    end0C = cameraMatrixInv * Vector3<>(110, -1000, 0.); // field2camera
    camera2image(start0C, start0I);
    camera2image(end0C, end0I);

    //LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x, start0I.y, end0I.x, end0I.y, 0, Drawings::ps_solid, ColorClasses::blue);
*/
}
