/**
 * @file CameraMatrix.cpp
 * Implementation of class CameraMatrix.
 */

#include "CameraMatrix.h"
#include "Boundary.h"
#include "DebugDrawings.h"
#include "Geometry.h"
#include <stdio.h>

using namespace Robot;

void CameraMatrix::draw(Image* img)
{

    Vector2<int> pointOnField[6];
    // calculate the projection of the four image corners to the ground
    Geometry::calculatePointOnField(0, 0, *this, pointOnField[0]);
    Geometry::calculatePointOnField(Camera::WIDTH, 0, *this,  pointOnField[1]);
    Geometry::calculatePointOnField(Camera::WIDTH, Camera::HEIGHT, *this,pointOnField[2]);
    Geometry::calculatePointOnField(0, Camera::HEIGHT, *this, pointOnField[3]);

    // calculate a line 15 pixels below the horizon in the image
    Geometry::Line horizon = Geometry::calculateHorizon(*this);
    Geometry::Line lineBelowHorizon;
    Vector2<> vertLineDirection(-horizon.direction.y, horizon.direction.x);
    lineBelowHorizon.direction = horizon.direction;
    lineBelowHorizon.base = horizon.base;
    lineBelowHorizon.base += vertLineDirection * 15.0f;

    // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
    Vector2<int> beginPoint;
    Vector2<int> endPoint;
    if(Geometry::getIntersectionPointsOfLineAndRectangle(
       Vector2<int>(0,0), Vector2<int>(Camera::WIDTH-1, Camera::HEIGHT-1), lineBelowHorizon, beginPoint, endPoint))
    {
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y),ColorRGB(ColorClasses::red));
      Geometry::calculatePointOnField(beginPoint.x,beginPoint.y, *this, pointOnField[4]);
      Geometry::calculatePointOnField(endPoint.x, endPoint.y, *this, pointOnField[5]);
    }

    // determine the boundary of all the points that were projected to the ground
    Boundary<int> boundary(-10000, +10000);
    if(pointOnField[0].x !=0 || pointOnField[0].y != 0) {boundary.add(pointOnField[0]); }
    if(pointOnField[1].x !=0 || pointOnField[1].y != 0) {boundary.add(pointOnField[1]); }
    if(pointOnField[2].x !=0 || pointOnField[2].y != 0) {boundary.add(pointOnField[2]); }
    if(pointOnField[3].x !=0 || pointOnField[3].y != 0) {boundary.add(pointOnField[3]); }
    if(pointOnField[4].x !=0 || pointOnField[4].y != 0) {boundary.add(pointOnField[4]); }
    if(pointOnField[5].x !=0 || pointOnField[5].y != 0) {boundary.add(pointOnField[5]); }

    // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
    int spacing = 100;
    for(int xx = boundary.x.min - boundary.x.min%spacing + spacing ; xx <= boundary.x.max; xx+=spacing)
    {
      Geometry::calculatePointInImage(Vector3<>((float) xx, (float) boundary.y.min, 0), *this, beginPoint);
      Geometry::calculatePointInImage(Vector3<>((float) xx, (float) boundary.y.max, 0), *this, endPoint);
      int lineWidth = 0;
      if(xx == 0) lineWidth = 3;
      //Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), lineWidth, ColorRGB(ColorClasses::white));
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), ColorRGB(ColorClasses::white));
    }

    for(int yy = boundary.y.min - boundary.y.min%spacing + spacing ; yy <= boundary.y.max; yy+=spacing)
    {
      Geometry::calculatePointInImage(Vector3<>((float) boundary.x.min, (float) yy, 0), *this, beginPoint);
      Geometry::calculatePointInImage(Vector3<>((float) boundary.x.max, (float) yy, 0), *this, endPoint);
      int lineWidth = 0;
      if(yy == 0) lineWidth = 3;
     // Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y),lineWidth, ColorRGB(ColorClasses::white));
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), ColorRGB(ColorClasses::black));

}
}

void RobotCameraMatrix::draw(Image* img)
{

    Vector2<int> pointOnField[6];
    // calculate the projection of the four image corners to the ground
    Geometry::calculatePointOnField(0, 0, *this, pointOnField[0]);
    Geometry::calculatePointOnField(Camera::WIDTH, 0, *this, pointOnField[1]);
    Geometry::calculatePointOnField(Camera::WIDTH, Camera::HEIGHT, *this, pointOnField[2]);
    Geometry::calculatePointOnField(0, Camera::HEIGHT, *this, pointOnField[3]);

    // calculate a line 15 pixels below the horizon in the image
    Geometry::Line horizon = Geometry::calculateHorizon(*this);
    Geometry::Line lineBelowHorizon;
    Vector2<> vertLineDirection(-horizon.direction.y, horizon.direction.x);
    lineBelowHorizon.direction = horizon.direction;
    lineBelowHorizon.base = horizon.base;
    lineBelowHorizon.base += vertLineDirection * 15.0;

    // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
    Vector2<int> beginPoint;
    Vector2<int> endPoint;
    if(Geometry::getIntersectionPointsOfLineAndRectangle(Vector2<int>(0,0), Vector2<int>(Camera::WIDTH-1, Camera::HEIGHT-1), lineBelowHorizon, beginPoint, endPoint))
    {
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), ColorRGB(ColorClasses::white));
      Geometry::calculatePointOnField(beginPoint.x,beginPoint.y, *this, pointOnField[4]);
      Geometry::calculatePointOnField(endPoint.x, endPoint.y, *this, pointOnField[5]);
    }

    // determine the boundary of all the points that were projected to the ground
    Boundary<int> boundary(-10000, +10000);
    if(pointOnField[0].x !=0 || pointOnField[0].y != 0) {boundary.add(pointOnField[0]); }
    if(pointOnField[1].x !=0 || pointOnField[1].y != 0) {boundary.add(pointOnField[1]); }
    if(pointOnField[2].x !=0 || pointOnField[2].y != 0) {boundary.add(pointOnField[2]); }
    if(pointOnField[3].x !=0 || pointOnField[3].y != 0) {boundary.add(pointOnField[3]); }
    if(pointOnField[4].x !=0 || pointOnField[4].y != 0) {boundary.add(pointOnField[4]); }
    if(pointOnField[5].x !=0 || pointOnField[5].y != 0) {boundary.add(pointOnField[5]); }

    // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
    int spacing = 100;
    for(int xx = boundary.x.min - boundary.x.min%spacing + spacing ; xx <= boundary.x.max; xx+=spacing)
    {
      Geometry::calculatePointInImage(Vector3<>((float) xx, (float) boundary.y.min, 0), *this,  beginPoint);
      Geometry::calculatePointInImage(Vector3<>((float) xx, (float) boundary.y.max, 0), *this,  endPoint);
      int lineWidth = 0;
      if(xx == 0) lineWidth = 3;
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), ColorRGB(ColorClasses::yellow));
    }
    for(int yy = boundary.y.min - boundary.y.min%spacing + spacing ; yy <= boundary.y.max; yy+=spacing)
    {
      Geometry::calculatePointInImage(Vector3<>((float) boundary.x.min, (float) yy, 0), *this, beginPoint);
      Geometry::calculatePointInImage(Vector3<>((float) boundary.x.max, (float) yy, 0), *this, endPoint);
      int lineWidth = 0;
      if(yy == 0) lineWidth = 3;
      Draw::Line(img, Point2D(beginPoint.x, beginPoint.y), Point2D(endPoint.x, endPoint.y), ColorRGB(ColorClasses::yellow));
    }
}

RobotCameraMatrix::RobotCameraMatrix(const RobotDimensions& robotDimensions, const float headYaw, const float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera)
{
  *this = RobotCameraMatrix();
  computeRobotCameraMatrix(robotDimensions, headYaw, headPitch, cameraCalibration, upperCamera);
}

void RobotCameraMatrix::computeRobotCameraMatrix(const RobotDimensions& robotDimensions,const float headYaw,const float headPitch,const CameraCalibration& cameraCalibration,bool upperCamera)
{
  *this = RobotCameraMatrix();

  translate(0., 0., robotDimensions.zLegJoint1ToHeadPan);
  rotateZ(headYaw);
  translate(0., 0., 30.5);
  rotateY(-headPitch);
  translate(robotDimensions.xHeadTiltToCamera, 0., robotDimensions.zHeadTiltToCamera);
  rotateY(robotDimensions.headTiltToCameraTilt + cameraCalibration.cameraTiltCorrection);
  rotateX(cameraCalibration.cameraRollCorrection);
  rotateZ(cameraCalibration.cameraPanCorrection);

  //printf("robotcameramatrix translation (x,y,z) =%f,%f,%f\n", translation.x, translation.y, translation.z);
}

CameraMatrix::CameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  *this = CameraMatrix();
  computeCameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
}

void CameraMatrix::computeCameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  (Pose3D&)*this = torsoMatrix;
  translate(cameraCalibration.bodyTranslationCorrection);
  rotateY(cameraCalibration.bodyTiltCorrection);
  rotateX(cameraCalibration.bodyRollCorrection);
  conc(robotCameraMatrix);

  //printf("cameramatrix translation (x,y,z) =%f,%f,%f\n", translation.x, translation.y, translation.z);
}

