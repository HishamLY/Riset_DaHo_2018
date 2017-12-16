/**
* @file BallPerceptor.h
* This file declares a module that provides the ball percept.
* @author Colin Graf
*/

#ifndef BallPerceptor_H
#define BallPerceptor_H

#include "FieldDimensions.h"
#include "Camera.h"
#include "CameraMatrix.h"
#include "ImageCoordinateSystem.h"
#include "BallSpots.h"
#include "BallPercept.h"

namespace Robot{
class BallPerceptor
{
public:
  /** A collection of parameters for the ball perceptor.*/
  class Parameters
  {
  public:
    Parameters() {}
    float clippingApproxRadiusScale;
    float clippingApproxRadiusPixelBonus;
    unsigned int scanMaxColorDistance;
    unsigned int scanPixelTolerance;
    unsigned int refineMaxPixelCount;
    unsigned int refineMaxColorDistance;
    float checkMaxRadiusDifference;
    float checkMinRadiusDifference;
    float checkMinRadiusPixelBonus;
    float checkOutlineRadiusScale;
    float checkOutlineRadiusPixelBonus;
  };

  class BallPoint
  {
  public:
    Vector2<int> step;
    Vector2<int> start;
    Vector2<int> point;
    Vector2<> pointf;
    bool atBorder;
    bool isValid;

    BallPoint() : atBorder(false), isValid(false) {}
  };

  Parameters p; /**< Parameters for the module. */
  float sqrMaxBallDistance; /**< The square of the maximal allowed ball distance. */

  FieldDimensions theFieldDimensions;
  CameraMatrix theCameraMatrix;
  ImageCoordinateSystem theImageCoordinateSystem;
  BallSpots theBallSpots;

  void init(FieldDimensions &FieldDim, BallSpots& ballSpots,CameraMatrix &CameraMat, ImageCoordinateSystem& ImageCoordinate);
  void update(BallPercept& ballPercept);


  bool checkBallSpot(const BallSpot& ballSpot);
  float approxRadius1; /**< Bearing based approximation of the radius. */

  bool searchBallPoints(const BallSpot& ballSpot);

  /** Image::Pixel ini gak ada di Darwin. Modif !! */
  //Image::Pixel startPixel; /**< The ball spot pixel. */


  BallPoint ballPoints[8]; /**< Points on the outer edge of the ball. */
  Vector2<int> approxCenter2;
  int totalPixelCount;
  int totalCbSum;
  int totalCrSum;

  //bool searchBallPoint(const Vector2<int>& start, Image::Pixel startPixel, const Vector2<int>& step, float maxLength, BallPoint& ballPoint);

  bool checkBallPoints();
  bool getBallFromBallPoints(Vector2<>& center, float& radius) const;
  unsigned int validBallPoints; /**< Count of usable points on the outer edge of the ball. */

  bool calculateBallInImage();
  Vector2<> center;     /**< Center of the ball in image. */
  float radius;         /**< Radius of the ball in image. */

  bool checkBallInImage();

  bool calculateBallOnField();
  Vector3<> sizeBasedCenterOnField;
  Vector3<> bearingBasedCenterOnField;
  Vector3<> usedCenterOnField;

  bool checkBallOnField();

};
}
#endif// __BallPerceptor_h_
