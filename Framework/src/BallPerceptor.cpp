/**
* @file BallPerceptor.cpp
* This file declares a module that provides a ball percept without using color tables.
* The ball center / radius calculation algorithm is based on the BallSpecialist in GT2005.
* @author Colin Graf
*/

#include "BallPerceptor.h"
#include "DebugDrawings.h"
#include "Matrix_nxn.h"
#include <stdio.h>

using namespace Robot;

void BallPerceptor::init(FieldDimensions &FieldDim, BallSpots& ballSpots, CameraMatrix &CameraMat, ImageCoordinateSystem& ImageCoordinate)
{
  theFieldDimensions = FieldDim;
  theBallSpots = ballSpots;
  theCameraMatrix = CameraMat;
  theImageCoordinateSystem = ImageCoordinate;

  sqrMaxBallDistance = float(Vector2<int>(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder,
  theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder).squareAbs());
  p.clippingApproxRadiusScale = 2.f;
  p.clippingApproxRadiusPixelBonus = 2.5f;
  p.scanMaxColorDistance = 30; //18;
  p.scanPixelTolerance = 2;
  p.refineMaxPixelCount = 2;
  p.refineMaxColorDistance = 35;
  p.checkMaxRadiusDifference = 1.6f;
  p.checkMinRadiusDifference = 0.9f;
  p.checkMinRadiusPixelBonus = 6.f;
  p.checkOutlineRadiusScale = 1.1f;
  p.checkOutlineRadiusPixelBonus = 2.f;
  //if(SystemCall::getMode() == SystemCall::simulatedRobot)
    p.scanMaxColorDistance = 35;
}

void BallPerceptor::update(BallPercept& ballPercept)
{
  //MODIFY("module:BallPerceptor:parameters", p);

  //DECLARE_DEBUG_DRAWING("module:BallPerceptor:image", "drawingOnImage");
  //DECLARE_DEBUG_DRAWING("module:BallPerceptor:field", "drawingOnField");
  //DECLARE_PLOT("module:BallPerceptor:angle");

  for(std::vector<BallSpot>::const_iterator it = theBallSpots.ballSpots.begin(), end = theBallSpots.ballSpots.end(); it != end; ++it)
  {
    const BallSpot& ballSpot = *it;

    // TODO: prefer large spots
/*
    if(!checkBallSpot(ballSpot)) // step 1
      continue;
    if(!searchBallPoints(ballSpot)) // step 2
      continue;
    if(!checkBallPoints()) // step 3
      continue;
    if(!calculateBallInImage()) // step 4
      continue;
    if(!checkBallInImage()) // step 5
      continue;
    if(!calculateBallOnField()) // step 6
      continue;
    if(!checkBallOnField()) // step 7
      continue;
*/
    ballPercept.positionInImage = center;
    ballPercept.radiusInImage = radius;
    ballPercept.ballWasSeen = true;
    ballPercept.relativePositionOnField = Vector2<>(usedCenterOnField.x, usedCenterOnField.y);

    //PLOT("module:BallPerceptor:angle", toDegrees(ballPercept.relativePositionOnField.angle()));

    return;
  }
  ballPercept.ballWasSeen = false;
}

bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot)
{
  // calculate an approximation of the radius based on bearing distance of the ball spot
  const Vector2<int>& spot = ballSpot.position;
  Vector2<> correctedStart = theImageCoordinateSystem.toCorrected(spot);
  Vector3<> cameraToStart(Camera::focalLength, Camera::WIDTH/2 - correctedStart.x, Camera::HEIGHT/2 - correctedStart.y);
  Vector3<> unscaledField = theCameraMatrix.rotation * cameraToStart;
  if(unscaledField.z >= 0.f)
    return false; // above horizon
  const float scaleFactor = (theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / unscaledField.z;
  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  if(Vector2<>(unscaledField.x, unscaledField.y).squareAbs() > sqrMaxBallDistance)
    return false; // too far away
  cameraToStart.y += cameraToStart.y > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  approxRadius1 = fabs(theImageCoordinateSystem.fromCorrectedApprox(Vector2<int>(int(Camera::WIDTH/2 - cameraToStart.y), int(Camera::HEIGHT/2 - cameraToStart.z))).x - spot.x);

  return true;
}

bool BallPerceptor::searchBallPoints(const BallSpot& ballSpot)
{
  Vector2<int> start = ballSpot.position;
  const float approxDiameter = approxRadius1 * p.clippingApproxRadiusScale + p.clippingApproxRadiusPixelBonus;
  int halfApproxRadius = int(approxRadius1 * 0.5f);

  //CROSS("module:BallPerceptor:image", start.x, start.y, 2, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, 0x6a));
  //CIRCLE("module:BallPerceptor:image", start.x, start.y, approxDiameter, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, 0x6a), Drawings::bs_null, ColorRGBA());

  // try to improve the start point
  int maxColorDistance = p.scanMaxColorDistance;
  int resolutionWidth = Camera::WIDTH;
  int resolutionHeight = Camera::HEIGHT;
  startPixel = theImage.image[start.y][start.x];
  Vector2<int> preScanPoints[4] = {
    Vector2<int>(start.x + halfApproxRadius, start.y + halfApproxRadius),
    Vector2<int>(start.x - halfApproxRadius, start.y - halfApproxRadius),
    Vector2<int>(start.x + halfApproxRadius, start.y - halfApproxRadius),
    Vector2<int>(start.x - halfApproxRadius, start.y + halfApproxRadius)
  };
  bool preScanResults[4];
  for(int i = 0; i < 4; ++i)
  {
    if(preScanPoints[i].x < 0 || preScanPoints[i].x >= resolutionWidth ||
      preScanPoints[i].y < 0 || preScanPoints[i].y >= resolutionHeight)
    {
      i -= i % 2;
      preScanResults[i++] = false;
      preScanResults[i] = false;
    }
    else
    {
      const Image::Pixel& pixel = theImage.image[preScanPoints[i].y][preScanPoints[i].x];
      preScanResults[i] = abs(startPixel.cb - pixel.cb) + abs(startPixel.cr - pixel.cr) < maxColorDistance;
    }
  }
  if(preScanResults[0] != preScanResults[1] && preScanResults[2] != preScanResults[3])
  {
    start = Vector2<int>();
    if(preScanResults[0])
      start += preScanPoints[0];
    else
      start += preScanPoints[1];
    if(preScanResults[2])
      start += preScanPoints[2];
    else
      start += preScanPoints[3];
    start /= 2;
  }
  else if(preScanResults[0] != preScanResults[1])
    start = preScanResults[0] ? preScanPoints[0] : preScanPoints[1];
  else if(preScanResults[2] != preScanResults[3])
    start = preScanResults[2] ? preScanPoints[2] : preScanPoints[3];

  //CROSS("module:BallPerceptor:image", start.x, start.y, 2, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
  //CIRCLE("module:BallPerceptor:image", start.x, start.y, approxDiameter, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0), Drawings::bs_null, ColorRGBA());

  // prepare scans
  totalPixelCount = 0;
  totalCbSum = 0;
  totalCrSum = 0;

  // vertical scan
  if(!searchBallPoint(start, startPixel, Vector2<int>(0, 1), approxDiameter, ballPoints[0]) ||
    !searchBallPoint(start, startPixel, Vector2<int>(0, -1), approxDiameter, ballPoints[4]))
    return false;
  if(ballPoints[0].atBorder && ballPoints[4].atBorder)
    return false; // too large
  else if(ballPoints[0].atBorder)
  {
    start.y = ballPoints[4].point.y + int(approxRadius1);
    if(start.y > ballPoints[0].point.y - 1)
      start.y = ballPoints[0].point.y - 1;
  }
  else if(ballPoints[4].atBorder)
  {
    start.y = ballPoints[0].point.y - int(approxRadius1);
    if(start.y < ballPoints[4].point.y + 1)
      start.y = ballPoints[4].point.y + 1;
  }
  else
    start.y = (ballPoints[0].point.y + ballPoints[4].point.y) / 2;

  // horizontal scan
  if(!searchBallPoint(start, startPixel, Vector2<int>(1, 0), approxDiameter, ballPoints[2]) ||
    !searchBallPoint(start, startPixel, Vector2<int>(-1, 0), approxDiameter, ballPoints[6]))
    return false;
  if(ballPoints[2].atBorder && ballPoints[6].atBorder)
    return false; // too large
  else if(ballPoints[2].atBorder)
  {
    start.x = ballPoints[6].point.x + int(approxRadius1);
    if(start.x > ballPoints[2].point.x - 1)
      start.x = ballPoints[2].point.x - 1;
  }
  else if(ballPoints[6].atBorder)
  {
    start.x = ballPoints[2].point.x - int(approxRadius1);
    if(start.x < ballPoints[6].point.x + 1)
      start.x = ballPoints[6].point.x + 1;
  }
  else
    start.x = (ballPoints[2].point.x + ballPoints[6].point.x) / 2;
  approxCenter2 = start;

  // maybe repeat vertical and horizontal scans
  int skipArea = std::min(ballPoints[0].point.y - ballPoints[4].point.y, ballPoints[2].point.x - ballPoints[6].point.x) / 4;
  float maxLength = approxDiameter - skipArea;
  if(abs(start.x - ballPoints[0].start.x) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2<int>(0, skipArea), startPixel, Vector2<int>(0, 1), maxLength, ballPoints[0]) ||
      !searchBallPoint(start + Vector2<int>(0, -skipArea), startPixel, Vector2<int>(0, -1), maxLength, ballPoints[4]))
      return false;
  }
  if(abs(start.y - ballPoints[2].start.y) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2<int>(skipArea, 0), startPixel, Vector2<int>(1, 0), maxLength, ballPoints[2]) ||
      !searchBallPoint(start + Vector2<int>(-skipArea, 0), startPixel, Vector2<int>(-1, 0), maxLength, ballPoints[6]))
      return false;
  }

  // diagonal scans
  skipArea = std::min(ballPoints[0].point.y - ballPoints[4].point.y, ballPoints[2].point.x - ballPoints[6].point.x) / 4;
  maxLength = approxDiameter - 1.41421356f * skipArea;
  if(!searchBallPoint(start + Vector2<int>(skipArea, skipArea), startPixel, Vector2<int>(1, 1), maxLength, ballPoints[1]) ||
    !searchBallPoint(start + Vector2<int>(-skipArea, -skipArea), startPixel, Vector2<int>(-1, -1), maxLength, ballPoints[5]) ||
    !searchBallPoint(start + Vector2<int>(skipArea, -skipArea), startPixel, Vector2<int>(1, -1), maxLength, ballPoints[3]) ||
    !searchBallPoint(start + Vector2<int>(-skipArea, skipArea), startPixel, Vector2<int>(-1, 1), maxLength, ballPoints[7]))
    return false;

  // improve ball points
  if(totalPixelCount == 0)
    return false;
  int cbAvg = totalCbSum / totalPixelCount;
  int crAvg = totalCrSum / totalPixelCount;
  int refineMaxPixelCount = p.refineMaxPixelCount;
  int refineMaxColorDistance = p.refineMaxColorDistance;
  for(unsigned j = 0; j < sizeof(ballPoints) / sizeof(*ballPoints); ++j)
  {
    BallPoint& ballPoint = ballPoints[j];
    const Vector2<int>& step = ballPoint.step;
    Vector2<int> pos = ballPoint.point;
    int i = 0;
    for(; i < refineMaxPixelCount; ++i)
    {
      pos += step;
      if(pos.x < 0 || pos.x >= resolutionWidth ||
        pos.y < 0 || pos.y >= resolutionHeight)
        break;
      if(abs(theImage.image[pos.y][pos.x].cb - cbAvg) + abs(theImage.image[pos.y][pos.x].cr - crAvg) > refineMaxColorDistance)
        break;
    }
    if(i)
      ballPoint.point = pos - step;
    ballPoint.pointf = Vector2<float>(float(ballPoint.point.x), float(ballPoint.point.y));
  }

  return true;
}


