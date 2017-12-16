/**
* @file ImageCoordinateSystem.h
* Declaration of a class that provides transformations on image coordinates.
* Parts of this class were copied from class ImageInfo.
*/

#ifndef _IMAGECOORDINATESYSTEM_H_
#define _IMAGECOORDINATESYSTEM_H_

#include "Camera.h"
#include "Common.h"
#include "Matrix2x2.h"

namespace Robot{
/**
* @class ImageCoordinateSystem
* A class that provides transformations on image coordinates.
*/
class ImageCoordinateSystem
{
public:
  /**
  * The rotation from a horizon-aligned coordinate system to the image coordinate
  * system. The horizon-aligned coordinate system is defined as follows:
  *  - the x-coordinate points parallel to the horizon to the right
  *  - the y-coordinate points perpendicular to the horizon downwards
  *  - the origin is the top left corner of the image, i.e. the same as the the origin
  *    of the image coordinate system. Thus the transformation from horizon-aligned to
  *    image coordinates only requires the rotation matrix.
  * The direction of the horizon is c[0], the direction downwards is c[1].
  */
  Matrix2x2<> rotation;
  Matrix2x2<> invRotation; /**< The rotation from the image coordinates to the horizon-aligned coordinates. */
  Vector2<> origin; /**< The origin of the horizon in image coordinates. */
  Vector2<> offset; /**< The offset of the previous image to the current one. */
  float a, /**< Constant part of equation to motion distortion. */
        b; /**< Linear part of equation to motion distortion. */
  Vector2<int> offsetInt; /**< The offset of the previous image to the current one. */
  int aInt,
      bInt;
  //CameraInfo cameraInfo; /**< Information required in some equations. */

  static int xTable[640],
             yTable[480],
             table[6144];
/*  static int xTable[Camera::WIDTH],
             yTable[Camera::HEIGHT],
             table[6144];
*/

  static void initTables()
  {
    for(int i = 0; i < Camera::WIDTH; ++i)
      xTable[i] = int(::atanf((Camera::WIDTH/2 - i) / Camera::focalLength) * 1024 + 0.5f);
    for(int i = 0; i < Camera::HEIGHT; ++i)
      yTable[i] = int(::atanf((i - Camera::HEIGHT/2) / Camera::focalLength) * 1024 + 0.5f);
    for(int i = -3072; i < 3072; ++i)
      table[i + 3072] = int(::tanf(i / 1024.0f) * Camera::focalLength + 0.5f);
  }


  /**
  * Converts image coordintates into coordinates in the horizon-aligned coordinate system.
  * @param imageCoords The point in image coordinates.
  * @return The point in horizon-aligned coordinates.
  */
  Vector2<> toHorizonAligned(const Vector2<int>& imageCoords) const
  {
    return invRotation * Vector2<>((float) imageCoords.x, (float) imageCoords.y);
  }


  /**
  * Converts image coordintates into coordinates in the horizon-aligned coordinate system.
  * @param imageCoords The point in image coordinates.
  * @return The point in horizon-aligned coordinates.
  */
  Vector2<> toHorizonAligned(const Vector2<>& imageCoords) const
  {
    return invRotation * imageCoords;
  }

  /**
  * Converts coordinates in the horizon-aligned coordinate system into image coordinates.
  * No clipping is done.
  * @param horizonAlignedCoords The point in horizon-aligned coordinates.
  * @return The point in image coordinates.
  */
  Vector2<int> fromHorizonAligned(const Vector2<>& horizonAlignedCoords) const
  {
    Vector2<> result = rotation * horizonAlignedCoords;
    return Vector2<int>(int(result.x), int(result.y));
  }


  /**
  * Converts image coordintates into coordinates in the horizon-based coordinate system,
  * i.e. a system of coordinates, in which the origin of the horizon is mapped to (0, 0).
  * @param imageCoords The point in image coordinates.
  * @return The point in horizon-based coordinates.
  */
  Vector2<> toHorizonBased(const Vector2<int>& imageCoords) const
  {
    return invRotation * (Vector2<>((float) imageCoords.x, (float) imageCoords.y) - origin);
  }

  /**
  * Converts image coordintates into coordinates in the horizon-based coordinate system,
  * i.e. a system of coordinates, in which the origin of the horizon is mapped to (0, 0).
  * @param imageCoords The point in image coordinates.
  * @return The point in horizon-based coordinates.
  */
  Vector2<> toHorizonBased(const Vector2<>& imageCoords) const
  {
    return invRotation * (imageCoords - origin);
  }

  /**
  * Converts coordinates in the horizon-based coordinate system into image coordinates.
  * No clipping is done.
  * @param horizonBasedCoords The point in horizon-based coordinates.
  * @return The point in image coordinates.
  */
  Vector2<int> fromHorizonBased(const Vector2<>& horizonBasedCoords) const
  {
    Vector2<> result = rotation * horizonBasedCoords;
    return Vector2<int>(int(result.x + origin.x), int(result.y + origin.y));
  }

  /**
  * Corrects image coordinates so that the distortion resulting from the rolling
  * shutter is compensated.
  * No clipping is done.
  * @param imageCoords The point in image coordinates.
  * @return The corrected point.
  */
  Vector2<> toCorrected(const Vector2<>& imageCoords) const
  {
    float factor = a + imageCoords.y * b;
    return Vector2<>(float(Camera::WIDTH/2 - tanf(atanf((Camera::WIDTH/2 - imageCoords.x) / Camera::focalLength) - factor * offset.x) * Camera::focalLength),
                     float(Camera::HEIGHT/2 + tanf(atanf((imageCoords.y - Camera::HEIGHT/2) / Camera::focalLength) - factor * offset.y) * Camera::focalLength));
  }

  /**
  * Corrects image coordinates so that the distortion resulting from the rolling
  * shutter is compensated.
  * No clipping is done.
  * @param imageCoords The point in image coordinates.
  * @return The corrected point.
  */
  inline Vector2<> toCorrected(const Vector2<int>& imageCoords) const
  {
    return toCorrected(Vector2<float>(float(imageCoords.x), float(imageCoords.y)));
  }

  /**
  * "Incorrects" image coordinates so that the result contains an approximation of distortion from the rolling shutter.
  * @param coords The point in corrected image coordinates.
  * @return The incorrected point.
  */
  Vector2<> fromCorrectedApprox(const Vector2<>& coords) const
  {
    float factor = a + Camera::HEIGHT/2 * b;
    Vector2<> v(factor * offset.x - atanf((coords.x - Camera::WIDTH/2) / Camera::focalLength),
                factor * offset.y + atanf((coords.y - Camera::HEIGHT/2) / Camera::focalLength));
    const static float EPSILON = 0.1f;
    if(v.x < (float) -pi_2 + EPSILON)
      v.x = (float) -pi_2 + EPSILON;
    else if(v.x > pi_2 - EPSILON)
      v.x = pi_2 - EPSILON;

    if(v.y < (float) -pi_2 + EPSILON)
      v.y = (float) -pi_2 + EPSILON;
    else if(v.y > pi_2 - EPSILON)
      v.y = pi_2 - EPSILON;

    return Vector2<>(Camera::WIDTH/2 - tanf(v.x) * Camera::focalLength,
                     Camera::HEIGHT/2 + tanf(v.y) * Camera::focalLength);
  }

  /**
  * "Incorrects" image coordinates so that the result contains an approximation of distortion from the rolling shutter.
  * @param coords The point in corrected image coordinates.
  * @return The incorrected point.
  */
  inline Vector2<> fromCorrectedApprox(const Vector2<int>& coords) const
  {
    return fromCorrectedApprox(Vector2<float>(float(coords.x), float(coords.y)));
  }

  /**
  * Corrects image coordinates so that the distortion resulting from the rolling
  * shutter is compensated.
  * No clipping is done.
  * @param x The x coordinate of the point in image coordinates.
  * @param y The y coordinate of the point in image coordinates.
  * @return The corrected point relative to the image center with negated signs.
  */
  Vector2<int> toCorrectedCenteredNeg(int x, int y) const
  {
    int factor = aInt + y * bInt;
    x = xTable[x] - ((factor * offsetInt.x) >> 10);
    y = yTable[y] - ((factor * offsetInt.y) >> 10);
    if(x < -3072)
      x = -3072;
    else if(x > 3071)
      x = 3071;
    if(y < -3072)
      y = -3072;
    else if(y > 3071)
      y = 3071;
    return Vector2<int>(table[x + 3072], -table[y + 3072]);
  }

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  /*
    STREAM_REGISTER_BEGIN();
    STREAM(rotation);
    STREAM(invRotation);
    STREAM(origin);
    STREAM(offset);
    STREAM(a);
    STREAM(b);
    STREAM_REGISTER_FINISH();
    if(in)
    {
      offsetInt = Vector2<int>(int(offset.x * 1024 + 0.5f),
                               int(offset.y * 1024 + 0.5f));
      aInt = int(a * 1024 + 0.5f);
      bInt = int(b * 1024 + 0.5f);
    }
    */
};
}
#endif //ImageCoordinateSystem_H
