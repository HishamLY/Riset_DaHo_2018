/**
* @file LinePercept.h
* Declaration of a class that represents the fieldline percepts
* @author jeff
*/

#ifndef __LINEPERCEPT_H_
#define __LINEPERCEPT_H_


#include <list>
#include <vector>
#include "Vector2.h"
#include "ImageCoordinateSystem.h"
#include "Camera.h"
#include "Image.h"
#include "CameraMatrix.h"
#include "FieldDimensions.h"

namespace Robot{
/**
* @class LinePercept
* A class that represents the found fieldlines, center circle and intersections.
*/
class LinePercept
{
private:

public:
  /**
   * @class Linesegment
   * This class represents a linesegment generated from a lienspot.
   */
  class LineSegment
  {
  private:

  public:
    float alpha; /**< direction of representation in Hesse norm form of this linesegment */
    float d; /**< distance of representation in Hesse norm form of this linesegment */
    Vector2<int> p1, /**< start/end point of this linesegment in field coordinates */
                 p2, /**< start/end point of this linesegment in field coordinates */
                 p1Img, /**< start/end point of this linesegment in image coordinates */
                 p2Img; /**< start/end point of this linesegment in image coordinates */
   int novar; /**testing variable*/
  };

  /**
   * @class Line
   * This class represents a found fieldline.
   */
  class Line
  {
  private:

  public:
    float alpha; /**< direction of this line in Hesse norm form */
    float d; /**< distance of this line in Hesse norm form */
    bool dead; /**< This is needed for merging lines */
    bool midLine; /**< Whether this is the line throught the center circle */
    std::vector<LineSegment> segments; /**< The linesegments forming this line */
    Vector2<int> first, last; /**< The starting/end point of this line in field coordinates */

    /**
     * Calculates the distance of a point p the this line
     * @param p a point
     * @return the distance
     */
    float calculateDistToLine(const Vector2<int>& p) const
    {
      return p.x * cosf(alpha) + p.y * sinf(alpha) - d;
    }

    /**
     * Calculates the closest point to a point on this line
     * @param p a point
     * @return the closest point on this line
     */
    Vector2<int> calculateClosestPointOnLine(const Vector2<int>& p) const;
  };

  /**
   * @class CircleSpot
   *
   * This class represents circle spots. A circle spot
   * is a point calculated from a linesegment where the
   * center circle would be if the linesegment is part
   * of the center circle.
   * This is also used for the found circle.
   */
  class CircleSpot
  {
  private:

  public:
    CircleSpot():found(false),lastSeen(0) {}

    Vector2<int> pos; /**< The position of the center of the center circle in field coordinates */
    bool found; /**< Whether the center circle was found in this frame */
    unsigned long lastSeen; /**< The last time the center circle was seen */
    std::list<LineSegment>::iterator iterator; /**< An iterator pointing to the according segment
                                                    in the singleSegs list */
  };

  /**
   * @class Intersection
   * A class representing a intersection of two fieldlines
   */
  class Intersection
  {
  private:

  public:
    enum IntersectionType
    {
      L,
      T,
      X,
    };
    IntersectionType type;
    Vector2<int> pos; /**< The fieldcoordinates of the intersection */
    Vector2<> dir1, dir2; /**< The directions of the lines intersected. */
  };

  std::list<Line> lines; /**<The found fieldlines */
  std::vector<Intersection> intersections; /**< The intersections of the lines */
  std::list<LineSegment> singleSegs; /**< Line segments which could not be clustered to a line */
  CircleSpot circle; /**< The position of the center circle if found */

  /**Clear the percept */
  void clear();

/** Assign linepercept value*/
void InitPercept();

  //variabel iseng
  int i;

  /** Determines the closest line to a given point
   * @param point the given point
   * @param retLine the closest line
   * @return the distance from point to retLine
   * */
  int getClosestLine(Vector2<int> point, Line& retLine) const;

  /**
  * The method draws the line percepts on the field.
  */
  void drawOnField(const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified) const;

  /**
  * The method draws the line percepts on the image.
  */
  //void drawOnImage(const CameraMatrix& theCameraMatrix, const CameraInfo& theCameraInfo, const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified, const ImageCoordinateSystem& theImageCoordinateSystem) const;
  void drawOnImage(Image* img, const CameraMatrix& theCameraMatrix, const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified, const ImageCoordinateSystem& theImageCoordinateSystem) const;

  /**
  * The method draws the line percepts in the 3D View.
  */
  void drawIn3D(const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified) const;
};
}
#endif //__LinePercept_h_

