/**
* @file LineSpots.h
* Declaration of a class that represents a spot that  indicates a line.
* @author jeff
*/

#ifndef _LINESPOTS_H_
#define _LINESPOTS_H_

#include "DebugDrawings.h"
#include "Image.h"
#include "Vector2.h"
#include "Common.h"
#include <vector>

namespace Robot
{
/**
* @class LineSpots
* This class contains all the linespots and nonlinesposts (=white regions which are no lines)
*/
class LineSpots
{
private:

public:

  /**
   * @class LineSpot
   * A class that represents a spot that's an indication of a line.
   */
  class LineSpot
  {
  public:
    int x_s, /**< "schwerpunkt_x" */
    y_s; /**< "schwerpunkt_y" */
    float alpha, /**< the direction/rotation of the region    | */
    alpha_len, /**< "ausbreitung entlang alpha"         |-> Haupttraegheitsachsenbla */
    alpha_len2; /**< "ausbreitung orthogonal zu alpha"  | */
    Vector2<int> p1, p2; /**< The starting/end point of this linespot in image coordinates*/
  };

  /**
   * @class NonLineSpot
   * This class represents a white region which is no line
   */
  class NonLineSpot
  {
  private:

  public:
    Vector2<int> p1, p2; /**< start/end point of this spot in image coordinates */
    int size; /**< The size of the coresponding region in the image */
  };

  std::vector<LineSpot> spots; /**< All the line spots */
  std::vector<NonLineSpot> nonLineSpots; /**< All the non line spots (= white regions which are no lines)*/

  /**
  * The method draws all line spots.
  */
  //void draw() const
  void draw(Image* img)
  {
    //Draw the NonLineSpots
    for(std::vector<LineSpots::NonLineSpot>::const_iterator i = nonLineSpots.begin(); i != nonLineSpots.end(); ++i)
      {
        //Draw::Arrow(img, Point2D(i->p1.x, i->p1.y), Point2D(i->p2.x, i->p2.y), ColorRGB(ColorClasses::yellow)); //awalnya blue
      }

    //Draw the linespots
    for(std::vector<LineSpots::LineSpot>::const_iterator i = spots.begin(); i != spots.end(); ++i)
      {
        //Draw::Line(img, Point2D(i->x_s, i->y_s), Point2D(i->x_s + (int)(cosf(i->alpha+pi_2) * i->alpha_len2), i->y_s + (int)(sinf(i->alpha+pi_2)*i->alpha_len2)), ColorRGB(ColorClasses::black));
        //Draw::Arrow(img, Point2D(i->p1.x, i->p1.y), Point2D(i->p2.x, i->p2.y), ColorRGB(ColorClasses::black));
      }
  }

   int count;
};
}
#endif //__LineSpots_h_
