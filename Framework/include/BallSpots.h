/**
* @file BallSpots.h
* Declaration of a class that represents a spot that might be an indication of a ball.
* @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
* @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
*
*/

#ifndef _BALLSPOTS_H_
#define _BALLSPOTS_H_

#include "Image.h"
#include "DebugDrawings.h"
#include "Vector2.h"

namespace Robot
{
/**
* @class BallSpot
* A class that represents a spot that might be an indication of a ball.
*/
class BallSpot
{
private:

public:
  Vector2<int> position;
  float eccentricity;

  BallSpot() {}
};

/**
* @class BallSpots
* A class that represents a spot that might be an indication of a ball.
*/
class BallSpots
{
private:

public:
  BallSpots()  {
    ballSpots.reserve(50);
  }

  std::vector<BallSpot> ballSpots;

  void addBallSpot(int x, int y, float eccentricity)
  {
    BallSpot bs;
    bs.position.x = x;
    bs.position.y = y;
    bs.eccentricity = eccentricity;
    ballSpots.push_back(bs);
  }

  /** The method draws all ball spots. */
  void draw(Image* img)
  {
    // Draws the ballspots to the image
    for(std::vector<BallSpot>::const_iterator i = ballSpots.begin(); i != ballSpots.end(); ++i)
      {
        Draw::Cross(img, Point2D(i->position.x, i->position.y), 5, ColorRGB(ColorClasses::orange));
        Draw::Cross(img, Point2D(i->position.x, i->position.y), 3, ColorRGB(ColorClasses::black));
      }
  }
};
}

#endif //__BallSpots_h_
