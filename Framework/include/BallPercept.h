/**
* @file BallPercept.h
* Very simple representation of a seen ball
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#ifndef __BallPercept_h_
#define __BallPercept_h_

#include "Vector2.h"

namespace Robot
{
class BallPercept
{
public:
  Vector2<> positionInImage;               /**< The position of the ball in the current image */
  float radiusInImage;                    /**< The radius of the ball in the current image */
  bool ballWasSeen;                        /**< Indicates, if the ball was seen in the current image. */
  Vector2<> relativePositionOnField; /**< Ball position relative to the robot. */
  float validity; // what the?
  Vector2<> sizeBasedRelativePosition; /**< Size based ball position relative to the robot. */
  Vector2<> bearingBasedRelativePosition; /**< Bearing based ball position relative to the robot. */

  /** Constructor */
  BallPercept() : ballWasSeen(false) {}

  /** Draws the ball*/
  void draw();

private:
/*
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(positionInImage);
    STREAM(radiusInImage);
    STREAM(ballWasSeen);
    STREAM(relativePositionOnField);
    STREAM(sizeBasedRelativePosition);
    STREAM(bearingBasedRelativePosition);
    STREAM_REGISTER_FINISH();
  }
  */
};
}
#endif// __BallPercept_h_
