/**
* @file BallPercept.cpp
*
* Very simple representation of a seen ball
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#include "BallPercept.h"
#include "DebugDrawings.h"
//#include "Tools/Debugging/DebugDrawings3D.h"

using namespace Robot;

void BallPercept::draw()
{
  //DECLARE_DEBUG_DRAWING("representation:BallPercept:Image", "drawingOnImage");
  //DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  //DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "origin");
  //TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if(ballWasSeen)
  {
      /**bakalan dipake*/
    //Circle(img, Point2D(positionInImage.x,positionInImage.y), radiusInImage, ColorRGB(255,128,64));
    //Circle(img_field, Point2D(relativePositionOnField.x,relativePositionOnField.y), 35, ColorRGB(ColorClasses::orange));
        /**-------*/
    //CIRCLE("representation:BallPercept:Image",positionInImage.x,positionInImage.y,radiusInImage,1,Drawings::ps_solid,ColorClasses::black,Drawings::bs_solid, ColorRGBA(255,128,64,100));
    //CIRCLE("representation:BallPercept:Field",relativePositionOnField.x,relativePositionOnField.y,35,0,Drawings::ps_solid,ColorClasses::orange,Drawings::bs_null,ColorClasses::orange);

    // Sorry, no access to field dimensions here, so ball radius is hard coded
    //SPHERE3D("representation:BallPercept", relativePositionOnField.x, relativePositionOnField.y, 35, 35, ColorClasses::orange);
  }
}
