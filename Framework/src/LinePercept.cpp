/**
* @file LinePercept.h
* Implementation of a class that represents the fieldline percepts
* @author jeff
*/

#include "LinePercept.h"
#include "DebugDrawings.h"
//#include "Tools/Debugging/DebugDrawings3D.h"
#include "Geometry.h"

using namespace Robot;

void LinePercept::InitPercept()
{
  std::vector<Line> lines;
  std::vector<LineSegment> singleSegs;
/*
  lines.reserve(this->lines.size());
  for(std::list<Line>::const_iterator i = this->lines.begin(); i != this->lines.end(); ++i)
  {
    lines.push_back(*i);
    printf("ada line masuk\n");
  }
  singleSegs.reserve(this->singleSegs.size());
  for(std::list<LineSegment>::const_iterator i = this->singleSegs.begin(); i != this->singleSegs.end(); ++i)
  {
    singleSegs.push_back(*i);
    printf("ada singlesegs masuk\n");
  }
*/

  this->lines.clear();
  for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    this->lines.push_back(*i);
  this->singleSegs.clear();
  for(std::vector<LineSegment>::const_iterator i = singleSegs.begin(); i != singleSegs.end(); ++i)
    this->singleSegs.push_back(*i);

}

/*
void LinePercept::serialize(In* in, Out* out)
{
  std::vector<Line> lines;
  std::vector<LineSegment> singleSegs;
  if(out)
  {
    lines.reserve(this->lines.size());
    for(std::list<Line>::const_iterator i = this->lines.begin();i != this->lines.end(); ++i)
      lines.push_back(*i);
    singleSegs.reserve(this->singleSegs.size());
    for(std::list<LineSegment>::const_iterator i = this->singleSegs.begin();i != this->singleSegs.end(); ++i)
      singleSegs.push_back(*i);
  }
  STREAM_REGISTER_BEGIN();
  STREAM_VECTOR(lines);
  STREAM_VECTOR(intersections);
  STREAM_VECTOR(singleSegs);
  STREAM(circle);
  STREAM_REGISTER_FINISH();
  if(in)
  {
    this->lines.clear();
    for(std::vector<Line>::const_iterator i = lines.begin();i != lines.end(); ++i)
      this->lines.push_back(*i);
    this->singleSegs.clear();
    for(std::vector<LineSegment>::const_iterator i = singleSegs.begin();i != singleSegs.end(); ++i)
      this->singleSegs.push_back(*i);
  }
}
*/

Vector2<int> LinePercept::Line::calculateClosestPointOnLine(const Vector2<int>& p) const
{
  const Vector2<> p_double = Vector2<>((float) p.x, (float) p.y);
  const Vector2<> normale = Vector2<>(cosf(alpha+pi), sinf(alpha+pi));
  const Vector2<> offset = normale * calculateDistToLine(p);
  return p + Vector2<int>((int)offset.x, (int)offset.y);
}

int LinePercept::getClosestLine(Vector2<int> point, Line& retLine) const
{
  std::list<LinePercept::Line>::const_iterator closestLine = lines.end();
  int minDist = -1;
  for(std::list<LinePercept::Line>::const_iterator l1 = lines.begin(); l1 != lines.end(); l1++)
    {
      const int dist = (int)fabs(l1->calculateDistToLine(point));
      if(dist < minDist || minDist == -1)
        {
          closestLine = l1;
          minDist = dist;
        }
    }

  if(minDist != -1)
    retLine = *closestLine;
  return minDist;
}

void LinePercept::clear()
{
  lines.clear();
  singleSegs.clear();
  intersections.clear();
  circle.found = false;
}

void LinePercept::drawOnField(const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified) const
{
}

void LinePercept::drawOnImage(Image* img, const CameraMatrix& theCameraMatrix, const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified, const ImageCoordinateSystem& theImageCoordinateSystem) const
{
  //draw the line found
  for(std::list<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      if(Geometry::calculatePointInImage(line->first, theCameraMatrix, p1) &&
          Geometry::calculatePointInImage(line->last, theCameraMatrix, p2))
        {

          //const Drawings::PenStyle pen = line->midLine ? Drawings::ps_dash : Drawings::ps_solid;
          Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedApprox(p1);
          Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedApprox(p2);
          //LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 3, pen, ColorClasses::red);
          //printf("p1(%i,%i) , p2(%i,%i) , uncor1(%.1f,%.1f) , uncor2(%.1f,%.1f)\n",p1.x, p1.y, p2.x,p2.y, uncor1.x, uncor1.y, uncor2.x, uncor2.y);
          Draw::Line(img, Point2D(uncor1.x, uncor1.y), Point2D(uncor2.x, uncor2.y), ColorRGB(ColorClasses::yellow));
        }
    }

  //draw circle if it found
  if(circle.found)
    {
      Vector2<int> p1;
      if(Geometry::calculatePointInImage(circle.pos, theCameraMatrix, p1))
        {
          //CROSS("representation:LinePercept:Image", p1.x, p1.y, 5, 3, Drawings::ps_solid, ColorClasses::robotBlue);
          Draw::Cross(img, Point2D(p1.x, p1.y), 30, ColorRGB(ColorClasses::yellow));
          printf("circle pos in image = %i,%i\n", p1.x, p1.y);
        }
      const float stepSize = 0.4f;
      for(float i = 0; i < pi2; i += stepSize)
        {
          Vector2<int> p1;
          Vector2<int> p2;
          if(Geometry::calculatePointInImage(circle.pos + Vector2<int>(theFieldDimensions.centerCircleRadius+circleBiggerThanSpecified, 0).rotate(i), theCameraMatrix, p1) &&
              Geometry::calculatePointInImage(circle.pos + Vector2<int>(theFieldDimensions.centerCircleRadius+circleBiggerThanSpecified, 0).rotate(i+stepSize), theCameraMatrix, p2))
            {
              Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedApprox(p1);
              Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedApprox(p2);
              //LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 3, Drawings::ps_solid, ColorClasses::blue);
              Draw::Line(img, Point2D(uncor1.x, uncor1.y), Point2D(uncor2.x, uncor2.y), ColorRGB(ColorClasses::yellow));
            }
        }
    }

  for(std::list<LineSegment>::const_iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      if(Geometry::calculatePointInImage(seg->p1, theCameraMatrix, p1) &&
          Geometry::calculatePointInImage(seg->p2, theCameraMatrix, p2))
        {
          Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedApprox(p1);
          Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedApprox(p2);
          //LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 2, Drawings::ps_solid, ColorClasses::orange);
          Draw::Line(img, Point2D(uncor1.x, uncor1.y), Point2D(uncor2.x, uncor2.y), ColorRGB(ColorClasses::orange));
        }
    }

  for(std::vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      Vector2<int> p3;
      Vector2<int> p4;
      const Vector2<int> dir1Int = Vector2<int>(int(inter->dir1.x*100), int(inter->dir1.y*100));
      const Vector2<int> dir2Int = Vector2<int>(int(inter->dir2.x*100), int(inter->dir2.y*100));
      switch(inter->type)
        {
        case Intersection::X:
          Geometry::calculatePointInImage(inter->pos - dir1Int, theCameraMatrix, p1);
          Geometry::calculatePointInImage(inter->pos + dir1Int, theCameraMatrix, p2);
          Geometry::calculatePointInImage(inter->pos - dir2Int, theCameraMatrix, p3);
          Geometry::calculatePointInImage(inter->pos + dir2Int, theCameraMatrix, p4);
	  printf("Intersection X Found !\n ");
          break;
        case Intersection::T:
          Geometry::calculatePointInImage(inter->pos, theCameraMatrix, p1);
          Geometry::calculatePointInImage(inter->pos + dir1Int, theCameraMatrix, p2);
          Geometry::calculatePointInImage(inter->pos - dir2Int, theCameraMatrix, p3);
          Geometry::calculatePointInImage(inter->pos + dir2Int, theCameraMatrix, p4);
	  printf("Intersection T Found !\n ");
          break;
        case Intersection::L:
          Geometry::calculatePointInImage(inter->pos, theCameraMatrix, p1);
          Geometry::calculatePointInImage(inter->pos + dir1Int, theCameraMatrix, p2);
          Geometry::calculatePointInImage(inter->pos, theCameraMatrix, p3);
          Geometry::calculatePointInImage(inter->pos + dir2Int, theCameraMatrix, p4);
	  printf("Intersection L Found !\n ");

          break;
        }

      Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedApprox(p1);
      Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedApprox(p2);
      Vector2<> uncor3 = theImageCoordinateSystem.fromCorrectedApprox(p3);
      Vector2<> uncor4 = theImageCoordinateSystem.fromCorrectedApprox(p4);
      //ARROW("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 3, Drawings::ps_solid, ColorClasses::robotBlue);
      //ARROW("representation:LinePercept:Image", uncor3.x, uncor3.y, uncor4.x, uncor4.y, 3, Drawings::ps_solid, ColorClasses::blue);
      Draw::Arrow(img, Point2D(uncor1.x, uncor1.y), Point2D(uncor2.x, uncor2.y), ColorRGB(ColorClasses::yellow));
      Draw::Arrow(img, Point2D(uncor3.x, uncor3.y), Point2D(uncor4.x, uncor4.y), ColorRGB(ColorClasses::yellow));
    }
}

void LinePercept::drawIn3D(const FieldDimensions& theFieldDimensions, int circleBiggerThanSpecified) const
{
}

