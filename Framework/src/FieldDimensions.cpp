/**
* @file FieldDimensions.cpp
* Some useful functions regarding field dimensions.
* @author Max Risler (edited by : Imre)
*
*/

#include "FieldDimensions.h"
#include <limits>
#include "Vector2.h"
//#include "Tools/Debugging/Modify.h"
//#include "Tools/Settings.h"
//#include "Tools/Streams/InStreams.h"

using namespace Robot;

FieldDimensions::FieldDimensions()
: Boundary<>(0,0)
{
}

void FieldDimensions::load(minIni* ini)
{
  int* const valuesInt[]={  //array pointer
    &xPosOpponentFieldBorder, &xPosOpponentGoal, &xPosOpponentGoalpost,
    &xPosOpponentGroundline, &xPosOpponentSideCorner, &xPosOpponentPenaltyArea,
    &xPosOpponentPenaltyMark, &xPosHalfWayLine,
    &xPosOwnPenaltyArea, &xPosOwnPenaltyMark, &xPosOwnSideCorner, &xPosOwnGroundline, &xPosOwnGoalpost,
    &xPosOwnGoal, &xPosOwnFieldBorder,
    &yPosLeftFieldBorder, &yPosLeftSideline, &yPosLeftGroundline,
    &yPosLeftPenaltyArea, &yPosLeftGoal, &yPosCenterGoal, &yPosRightGoal,
    &yPosRightPenaltyArea, &yPosRightGroundline, &yPosRightSideline, &yPosRightFieldBorder,
    &centerCircleRadius, &goalHeight, &fieldLinesWidth, &goalPostRadius,
    &xPosThrowInPointOpponentHalf, &xPosThrowInPointCenter,
    &xPosThrowInPointOwnHalf, &ballRadius, &ballFriction
  };
  const char* const namesInt[]={
    "xPosOpponentFieldBorder", "xPosOpponentGoal", "xPosOpponentGoalpost",
    "xPosOpponentGroundline", "xPosOpponentSideCorner", "xPosOpponentPenaltyArea",
    "xPosOpponentPenaltyMark", "xPosHalfWayLine",
    "xPosOwnPenaltyArea", "xPosOwnPenaltyMark", "xPosOwnSideCorner", "xPosOwnGroundline", "xPosOwnGoalpost",
    "xPosOwnGoal", "xPosOwnFieldBorder",
    "yPosLeftFieldBorder", "yPosLeftSideline", "yPosLeftGroundline",
    "yPosLeftPenaltyArea", "yPosLeftGoal", "yPosCenterGoal", "yPosRightGoal",
    "yPosRightPenaltyArea", "yPosRightGroundline", "yPosRightSideline", "yPosRightFieldBorder",
    "centerCircleRadius",
    "goalHeight", "fieldLinesWidth", "goalPostRadius",
    "xPosThrowInPointOpponentHalf", "xPosThrowInPointCenter",
    "xPosThrowInPointOwnHalf", "ballRadius", "ballFriction"
  };

  const int numOfValuesInt = sizeof(valuesInt)/sizeof(int*);

  bool initializedInt[numOfValuesInt];

  for (int i = 0; i < numOfValuesInt; initializedInt[i++] = false);

  ReadDimensionsFromIni(ini, "Dimensions", valuesInt, namesInt, initializedInt, numOfValuesInt);
  readLines();
  readCorners();

  //printf("xPosOpponentFieldBorder = %i\n",ballFriction);

  add(Vector2<>((float) xPosOpponentFieldBorder,(float) yPosLeftFieldBorder));
  add(Vector2<>((float) xPosOwnFieldBorder,(float) yPosRightFieldBorder));
}

void FieldDimensions::ReadDimensionsFromIni(minIni* ini, const std::string &section, int* const* valuesInt,
                                            const char* const* namesInt, bool* initializedInt, int numOfValuesInt)
{
    int value = -2;
    for (int i = 0; i < numOfValuesInt; i++)
    {
            if((value = ini->geti(section, namesInt[i], INVALID_VALUE)) != INVALID_VALUE)
            {
                *valuesInt[i] = value ;
                initializedInt[i] = true;
            }
    }
}

void FieldDimensions::readLines()
{
  LinesTable* linesTables[] = {&carpetBorder, &fieldBorder, &fieldLines};
  const char* linesTableNames[] = {"carpetBorder", "fieldBorder", "fieldLines"};
  const int numOfLinesTables = sizeof(linesTables)/sizeof(LinesTable*);

  /**carpetBorder*/
  linesTables[0]->push(Vector2<>(xPosOpponentFieldBorder,yPosRightFieldBorder),Vector2<>(xPosOpponentFieldBorder,yPosLeftFieldBorder));
  linesTables[0]->push(Vector2<>(xPosOpponentFieldBorder,yPosLeftFieldBorder),Vector2<>(xPosOwnFieldBorder,yPosLeftFieldBorder));
  linesTables[0]->push(Vector2<>(xPosOwnFieldBorder,yPosLeftFieldBorder),Vector2<>(xPosOwnFieldBorder,yPosRightFieldBorder));
  linesTables[0]->push(Vector2<>(xPosOwnFieldBorder,yPosRightFieldBorder),Vector2<>(xPosOpponentFieldBorder,yPosRightFieldBorder));
  /**fieldBorder*/
  linesTables[1]->push(Vector2<>(xPosOpponentGroundline,yPosRightSideline),Vector2<>(xPosOpponentGroundline,yPosLeftSideline));
  linesTables[1]->push(Vector2<>(xPosOpponentGroundline,yPosLeftSideline),Vector2<>(xPosOpponentGroundline,yPosLeftSideline));
  linesTables[1]->push(Vector2<>(xPosOwnGroundline,yPosLeftSideline),Vector2<>(xPosOwnGroundline,yPosRightSideline));
  linesTables[1]->push(Vector2<>(xPosOwnGroundline,yPosRightSideline),Vector2<>(xPosOpponentGroundline,yPosRightSideline));
  /**FieldLines*/
  //field border lines
  linesTables[2]->push(Vector2<>(xPosOpponentGroundline,yPosRightSideline),Vector2<>(xPosOpponentGroundline,yPosLeftSideline));
  linesTables[2]->push(Vector2<>(xPosOpponentGroundline,yPosLeftSideline),Vector2<>(xPosOwnGroundline,yPosLeftSideline));
  linesTables[2]->push(Vector2<>(xPosOwnGroundline,yPosLeftSideline),Vector2<>(xPosOwnGroundline,yPosRightSideline));
  linesTables[2]->push(Vector2<>(xPosOwnGroundline,yPosRightSideline),Vector2<>(xPosOpponentGroundline,yPosRightSideline));
  //center line
  linesTables[2]->push(Vector2<>(xPosHalfWayLine,yPosLeftSideline),Vector2<>(xPosHalfWayLine,yPosRightSideline));
  //penalty areas
  linesTables[2]->push(Vector2<>(xPosOwnGroundline,yPosLeftPenaltyArea),Vector2<>(xPosOwnPenaltyArea,yPosLeftPenaltyArea));
  linesTables[2]->push(Vector2<>(xPosOwnPenaltyArea,yPosLeftPenaltyArea),Vector2<>(xPosOwnPenaltyArea,yPosRightPenaltyArea));
  linesTables[2]->push(Vector2<>(xPosOwnPenaltyArea,yPosRightPenaltyArea),Vector2<>(xPosOwnGroundline,yPosRightPenaltyArea));
  linesTables[2]->push(Vector2<>(xPosOpponentGroundline,yPosLeftPenaltyArea),Vector2<>(xPosOpponentPenaltyArea,yPosLeftPenaltyArea));
  linesTables[2]->push(Vector2<>(xPosOpponentPenaltyArea,yPosLeftPenaltyArea),Vector2<>(xPosOpponentPenaltyArea,yPosRightPenaltyArea));
  linesTables[2]->push(Vector2<>(xPosOpponentPenaltyArea,yPosRightPenaltyArea),Vector2<>(xPosOpponentGroundline,yPosRightPenaltyArea));
  //throw-in lines
  linesTables[2]->push(Vector2<>(1150,0),Vector2<>(1250,0));
  linesTables[2]->push(Vector2<>(xPosThrowInPointOpponentHalf,-50),Vector2<>(xPosThrowInPointOpponentHalf,50));
  linesTables[2]->push(Vector2<>(-1150,0),Vector2<>(-1250,0));
  linesTables[2]->push(Vector2<>(xPosThrowInPointOwnHalf,-50),Vector2<>(xPosThrowInPointOwnHalf,50));
  linesTables[2]->push(Vector2<>(-50,0),Vector2<>(50,0));
  //center circle
  linesTables[2]->pushCircle(Vector2<>(0.0f,0.0f), (float)centerCircleRadius, 16);
}

void FieldDimensions::readCorners()
{
  //xCorner
  corners[0].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[0].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  //tCorner0
  corners[1].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[1].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[1].push_back(Vector2<int>(xPosOwnGroundline,yPosLeftPenaltyArea));
  corners[1].push_back(Vector2<int>(xPosOwnGroundline,yPosRightPenaltyArea));
  //tCorner90
  corners[2].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[2].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[2].push_back(Vector2<int>(xPosHalfWayLine,yPosRightSideline));
  //tCorner180
  corners[3].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[3].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[3].push_back(Vector2<int>(xPosOpponentGroundline,yPosLeftPenaltyArea));
  corners[3].push_back(Vector2<int>(xPosOpponentGroundline,yPosRightPenaltyArea));
  //tCorner270
  corners[4].push_back(Vector2<int>(xPosHalfWayLine, centerCircleRadius));
  corners[4].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[4].push_back(Vector2<int>(xPosHalfWayLine,yPosLeftSideline));
  //lCorner0
  corners[5].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[5].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[5].push_back(Vector2<int>(xPosOwnGroundline,yPosLeftPenaltyArea));
  corners[5].push_back(Vector2<int>(xPosOwnGroundline,yPosRightPenaltyArea));
  corners[5].push_back(Vector2<int>(xPosHalfWayLine,yPosRightSideline));
  corners[5].push_back(Vector2<int>(xPosOwnGroundline,yPosRightSideline));
  corners[5].push_back(Vector2<int>(xPosOpponentPenaltyArea,yPosRightPenaltyArea ));
  //lCorner90
  corners[6].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[6].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[6].push_back(Vector2<int>(xPosOpponentGroundline,yPosLeftPenaltyArea));
  corners[6].push_back(Vector2<int>(xPosOpponentGroundline,yPosRightPenaltyArea));
  corners[6].push_back(Vector2<int>(xPosHalfWayLine,yPosRightSideline));
  corners[6].push_back(Vector2<int>(xPosOpponentGroundline,yPosRightSideline));
  corners[6].push_back(Vector2<int>(xPosOwnPenaltyArea,yPosRightPenaltyArea));
  //lCorner180
  corners[7].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[7].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[7].push_back(Vector2<int>(xPosOpponentGroundline,yPosLeftPenaltyArea));
  corners[7].push_back(Vector2<int>(xPosOpponentGroundline,yPosRightPenaltyArea));
  corners[7].push_back(Vector2<int>(xPosHalfWayLine,yPosLeftSideline));
  corners[7].push_back(Vector2<int>(xPosOpponentGroundline,yPosLeftSideline));
  corners[7].push_back(Vector2<int>(xPosOwnPenaltyArea,yPosLeftPenaltyArea));
  //lCorner270
  corners[8].push_back(Vector2<int>(xPosHalfWayLine,centerCircleRadius));
  corners[8].push_back(Vector2<int>(xPosHalfWayLine,-centerCircleRadius));
  corners[8].push_back(Vector2<int>(xPosOwnGroundline,yPosLeftPenaltyArea));
  corners[8].push_back(Vector2<int>(xPosOwnGroundline,yPosRightPenaltyArea));
  corners[8].push_back(Vector2<int>(xPosHalfWayLine,yPosLeftSideline));
  corners[8].push_back(Vector2<int>(xPosOwnGroundline,yPosLeftSideline));
  corners[8].push_back(Vector2<int>(xPosOpponentPenaltyArea,yPosLeftPenaltyArea));
}

Pose2D FieldDimensions::randomPoseOnField() const
{
  Pose2D pose;
  do
    pose = Pose2D::random(x,y,Range<>(-pi,pi));
  while(!isInsideField(pose.translation));
  return pose;
}

Pose2D FieldDimensions::randomPoseOnCarpet() const
{
  Pose2D pose;
  do
    pose = Pose2D::random(x,y,Range<>(-pi,pi));
  while(!isInsideCarpet(pose.translation));
  return pose;
}

void FieldDimensions::draw(Image* image)
{
  img = image;
  drawLines();
  //drawCorners();
}

void FieldDimensions::drawLines() const
{
    int lineWidth = fieldLinesWidth;
    ColorRGB lineColor(192,192,192);

    for(std::vector<LinesTable::Line>::const_iterator i = fieldLines.lines.begin(); i != fieldLines.lines.end(); ++i)
    {
      Vector2<> source = i->corner.translation;
      Pose2D target(i->corner);
      target.translate(i->length, 0);

      Draw::Line(img, Point2D(source.x/10, source.y/10 + 540), Point2D(target.translation.x/10, target.translation.y/10), lineWidth/10, lineColor);
    }
}

  /*
void FieldDimensions::drawPolygons(RoboCup::uint32 ownColor) const
{

  DECLARE_DEBUG_DRAWING("field polygons", "drawingOnField");
  COMPLEX_DRAWING("field polygons",
    Vector2<>* points = new Vector2<>[carpetBorder.lines.size()];
    for (unsigned i = 0; i < carpetBorder.lines.size(); ++i)
    {
      points[i] = carpetBorder.lines[i].corner.translation;
    }
    POLYGON("field polygons", (int) carpetBorder.lines.size(), points,
            0, Drawings::ps_solid, ColorRGBA(0, 180, 0), Drawings::bs_solid, ColorRGBA(0, 140, 0));
    delete [] points;

    ColorRGBA own = ownColor == TEAM_BLUE ? ColorRGBA(100,100,255) : ColorRGBA(ColorClasses::yellow);
    ColorRGBA opp = ownColor != TEAM_BLUE ? ColorRGBA(100,100,255) : ColorRGBA(ColorClasses::yellow);

    CIRCLE("field polygons", xPosOwnGoalpost, yPosLeftGoal, 50, 0, Drawings::ps_solid,
      own, Drawings::bs_solid, own);
    CIRCLE("field polygons", xPosOwnGoalpost, yPosRightGoal, 50, 0, Drawings::ps_solid,
      own, Drawings::bs_solid, own);
    own.a = 100;
    Vector2<> goal[4];
    goal[0] = Vector2<>((float) xPosOwnGoalpost, (float) yPosLeftGoal);
    goal[1] = Vector2<>((float) xPosOwnGoalpost, (float) yPosRightGoal);
    goal[2] = Vector2<>((float) xPosOwnGoal, (float) yPosRightGoal);
    goal[3] = Vector2<>((float) xPosOwnGoal, (float) yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, own, Drawings::bs_solid, own);

    CIRCLE("field polygons", xPosOpponentGoalpost, yPosLeftGoal, 50, 0, Drawings::ps_solid,
      opp, Drawings::bs_solid, opp);
    CIRCLE("field polygons", xPosOpponentGoalpost, yPosRightGoal, 50, 0, Drawings::ps_solid,
      opp, Drawings::bs_solid, opp);
    opp.a = 100;
    goal[0] = Vector2<>((float) xPosOpponentGoalpost, (float) yPosLeftGoal);
    goal[1] = Vector2<>((float) xPosOpponentGoalpost, (float) yPosRightGoal);
    goal[2] = Vector2<>((float) xPosOpponentGoal, (float) yPosRightGoal);
    goal[3] = Vector2<>((float) xPosOpponentGoal, (float) yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, opp, Drawings::bs_solid, opp);
  );
}
*/


/*void FieldDimensions::drawCorners() const
{

  DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");
#ifndef RELEASE
  CornerClass c = xCorner;
#endif
  MODIFY_ENUM("fieldDimensions:cornerClass", c, numOfCornerClasses, &getCornerClassName);
  COMPLEX_DRAWING("field corners",
    for(CornersTable::const_iterator i = corners[c].begin(); i != corners[c].end(); ++i)
      LARGE_DOT("field corners", i->x, i->y, ColorRGBA(255,255,255), ColorRGBA(255,255,255));
  );
}
*/
void FieldDimensions::LinesTable::push(const Pose2D& p, float l, bool isPartOfCircle)
{
  LinesTable::Line line;
  line.corner = p;
  line.length = l;
  line.isPartOfCircle = isPartOfCircle;
  lines.push_back(line);
}

void FieldDimensions::LinesTable::push(const Vector2<>& s, const Vector2<>& e, bool isPartOfCircle)
{
  Vector2<> d = e - s;
  push(Pose2D(d.angle(), s), d.abs(), isPartOfCircle);
}


void FieldDimensions::LinesTable::pushCircle(const Vector2<>& center, float radius, int numOfSegments)
{
  Vector2<> p1, p2;
  for (float a = 0; a <= pi_4; a += pi2/numOfSegments)
  {
    p1 = Vector2<>(sinf(a), cosf(a)) * radius;
    if (a > 0)
    {
      push(center + p1, center + p2, true);
      push(center + Vector2<>(p1.x,-p1.y), center + Vector2<>(p2.x,-p2.y), true);
      push(center + Vector2<>(-p1.x,p1.y), center + Vector2<>(-p2.x,p2.y), true);
      push(center - p1, center - p2, true);
      push(center + Vector2<>(p1.y,p1.x), center + Vector2<>(p2.y,p2.x), true);
      push(center + Vector2<>(p1.y,-p1.x), center + Vector2<>(p2.y,-p2.x), true);
      push(center + Vector2<>(-p1.y,p1.x), center + Vector2<>(-p2.y,p2.x), true);
      push(center + Vector2<>(-p1.y,-p1.x), center + Vector2<>(-p2.y,-p2.x), true);
    }
    p2 = p1;
  }
}

void FieldDimensions::LinesTable::doubleSided(float width, const FieldDimensions::LinesTable& single)
{
  for(std::vector<Line>::const_iterator i = single.lines.begin(); i != single.lines.end(); ++i)
  {
    push(i->corner +
          Pose2D(Vector2<>(-width/2,width/2)),
          i->length + width);
    push(i->corner +
          Pose2D(pi, Vector2<>(i->length,0)) +
          Pose2D(Vector2<>(-width/2,width/2)),
          i->length + width);
  }
}

bool FieldDimensions::LinesTable::isInside(const Vector2<>& v) const
{
  //note:
  //This function assumes that the point (0,0) is inside and
  //that for any point inside the area the line to (0,0) belongs to the area too.

  Geometry::Line testLine(v, -v);
  for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    float factor;
    Geometry::Line border(i->corner, i->length);
    if (Geometry::getIntersectionOfRaysFactor(border, testLine, factor))
      return false;
  }
  return true;
}

float FieldDimensions::LinesTable::clip(Vector2<>& v) const
{
  if(isInside(v))
    return 0;
  else
  {
    Vector2<> old = v,
                          v2;
    float minDist = 100000;
    for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    {
      Vector2<> diff = (Pose2D(old) - i->corner).translation;
      if(diff.x < 0)
        v2 = i->corner.translation;

      else if(diff.x > i->length)
        v2 = (i->corner + Pose2D(Vector2<>(i->length,0))).translation;
      else
        v2 = (i->corner + Pose2D(Vector2<>(diff.x,0))).translation;
      float dist = (old - v2).abs();
      if(minDist > dist)
      {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).abs();
  }
}

bool FieldDimensions::LinesTable::getClosestPoint(Vector2<>& vMin, const Pose2D& p, int numberOfRotations, float minLength) const
{
  int trueNumberOfRotations = numberOfRotations;
  if(numberOfRotations == 2)
    numberOfRotations = 4;

  // target angle -> target index
  float r = p.rotation / pi2 * numberOfRotations + 0.5f;
  if(r < 0)
    r += numberOfRotations;
  int targetRot = int(r);
  //ASSERT(targetRot >= 0 && targetRot < numberOfRotations);
  targetRot %= trueNumberOfRotations;
  Vector2<> v2;
  float minDist = 100000;
  for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if(i->length >= minLength)
    {
      // angle -> index
      float r = (i->corner.rotation + pi_2) / pi2 * numberOfRotations + 0.5f;
      if(r < 0)
        r += numberOfRotations;
      else if(r >= numberOfRotations)
        r -= numberOfRotations;
      int rot = int(r);
      //ASSERT(rot >= 0 && rot < numberOfRotations);
      rot %= trueNumberOfRotations;

      // index must be target index
      if(rot == targetRot)
      {
        Vector2<> diff = (p - i->corner).translation;
        if(diff.x < 0)
          v2 = i->corner.translation;
        else if(diff.x > i->length)
          v2 = (i->corner + Pose2D(Vector2<>(i->length,0))).translation;
        else
          v2 = (i->corner + Pose2D(Vector2<>(diff.x,0))).translation;
        Vector2<> vDiff = v2 - p.translation;
        float dist = vDiff.abs();
        if(minDist > dist)
        {
          minDist = dist;
          vMin = v2;
        }
      }
    }
  return (minDist<100000);
}

float FieldDimensions::LinesTable::getDistance(const Pose2D& pose) const
{
  float minDist = 100000;
  for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Vector2<> v1 = (i->corner - pose).translation,
                    v2 = (i->corner + Pose2D(Vector2<>(i->length,0))
                          - pose).translation;
    if(v1.y < 0 && v2.y > 0)
    {
      float dist = v1.x + (v2.x - v1.x) * -v1.y / (v2.y - v1.y);
      if(dist >= 0 && dist < minDist)
        minDist = dist;
    }
  }
  return minDist == 100000 ? -1 : minDist;
}

/*
void FieldDimensions::LinesTable::draw(const ColorRGBA& color, bool drawNormals) const
{
  for(std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Vector2<> s = i->corner.translation;
    Vector2<> p = (i->corner + Pose2D(Vector2<>(i->length,0))).translation;
    LINE("field lines",
       (int) s.x,
       (int) s.y,
       (int) p.x,
       (int) p.y,
       0, Drawings::ps_solid, color);
    if(drawNormals)
    {
      p = (i->corner + Pose2D(Vector2<>(i->length/2,0))).translation;
      Vector2<> p2 = (i->corner + Pose2D(Vector2<>(i->length/2,100))).translation;
      LINE("field lines",
         (int) p2.x,
         (int) p2.y,
         (int) p.x,
         (int) p.y,
         0, Drawings::ps_solid, color);
    }
  }
}
*/

const Vector2<int>& FieldDimensions::CornersTable::getClosest(const Vector2<int>& p) const
{
  //ASSERT(!empty());
  int maxDistance2 = std::numeric_limits<int>().max();
  const Vector2<int>* closest = 0;
  for(const_iterator i = begin(); i != end(); ++i)
  {
    Vector2<int> diff = p - *i;
    int distance2 = diff * diff;
    if(maxDistance2 > distance2)
    {
      maxDistance2 = distance2;
      closest = &*i;
    }
  }
  return *closest;
}

const Vector2<> FieldDimensions::CornersTable::getClosest(const Vector2<>& p) const
{
  Vector2<int> closest = getClosest(Vector2<int>((int) p.x, (int) p.y));
  return Vector2<>((float) closest.x, (float) closest.y);
}

const char* FieldDimensions::getCornerClassName(FieldDimensions::CornerClass cornerClass)
{
  switch(cornerClass)
  {
  case xCorner: return "xCorner";
  case tCorner0: return "tCorner0";
  case tCorner90: return "tCorner90";
  case tCorner180: return "tCorner180";
  case tCorner270: return "tCorner270";
  case lCorner0: return "lCorner0";
  case lCorner90: return "lCorner90";
  case lCorner180: return "lCorner180";
  case lCorner270: return "lCorner270";
  default: return "UNKNOWN";
  }
}
