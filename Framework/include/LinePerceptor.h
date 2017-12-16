/**
* @file LinePerceptor.h
* @author jeff
*/

#ifndef __LINEPERCEPTOR_H_
#define __LINEPERCEPTOR_H_

//#include "Tools/Module/Module.h"
//#include "Representations/Infrastructure/FrameInfo.h"
#include "LineSpots.h"
#include "LinePercept.h"
//#include "Camera.h"
#include "ImageCoordinateSystem.h"
#include "CameraMatrix.h"
#include "FieldDimensions.h"

namespace Robot
{
/**
 * @class LinePerceptor
 * This class creates lines and intersection from the lineSpots. It takes
 * the nonLineSpots to form "BanSectors" to filter out lines seen in robots.
 */
class LinePerceptor
{
  /**
   * @class Parameters
   * The parameters for the LinePerceptor.
   */
  class Parameters
  {
  private:

  public:
    Parameters() :

      minWidthRatio(1.25f), /**< The minimum height/width ratio a spot needs to be taken into account */
      maxAlphaDiff(0.15f), /**< The maximum difference in direction (Hess norm form) for spots belonging to the same line */
      maxDDiff(150), /**< The maximum difference in distance (Hess norm form) for spots belonging to the same line */
      minSupporters(1), /**< The minimum of supporters a segments needs to become a line */
      minLineSingleRegionLength(400), /**< The minimum size of a region to form a line wihtout other the support of other spots */
      minLineStartLength(50), /**< The minimum size of a region to be able to look for supporters to forma line */
      maxLineUniteDist(175), /**< The maximum distance (Hess norm form) for two lines to be merge together */
      maxLineUniteAlphaDiff(0.785f), /**< The maximum difference in direction (Hess norm form) for two lines to be merged together */
      maxLineSingleSegDist(150), /**< The maximum distance of the start and end point of a single segment to be added to a line */
      maxLineSingleSegDist2(250), /**< The maximum disatnce of the start / end point to the start / end point of a line to be added to it */
      maxLineDistance(6000), /**< The maximum distance a line segment can be away from the camera to be taken into account */
      minHardcover(0.6f), /**< The minimum hardcover ratio (length/covered by segments) of a line */
      minTToEnd(200),//(250), /**< The minimum distance a intersection needs to the start / end of a line to be a T or X intersection */
      maxTFromEnd(250), /**< The maximum distance a intersection can have to the start / end of a line to be a T and not a X */
      minIntersectionAlphaDiff(1.0f),//(1.178f), /**< The minimum difference in direction two lines need to intersect */
      minIntersectionLength(100), // ini diatur buat ngebaca intersection /**< The minimum length of a line to be able to intersect another one */
      circleBiggerThanSpecified(0),  /**< hrhrhr, this is a parameter to (fine)tune the center circle detection. This can improve the center circle detection if the cameraMatrix is not B-Human like aaeeh perfectly calibrated :-p */
      maxMidLineToCircleDist(50),//100 /**< The maximum distance a line may have to the center of the center circle to be detected as the middle line */
      minMidLineLength(500),//700 /**< The minimum length of a line to be accepted a middle line */
      maxLineCircleDist(100) /**< If a line is closer to the center circle than this, it will be deleted */
    {}

    float minWidthRatio;
    float maxAlphaDiff,
    maxDDiff;
    int minLineSingleRegionLength;
    int minLineStartLength;
    int maxLineUniteDist;
    float maxLineUniteAlphaDiff;
    int maxLineSingleSegDist;
    int maxLineSingleSegDist2;
    int maxLineDistance;
    float minHardcover;
    int minSupporters;
    int minTToEnd;
    int maxTFromEnd;
    float minIntersectionAlphaDiff;
    int minIntersectionLength;
    int circleBiggerThanSpecified;
    int maxMidLineToCircleDist;
    int minMidLineLength;
    int maxLineCircleDist;
  };

  /**
   * @class NonLineParameters
   * Parameters to filter out lineSpots which are in BanSectors.
   */
  class NonLineParameters
  {
  private:

  public:
    NonLineParameters() :
      minLineLength(700),
      maxAlphaDiff(0.78539816339744828f),
      minWidthRatio(2)
    {}

    int minLineLength; /**< The minimum length of a spot to be filtered out */
    float maxAlphaDiff; /**< The maximum direction offset of a spot from upright to be filtered out */
    float minWidthRatio; /**< The minimum height/width ratio of a spot to be filtered out */
  };

  /**
   * @class CircleParameters
   * Parameters for the center circle detection.
   */
  class CircleParameters
  {
  private:

  public:
    CircleParameters() :
      maxNgbhDist(200),
      maxRadiusError(250),
      minSegmentLength(0),
      minSegmentImgLength(32),
      minSupporters(4),
      maxSupporterDist(200),
      maxSupporterDist2(300)
    {}

    int maxNgbhDist; /**< The maximum distance of two linesegments to create a circleSpot */
    int maxRadiusError; /**< The maximum error in radius a intersection from two linesegments may have to create a circleSpot */
    int minSegmentLength; /**< The minimum length of a lineSegment to be taken into account for the center circle */
    int minSegmentImgLength; /**< The minimum length in image coordinates of a linesegment to be taken into acount for the center circle */
    int minSupporters; /**< The minimum number of supporters for the center circle */
    int maxSupporterDist; /**< The maximum distance of two circleSpots to support each other */
    int maxSupporterDist2; /**< The maximum distance of two circleSpots to support each other in the second round */
  };

  /**
   * @class BanSectorParameters
   * Parameters for the creation of the BanSectors.
   */
  class BanSectorParameters
  {
  private:

  public:
    BanSectorParameters() :
      angleStepSize(0.05f),
      minSectorCounter(3),
      maxLineAngleDiff(0.05f)
    {}

    float angleStepSize; /**< A angle added to the left and right of a sector (so a sector is bigger than the detected nonLineSpots) */
    int minSectorCounter; /**< The minimum number of spots in a sector to be a BanSector */
    float maxLineAngleDiff; /**< The maximum distance (angle) a line may have from the sector to be filtered out */
  };

  /**
   * @class BanSector
   * A class to hold a BanSector.
   */
  class BanSector
  {
  public:
    int start; /**< Distance the Sector starts */
    int end; /**<Distance the sector ends */
    float alphaLeft; /**< left angle of the sector */
    float alphaRight; /**< right angle of the sector */
    int counter; /**< Number of nonLineSpots in this sector */
  };

  Parameters parameters; /**< Parameters for this module */
  CircleParameters circleParams; /**< Parameters for center circle detection */
  NonLineParameters nonLineParams; /**< Parameters for filtering out lines near robots */
  BanSectorParameters banSectorParams; /**< Parameters for the creation of ban sectors */

  std::list<LinePercept::LineSegment> lineSegs; /**< All the lineSegments */
  std::list<BanSector> banSectors; /**< The ban sectors, where no vertical, long spots are accepted */



  /** create BanSectors */
  void createBanSectors();

  /**
   * creates the linesegments from the lineSpots, it creates the BanSector and filters out lines which meet
   * the banSector filter criterions
   * @param singleSegs a reference to the singleSegs list in the LinePercept
   * */
  void createLineSegments(std::list<LinePercept::LineSegment>& singleSegs);

  /**
   * creates the lines from the singleSegments
   * @param lines a reference to the lines list in the LinePercept
   * @param singleSegs a reference to the singleSegs list in the LinePercept
   * */
  void createLines(std::list<LinePercept::Line>& lines, std::list<LinePercept::LineSegment>& singleSegs);

  /**
   * analyzes the lines, merges and deletes some if neccessary and creates intersections
   * @param lines a reference to the lines list in the LinePercept
   * @param intersections a reference to the intersections vector in the LinePercept
   * @param circle a reference to the circle in the LinePercept
   * @param singleSegs a reference to the singleSegs list in the LinePercept
   * */
  void analyzeLines(std::list<LinePercept::Line>& lines, std::vector<LinePercept::Intersection>& intersections, LinePercept::CircleSpot& circle, std::list<LinePercept::LineSegment>& singleSegs);

  /**
   * analyze the singleSegments and try to find the center circle
   * @param singleSegs a reference to the singleSegs list in the LinePercept
   * @param circle a reference to the circle in the LinePercept
   * @param lines a reference to the lines list in the LinePercept
   * */
  void analyzeSingleSegments(std::list<LinePercept::LineSegment>& singleSegs, LinePercept::CircleSpot& circle, std::list<LinePercept::Line>& lines);

  /**
   * Determines the start and end point of a line. If updateLine == true the d and alpha values of the line
   * (Hess norm form) are recalculated.
   * @param line the line to determine the start and end point of
   * @param first returns the start/end point of the line
   * @param last returns the start/end point of the line
   * @param updateLine recalculate d and alpha of line (Hess norm form)?
   * */
  void getFirstAndLastOfLine(LinePercept::Line& line, Vector2<int>& first, Vector2<int>& last, bool updateLine = true);

public:
  /*
   * Default constructor
   */
  LinePerceptor();

  ImageCoordinateSystem theImageCoordinateSystem;
  FieldDimensions theFieldDimensions;

  LineSpots* theLineSpots; /**< internal pointer to the LineSpots */
  CameraMatrix* theCameraMatrix; /** <internal pointer to the CameraMatrix */
  Image* img;

  /** update the LinePercept */
  void update(LinePercept& linePercept , LineSpots& linespots , Image* image, CameraMatrix& thecameramatrix); //bener
};
}
#endif// __LinePerceptor_h_
