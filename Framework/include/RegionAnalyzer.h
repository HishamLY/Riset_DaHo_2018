/**
* @file RegionAnalyzer.h
* @author jeff
*/

#ifndef _REGIONANALYZER_H_
#define _REGIONANALYZER_H_

//#include "Tools/Module/Module.h"
//#include "Representations/Perception/CameraMatrix.h"
#include "Image.h"
#include "RegionPercept.h"
#include "BallSpots.h"
#include "LineSpots.h"

namespace Robot
{
/**
 * @class RegionAnalyzer
 * This class analyzes the regions from the RegionPercept and creates
 * LinesSpots, NonLineSpots and BallSpots.
 */

class RegionAnalyzer
{
private:
  /**
   * @class Parameters
   * The parameters for the RegionAnalyzer
   */

  class Parameters
  {
  private:

  public:
    Parameters() :
      minLineSize(10),
      minLineSegmentCount(2),
      maxLineNghbNoneSize(550),
      minLineNghbGreenAboveSize(30),
      minLineNghbGreenBelowSize(30),
      minLineNghbGreenSideSize(20),
      minLineSingleSegmentSize(300),
      maxLineNghbNoneRatio(0.5f),
      maxLineNghbGreySkip(2),
      minEccentricityOfBallRegion(-100.0f),
      minRobotRegionSize(100),
      maxRobotRegionAlphaDiff(0.785398f),
      minRobotWidthRatio(2)
    {}

    int minLineSize; /**< The minimum size of a line region */
    int minLineSegmentCount; /**< The minimum number of segments for a line regions */
    int maxLineNghbNoneSize; /**< The maximum size of neighboring none colored regions for a line region */
    int minLineNghbGreenAboveSize; /**< The minimum summed size of green segments above a line region */
    int minLineNghbGreenBelowSize; /**< The minimum summed size of green segments below a line region */
    int minLineNghbGreenSideSize; /**< The minimum summed size of green segments on each side of a line region */
    int minLineSingleSegmentSize; /**< The minimum size a region needs to be a line if the minLineSegmentCount criterion is not met */
    float maxLineNghbNoneRatio; /**< The maximum ratio of none neighbor regions for a line region */
    int maxLineNghbGreySkip; /**< The maximal amount of pixels skip to find some green above/below/aside a white region */
    float minEccentricityOfBallRegion; /**< Minimum eccentricity of a orange region to be a ballspot */
    int minRobotRegionSize; /**< The minimum region size for a white region to become nonLineSpot */
    float maxRobotRegionAlphaDiff; /**< The maximum angle offset a white region can have from upright to become a nonLineSpot */
    float minRobotWidthRatio; /**< The minimum height/width ratio a white regions needs to become nonLineSpot */
  };

  Parameters parameters; /**< The parameters of this module. */
  LineSpots lineSpots; /**< The linespots found. */

  /** analyzes all the regions and
  create the ballSpots, lineSpots and nonLineSpots */
  void analyzeRegions(BallSpots& ballSpots);

  /** debug drawing or not */
  bool debug_draw;


  /** Checks whether a region is a line region
   * @param region the region to be checked
   * @param direction the direction of the spot is returned
   * @param spot a lineSpots for the region
   * @return is it a line?
   */
  inline bool isLine(const RegionPercept::Region* region, float& direction, LineSpots::LineSpot& spot);
  /** Returns how much green is below the region */
  int getGreenBelow(const RegionPercept::Region* region);
  /** Returns how much green is above the region */
  int getGreenAbove(const RegionPercept::Region* region);
  /** Returns how much green is left to the region */
  int getGreenLeft(const RegionPercept::Region *region);
  /** Returns how much green is right to the region */
  int getGreenRight(const RegionPercept::Region *region);

public:
  /**
  * Default constructor.
  */
  RegionPercept* regionPercept; /**< internal pointer to the RegionPercept */

  /** update the BallSpots Representation */
  void update(BallSpots& ballSpots, RegionPercept& rPercept, Image* image);

  /** update the LineSpots Representation */
  void update(LineSpots& lineSpots)
  {
    lineSpots = this->lineSpots;

  }

  Image* img;

  RegionAnalyzer();
  ~RegionAnalyzer();
};
}



#endif// __RegionAnalyzer_h_
