/**
* @file Regionizer.h
* @author Imre
*/


#ifndef _REGIONIZER_H_
#define _REGIONIZER_H_

#include "MathFunction.h"
#include "minIni.h"
#include <vector>

#include "Image.h"
#include "ColorClasses.h"
#include "PointExplorer.h"
#include "Vector2.h"
#include "DebugDrawings.h"
#include "RegionPercept.h"
//#include "Point.h"

#define SECTION   "Regionizer"
#define INVALID_VALUE   -1024.0


namespace Robot
{
/**
 * @class Regionizer
 * This class segments the image and create regions from the segments.
 */
class Regionizer
{
public:
  //forward declaration
  /*class RegionPercept
  {
      public:
       class Segment;
  };*/

  /**
  * Default constructor.
  */
  Regionizer();
  ~Regionizer();

  /** Updates the RegionPercept */
  void update(RegionPercept& rPercept, Image* img);

  void tes(RegionPercept* regionpercept, Image* img);
  void LoadINISettings(minIni* ini);
  void LoadINISettings(minIni* ini, const std::string &section);


private:
  typedef std::vector<Vector2<int> >::const_iterator CI;

  /**
     * @class Parameters
     * The parameters for the Regionizer
     */
  class Parameters
  {
  private:

  public:
    int gridStepSize, /**< The distance in pixels between neighboring scan lines. */
    skipOffset; /**< The maximum number of pixels to skip when grouping pixels to segments. */
    int minSegSize[ColorClasses::numOfColors]; /**< The minimal size/length for a segment in pixels for each color. numOfColors = 9*/
    float regionLengthFactor[ColorClasses::numOfColors]; /**< The maximal allowed factor one segment is to be longer than another when grouping to a region */
    int regionMaxSize; /**< The maximal number of segments per region */
    float maxAngleDiff; /**< The maximal angle difference from one segment to another.
                              The angle is the vector from the middle of the last segment to the next one. */
    int exploreStepSize; /**< The distance in pixels between exploring scanlines */
    int borderMinGreen; /**< The minimum number of green pixels for border detection */
    long searchBallStartTime; /**< The time when additional scanlines were used to locate balls near the horizon in ms. */
    int searchBallGridStepSize; /**< The number grid size for extra ball scanlines when ball wasn't seen. Must be a divisor of gridStepSize, otherwise there won't be any extra scanlines used. */
  };

  RegionPercept* regionPercept; /**< internal pointer to the RegionPercept */
  Parameters parameters; /**< The parameters of this module. */
  PointExplorer pointExplorer; /**< PointerExplorer instace for running in the image */


  /**
  * The method scans vertically for segments in the image.
  * It includes the scanning for goal / post segments
  */
  void scanVertically();

  /**
   * Unites the regions of two segments if certain criterions are met. If seg2
   * does not have a region yet, it will be added to seg1's region.
   * @param seg1 segment which already has region
   * @param seg2 segment(including it's region if it has one) which will be merge to seg1's region
   * @return whether the regions where united
   */
  bool uniteRegions(RegionPercept::Segment* seg1, RegionPercept::Segment* seg2);

  /**
   * Returns a pointer to a new segment (from the segments array within the RegionPercept).
   * @param x the x coordinate of the new segment
   * @param y the y coordinate of the new segment
   * @param length the length of the new segment
   * @param color the color of the new segment
   * @param yEnd the end of the scanline the segment was found on
   * @return pointer to the new segment or NULL if segments array is full
   */

  inline RegionPercept::Segment* addSegment(int x, int y, int length, ColorClasses::Color color, int yEnd);
  /**
   * Creates a new Region in the RegionPercept for the segment passed.
   * @param seg the segment to be added to the region
   * @return whether a region was created (the percept was not full)
   */
  inline bool createNewRegionForSegment(RegionPercept::Segment* seg); //Segment belum dideclare
  /**
   * Connects a segment to the already created regions. If the segment
   * does not touch a segment of the same color, a new region will be formed.
   * @param newSegment the segment to be connected
   * @param lastColumPointer a pointer to a segment in the last colum which is above newSegment
   * @param xDiff the difference in x coordinates from the last colum to the actual (=gridStepSize)
   * @return the lastColumpointer for the next segment
   */
  inline RegionPercept::Segment* connectToRegions(RegionPercept::Segment* newSegment, RegionPercept::Segment* lastColumPointer, int xDiff);
  /**
   * Builds the regions from the segments.
   */
  void buildRegions();

  /**
   * Detects the fieldborder in the image and stores it in the RegionPercept.
   * @param xStart the x coordinate to start scanning
   * @param xEnd the x coordinate to stop scanning
   */
  void calcBorders(int xStart, int xEnd);
  /**
   * Forms the upper part of the convex hull of the fieldborder points.
   * @return The points of the upper part of the convex hull
   */
  std::vector<Vector2<int> > getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders);


};
}

#endif