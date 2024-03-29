#include "Regionizer.h"
#include <stdio.h>

using namespace Robot;

Regionizer::Regionizer()
{
}

Regionizer::~Regionizer()
{
}

void Regionizer::update(RegionPercept& rPercept, Image* img)
{
  regionPercept = &rPercept;

  pointExplorer.initFrame(img, parameters.exploreStepSize, parameters.gridStepSize, parameters.skipOffset, parameters.minSegSize);

  //Draw::Line(pointExplorer.theImage, Point2D(0,0),Point2D(100,100), ColorRGB(ColorClasses::red));
  if(parameters.searchBallGridStepSize > parameters.gridStepSize || parameters.searchBallGridStepSize < 1 || parameters.gridStepSize % parameters.searchBallGridStepSize != 0)
    parameters.searchBallGridStepSize = parameters.gridStepSize;
  regionPercept->fieldBorders.clear();
  regionPercept->segmentsCounter = 0;
  regionPercept->regionsCounter = 0;
  regionPercept->gridStepSize = parameters.gridStepSize;
  scanVertically();   //STOP_TIME_ON_REQUEST("scanVert", scanVertically(););
  buildRegions();   //STOP_TIME_ON_REQUEST("buildRegions", buildRegions(););
  //regionPercept->draw(pointExplorer.theImage);
}

void Regionizer::buildRegions()
{
  int lastx = -1;

  RegionPercept::Segment* firstInColum=NULL, *lastColumPointer = NULL, *newSegment, *lastSegment = NULL;

  for(int i = 0; i < regionPercept->segmentsCounter; i++)
    {
      newSegment = regionPercept->segments + i;

      if(newSegment->color == ColorClasses::green)
        continue;
      //a new colum started
      if(lastx == -1 || lastx != newSegment->x)
        {
          if(lastx == newSegment->x - parameters.gridStepSize)
            lastColumPointer = firstInColum;
          else
            lastColumPointer = NULL;
          firstInColum = newSegment;
          lastSegment = NULL;
        }

      lastColumPointer = connectToRegions(newSegment, lastColumPointer, parameters.gridStepSize);
      //ASSERT(newSegment->region != NULL || regionPercept->regionsCounter >= MAX_REGIONS_COUNT);
      if(newSegment->region == NULL) //MAX_REGIONS_COUNT
        break;

      if(lastSegment != NULL)
        {
          if(newSegment-> y - (lastSegment->y + lastSegment->length) < parameters.skipOffset)
            {
              lastSegment->region->neighborRegions.push_back(newSegment->region);
              newSegment->region->neighborRegions.push_back(lastSegment->region);
            }
          else
            lastSegment = NULL;
        }

      lastSegment = newSegment;
      lastx = newSegment->x;
    }
}

RegionPercept::Segment* Regionizer::connectToRegions(RegionPercept::Segment* newSegment, RegionPercept::Segment* lastColumPointer, int xDiff)
{
  //ASSERT(!newSegment->region);
  if(!lastColumPointer)
    {
      createNewRegionForSegment(newSegment);
      return NULL;
    }

  //ASSERT(lastColumPointer->x == newSegment->x-xDiff);//parameters.gridStepSize);
  //if lastColumPointer needs to move on
  // _
  //|_|
  //   _    this case
  //  |_|
  //
  while(lastColumPointer->y + lastColumPointer->length < newSegment->explored_min_y && lastColumPointer->x == newSegment->x-xDiff)
    lastColumPointer++;
  //lastColumPointer is now either the first segment in the last line
  //which ends after the start of newSegment or is already in the next line -> return NULL

  if(lastColumPointer->x != newSegment->x-xDiff)
    {
      createNewRegionForSegment(newSegment);
      return NULL;
    }

  std::vector<RegionPercept::Region*> neighborRegions;

  if(lastColumPointer->y + lastColumPointer->length >= newSegment->explored_min_y)
    if(lastColumPointer->y <= newSegment->explored_max_y)
      {
        //TOUCHING
        if(lastColumPointer->color == newSegment->color)
          {
            if(!uniteRegions(lastColumPointer, newSegment))
              neighborRegions.push_back(lastColumPointer->region);
          }
        else
          {
            if(lastColumPointer->region)
              neighborRegions.push_back(lastColumPointer->region);
          }
      }

  //  _
  // | |_
  // |_| |
  //   |_| this case
  RegionPercept::Segment *tmpLastColumPointer;
  tmpLastColumPointer = lastColumPointer;
  while(tmpLastColumPointer->y + tmpLastColumPointer->length <= newSegment->explored_max_y)
    {
      tmpLastColumPointer++;
      //if the lastColumPointer moved to the next colum, move one back and break
      if(tmpLastColumPointer->x != newSegment->x - xDiff)
        {
          if(!newSegment->region)
            if(!createNewRegionForSegment(newSegment))
              return NULL;
          break;
        }

      //ASSERT(tmpLastColumPointer->y + tmpLastColumPointer->length >= newSegment->explored_min_y);
      if(tmpLastColumPointer->y <= newSegment->explored_max_y)
        {
          //TOUCHING
          if(tmpLastColumPointer->color == newSegment->color)
            {
              if(!uniteRegions(tmpLastColumPointer, newSegment))
                neighborRegions.push_back(tmpLastColumPointer->region);
            }
          else
            {
              if(tmpLastColumPointer->region)
                neighborRegions.push_back(tmpLastColumPointer->region);
            }
        }
      else
        break;
    }

  if(!newSegment->region)
    if(!createNewRegionForSegment(newSegment))
      return NULL;

  for(std::vector<RegionPercept::Region*>::iterator nb_reg = neighborRegions.begin(); nb_reg != neighborRegions.end(); nb_reg++)
    {
      //ASSERT(newSegment->region);
      (*nb_reg)->neighborRegions.push_back(newSegment->region);
      newSegment->region->neighborRegions.push_back(*nb_reg);
    }
  return lastColumPointer;
}

bool Regionizer::createNewRegionForSegment(RegionPercept::Segment* seg)
{
  if(regionPercept->regionsCounter < MAX_REGIONS_COUNT)
    {
      seg->region = regionPercept->regions + regionPercept->regionsCounter++;
      seg->region->color = seg->color;
      seg->region->childs.clear();
      seg->region->neighborRegions.clear();
      seg->region->min_y = seg->y;
      seg->region->max_y = seg->y + seg->length;
      seg->region->root = NULL;
      seg->region->size = seg->explored_size;
      seg->region->childs.push_back(seg);
      return true;
    }
  return false;
}

bool Regionizer::uniteRegions(RegionPercept::Segment* seg1, RegionPercept::Segment* seg2)
{
  //ASSERT(seg1->region);
  //we want straight white regions (lines) so don't unite white regions which would not be straight
  if(seg1->color == ColorClasses::white)
    {
      //ASSERT(seg1->region->childs.size() >= 1);
      if(seg1->region->childs.at(seg1->region->childs.size()-1)->x == seg2->x)
        return false;

      if(seg1->link)
        {
          Vector2<int> linkDiff = Vector2<int>(seg1->x - seg1->link->x, (seg1->y + seg1->length/2) - (seg1->link->y + seg1->link->length/2)),
                                  thisDiff = Vector2<int>(seg2->x - seg1->x, (seg2->y + seg2->length/2) - (seg1->y + seg1->length/2));
          if(fabs(linkDiff.angle() - thisDiff.angle()) > parameters.maxAngleDiff)
            return false;

        }
    }

  if(seg1->length * parameters.regionLengthFactor[seg1->color] < seg2->length ||
      seg2->length * parameters.regionLengthFactor[seg1->color] < seg1->length)
    return false;

  //seg1 always has a region
  if(!seg2->region)
    {
      RegionPercept::Segment *sWithRegion = seg1,
                                            *sWithoutRegion = seg2;

      if((int)sWithRegion->region->childs.size() < parameters.regionMaxSize ||
          seg1->color == ColorClasses::yellow || //yellow, blue and orange regions can grow unlimited
          seg1->color == ColorClasses::blue ||
          seg1->color == ColorClasses::orange)
        {
          sWithRegion->region->childs.push_back(sWithoutRegion);
          sWithoutRegion->region = sWithRegion->region;
          sWithRegion->region->size += sWithoutRegion->explored_size;

          if(sWithoutRegion->y < sWithRegion->region->min_y)
            sWithRegion->region->min_y = sWithoutRegion->y;
          if(sWithoutRegion->y + sWithoutRegion->length > sWithRegion->region->max_y)
            sWithRegion->region->max_y = sWithoutRegion->y + sWithoutRegion->length;
          seg2->link = seg1;
          return true;
        }
      //ASSERT(seg1->region == seg2->region || !seg2->region);
    }
  //both segments already have a region
  else
    {
      //don't unite two white regions (since we want straight white regions -> lines)
      if(seg1->color != ColorClasses::white)
        {
          if(seg1->region != seg2->region && ((int)(seg1->region->childs.size() + seg2->region->childs.size()) < parameters.regionMaxSize || seg1->color == ColorClasses::yellow || seg1->color == ColorClasses::blue || seg1->color == ColorClasses::orange))
            {
              RegionPercept::Region* oldRegion = seg2->region;
              seg1->region->mergeWithRegion(seg2->region);
              oldRegion->childs.clear();
              oldRegion->root = seg1->region;
              seg2->link = seg1;
              return true;
            }
        }
    }
  return false;
}

void Regionizer::scanVertically()
{
  //printf("width = %i\n",pointExplorer.theImage->m_ImageData[1*pointExplorer.theImage->m_PixelSize + 1]);
  int xStart = parameters.gridStepSize-1 + ((pointExplorer.theImage->m_Width) %  parameters.gridStepSize) / 2,
               xEnd = pointExplorer.theImage->m_Width,
                      yEnd = pointExplorer.theImage->m_Height,
                             yStart = 0, yFullStart = 0;
    /**bagian ini nanti ganti sama yHorizon robot*/
  //int yHorizon = pointExplorer.theImage->m_Height / 2; //int yHorizon = (int) Head::GetInstance()->GetHorizon();
  int yHorizon = 0;

  RegionPercept::Segment *newSegment;
  ColorClasses::Color curColor;
  int run_end_y, explored_min_y, explored_max_y, explored_size;

 //regionPercept->fieldborder diisi dengan titik2 yang menyusun field border
  calcBorders(xStart, xEnd);

  CI border = regionPercept->fieldBorders.begin();

  bool perceptFull = false;
  //bool useExtraScanlinesForBall = parameters.searchBallGridStepSize < parameters.gridStepSize && theFrameInfo.getTimeSince(theBallHypotheses.timeWhenDisappeared) > parameters.searchBallStartTime;
  bool useExtraScanlinesForBall = true; //true bola udah lama gak kelihatan, false gak ketemu, tapi masih dalam waktu yg ditetapkan
  bool ballScanline = false;
  int x = xStart,
          y = 0;

  for(; x < xEnd && !perceptFull ; x += parameters.gridStepSize)
    {
      // calculate yEnd based on stopPolygon
      yEnd = pointExplorer.theImage->m_Height;

      //theBodyContour.clipBottom(x, yEnd);

        //kalau border belum yg terakhir
      if(border != regionPercept->fieldBorders.end())
        {
          yStart = std::max(yHorizon, 0); //mulainya dari horizon
          yFullStart = border->y; //mulainya dari border
          border++;
        }
      else //kalau udah terakhir, mulai dari 0
        {
          yStart = 0;
          yFullStart = 0;
        }

      //true -> for(int xb = x; xb < x + parameters.gridStepSize && xb < xEnd && !perceptFull; xb += parameters.searchBallGridStepSize)
      //false -> for(int xb = x; xb == x; xb += parameters.searchBallGridStepSize)
      //x diincreament di for diatas.
      for(int xb = x; useExtraScanlinesForBall ? xb < x + parameters.gridStepSize && xb < xEnd && !perceptFull : xb == x; xb += parameters.searchBallGridStepSize)
        {
          if(useExtraScanlinesForBall)
            ballScanline = xb != x;

          y = yStart;

          while(y < yEnd)
            {
              if(ballScanline)
                {
                  curColor = ColorClasses::orange;
                  const int byEnd = ((yEnd-yStart) >> 1) + yStart;
                  y = pointExplorer.findDown(xb, y, curColor, byEnd);
                  if(y == byEnd)
                    break;
                }
              else
                curColor = pointExplorer.getColor(xb,y);

              explored_size = pointExplorer.explorePoint(xb, y, curColor, std::max(0, xb - parameters.gridStepSize), yEnd, yStart, run_end_y, explored_min_y, explored_max_y);

              if(run_end_y - y >= parameters.minSegSize[curColor])
                {
                  if(run_end_y > yFullStart || curColor == ColorClasses::blue || curColor == ColorClasses::red) //kenapa ada red ?
                    {
                      if(y < yFullStart && curColor != ColorClasses::blue && curColor != ColorClasses::red)
                        {
                          y = yFullStart;
                          explored_min_y = y;
                          explored_size = explored_max_y - explored_min_y;
                        }
                      newSegment = addSegment(xb, y, run_end_y - y, curColor, yEnd);

                      if(!newSegment) //MAX_SEGMENTS_COUNT
                        {
                          //ASSERT(regionPercept->segmentsCounter == MAX_SEGMENTS_COUNT - 1);
                          perceptFull = true;
                          break;
                        }

                      newSegment->explored_min_y = explored_min_y;
                      newSegment->explored_max_y = explored_max_y;
                      newSegment->explored_size = explored_size;
                    }
                }
              y = run_end_y;
            }
        }
    }
}

inline RegionPercept::Segment* Regionizer::addSegment(int x, int y, int length, ColorClasses::Color color, int yEnd)
{
  if(!(regionPercept->segmentsCounter < MAX_SEGMENTS_COUNT - 1))
    return NULL;

  RegionPercept::Segment* seg = regionPercept->segments + regionPercept->segmentsCounter++;
  seg->color = color;
  seg->x = x;
  seg->y = y;
  seg->explored_min_y = y;
  seg->explored_max_y = y+length;
  seg->explored_size = 0;
  seg->length = length;
  seg->region = NULL;
  seg->link = NULL;

  return seg;
}


void Regionizer::calcBorders(int xStart, int xEnd)
{
  regionPercept->fieldBorders.clear();

  //Vector2<int> p1 = theImageCoordinateSystem.fromHorizonBased(Vector2<>());
  //int yHorizon = pointExplorer.theImage->m_Height / 2; //int yHorizon = (int) Head::GetInstance()->GetHorizon();
  //Vector2<int> p1(0, pointExplorer.theImage->m_Height / 2);
  /**bagian ini nanti ganti sama horizon robot*/
  Vector2<int> p1(0,0);
  Vector2<int> b;
  int min_y_border=0;

  //if horizon is below cameraimage
  if(p1.y >= pointExplorer.theImage->m_Height)
    {
      regionPercept->fieldBorders.push_back(Vector2<int>(0, pointExplorer.theImage->m_Height-1));
      regionPercept->fieldBorders.push_back(Vector2<int>(pointExplorer.theImage->m_Width-1, pointExplorer.theImage->m_Height-1));
    }
  else
    {
      if(p1.y <= 0)
        min_y_border = 0;
      else
        min_y_border = p1.y;

      int x = xStart;
      int run_end;
      for(; x < xEnd; x += parameters.gridStepSize)
        {
          b.x = x;
          b.y = min_y_border;

          run_end = b.y;
          while(b.y < pointExplorer.theImage->m_Height)
            {
              b.y = pointExplorer.findDown(b.x, b.y, ColorClasses::green, pointExplorer.theImage->m_Height);
              if(b.y >= pointExplorer.theImage->m_Height)
                break;
              const int tyEnd = pointExplorer.theImage->m_Height < b.y + parameters.borderMinGreen+1? pointExplorer.theImage->m_Height : b.y + parameters.borderMinGreen+1;
              run_end = pointExplorer.runDown(b.x, b.y, ColorClasses::green, tyEnd);

              if(run_end - b.y > parameters.borderMinGreen)
                break;

              b.y = run_end;
            }
          if(b.y >= pointExplorer.theImage->m_Height)
            b.y =  pointExplorer.theImage->m_Height-1;
          regionPercept->fieldBorders.push_back(b);
        }
    }
  //sampai bagian ini, fieldborders udah ada di variabel. Titik masih banyak

  regionPercept->fieldBorders = getConvexFieldBorders(regionPercept->fieldBorders);
  //fieldborder udah dikurangin jadi dikit

  std::vector<Vector2<int> > newBorders;
  int x = xStart;
  CI p = regionPercept->fieldBorders.begin();
  CI last_p = p++;
  for(; p != regionPercept->fieldBorders.end(); p++)
    {
       while(x <= p->x)
        {
          const Vector2<int> v_m = (*p) - (*last_p);
          b.x = x;
          b.y = (x - last_p->x) * v_m.y / v_m.x + last_p->y;
          if(b.y < 0)
            b.y = 0;
          if(b.y >= pointExplorer.theImage->m_Height)
            b.y = pointExplorer.theImage->m_Height-1;
          //ASSERT(b.x >= 0);
          //ASSERT(b.x < theImage.cameraInfo.resolutionWidth);
          newBorders.push_back(b);
          x+=parameters.gridStepSize;
        }
      last_p = p;
    }

  regionPercept->fieldBorders = newBorders;

        for(CI p = regionPercept->fieldBorders.begin(); p != regionPercept->fieldBorders.end(); p++)
        {
          if(p != regionPercept->fieldBorders.begin())
            {
                //Draw::Line((pointExplorer.theImage), Point2D(p->x,p->y),Point2D(last_p->x,last_p->y), ColorRGB(ColorClasses::blue));
            }
          last_p = p;
        }
        {
            //Draw::Line((pointExplorer.theImage), Point2D(0,min_y_border),Point2D(pointExplorer.theImage->m_Width, min_y_border), ColorRGB(ColorClasses::blue));
        }

}

#define LEFT_OF(x0, x1, x2) ((x1.x-x0.x)*(-x2.y+x0.y)-(x2.x-x0.x)*(-x1.y+x0.y) > 0)

std::vector<Vector2<int> > Regionizer::getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders)
{
  //Andrew's Monotone Chain Algorithm to compute the upper hull
  std::vector<Vector2<int> > hull;
  const CI pmin = fieldBorders.begin(), pmax = fieldBorders.end()-1;
  hull.push_back(*pmin);
  for(CI pi = pmin + 1; pi != pmax+1; pi++)
    {
      if(!LEFT_OF((*pmin), (*pmax), (*pi)) && pi != pmax)
        continue;

      while((int)hull.size() > 1)
        {
          const CI p1 = hull.end() - 1, p2 = hull.end() - 2;
          if(LEFT_OF((*p1), (*p2), (*pi)))
            break;
          hull.pop_back();
        }
      hull.push_back(*pi);
    }
  return hull;
}

void Regionizer::LoadINISettings(minIni* ini)
{
  LoadINISettings(ini, SECTION);
}

void Regionizer::LoadINISettings(minIni* ini, const std::string &section)
{
  int value = -2;
  if((value = ini->geti(section, "gridStepSize", INVALID_VALUE)) != INVALID_VALUE)             parameters.gridStepSize = value;
  if((value = ini->geti(section, "Offsets", INVALID_VALUE)) != INVALID_VALUE)   parameters.skipOffset = value;
  if((value = ini->geti(section, "minSegSize_none", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[0]= value;
  if((value = ini->geti(section, "minSegSize_orange", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[1] = value;
  if((value = ini->geti(section, "minSegSize_yellow", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[2] = value;
  if((value = ini->geti(section, "minSegSize_blue", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[3] = value;
  if((value = ini->geti(section, "minSegSize_white", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[4] = value;
  if((value = ini->geti(section, "minSegSize_green", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[5] = value;
  if((value = ini->geti(section, "minSegSize_black", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[6] = value;
  if((value = ini->geti(section, "minSegSize_red", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[7] = value;
  if((value = ini->geti(section, "minSegSize_robotBlue", INVALID_VALUE)) != INVALID_VALUE)   parameters.minSegSize[8] = value;
  if((value = ini->geti(section, "regionLengthFactor_none", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[0]= value;
  if((value = ini->geti(section, "regionLengthFactor_orange", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[1] = value;
  if((value = ini->geti(section, "regionLengthFactor_yellow", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[2] = value;
  if((value = ini->geti(section, "regionLengthFactor_blue", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[3] = value;
  if((value = ini->geti(section, "regionLengthFactor_white", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[4] = value;
  if((value = ini->geti(section, "regionLengthFactor_green", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[5] = value;
  if((value = ini->geti(section, "regionLengthFactor_black", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[6] = value;
  if((value = ini->geti(section, "regionLengthFactor_magenta", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[7] = value;
  if((value = ini->geti(section, "regionLengthFactor_cyan", INVALID_VALUE)) != INVALID_VALUE)   parameters.regionLengthFactor[8]= value;
  if((value = ini->geti(section, "regionMaxSize", INVALID_VALUE)) != INVALID_VALUE)  parameters.regionMaxSize = value;
  if((value = ini->geti(section, "maxAlphaDiff", INVALID_VALUE)) != INVALID_VALUE)       parameters.maxAngleDiff = (float)value/3; //yang ini belum kebaca
  if((value = ini->geti(section, "exploreStepSize", INVALID_VALUE)) != INVALID_VALUE)       parameters.exploreStepSize = value;
  if((value = ini->geti(section, "borderMinGreen", INVALID_VALUE)) != INVALID_VALUE)       parameters.borderMinGreen = value;
  if((value = ini->geti(section, "searchBallStartTime", INVALID_VALUE)) != INVALID_VALUE)       parameters.searchBallStartTime = value;
  if((value = ini->geti(section, "searchBallGridStepSize", INVALID_VALUE)) != INVALID_VALUE)       parameters.searchBallGridStepSize = value;

}

void Regionizer::tes(RegionPercept* regionpercept, Image* img)
{
  //update(*regionpercept, *img);
}
