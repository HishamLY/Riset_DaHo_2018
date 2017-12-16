#include "PointExplorer.h"
//#include "RegionPercept.h"
//#include "Tools/Debugging/Modify.h"

using namespace Robot;

ColorClasses::Color PointExplorer::getColor(int x, int y)
{
  if(theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 0] == 255 &&
      theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 1] == 255 &&
      theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 2] == 0)
    return ColorClasses::yellow;
  else if(theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 0] == 0 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 1] == 0 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 2] == 255)
    return ColorClasses::blue;
  else if(theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 0] == 0 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 1] == 255 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 2] == 0)
    return ColorClasses::green;
  else if(theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 0] == 255 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 1] == 0 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 2] == 0)
    return ColorClasses::white;
  else if(theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 0] == 255 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 1] == 128 &&
          theImage->m_ImageData[(y * Camera::WIDTH + x) * theImage->m_PixelSize + 2] == 0)
    return ColorClasses::orange;
  else
    return ColorClasses::none;
}

int PointExplorer::explorePoint(int x, int y, ColorClasses::Color col, int xMin, int yEnd, int yMin, int& run_end, int& explored_min_y, int& explored_max_y, bool force_detailed)
{
  int size = 0;
  if(col != ColorClasses::green || force_detailed)
  {
    run_end = runDown(x, y, col, yEnd);//, Drawings::ps_solid);
    explored_min_y = y;
    explored_max_y = run_end-1;

    size = (run_end - y) * parameters.gridStepSize;

    if(!(run_end - y >= parameters.minSegSize[col]))
      return -1;

    for(x -= parameters.exploreStepSize; x > xMin; x -= parameters.exploreStepSize)
    {
      if(getColor(x, explored_min_y) == col)
      {
        const int expl_run_end = runUp(x,explored_min_y, col, yMin);//, Drawings::ps_dot);
        if(expl_run_end < explored_min_y)
          explored_min_y = expl_run_end+1;
      }
      if(getColor(x, explored_max_y) == col)
      {
        const int expl_run_end = runDown(x, explored_max_y, col, yEnd);//, Drawings::ps_dot);
        if(expl_run_end > explored_max_y)
          explored_max_y = expl_run_end-1;
      }
    }
  }
  else
  {
    run_end = runDown(x, y, col, yEnd);//, Drawings::ps_solid);
    size = (run_end - y) * parameters.gridStepSize;
    explored_min_y = y;
    explored_max_y = run_end;
  }
  return size;
}


void PointExplorer::initFrame(Image *image, int exploreStepSize, int gridStepSize, int skipOffset, int* minSegLength)
{
  //printf("img di pointexp = %i,%i\n",image, &image);
  //Draw::Line(image, Point2D(100,100),Point2D(200,200), ColorRGB(ColorClasses::red));
  theImage = image;
  //printf("\n exp = %i", theImage->m_ImageData[1*theImage->m_PixelSize + 1]);
  //theColorTable64 = colorTable;
  parameters.exploreStepSize = exploreStepSize;
  parameters.gridStepSize = gridStepSize;
  parameters.skipOffset = skipOffset;
  parameters.minSegSize = minSegLength;
}

int PointExplorer::runDown(int x, int yStart, ColorClasses::Color col, int yEnd)
{
  int y = yStart;
  int tmp;

  for(y+=parameters.skipOffset; y < yEnd; y+=parameters.skipOffset)
    {
      if(getColor(x,y) != col)
        {
          tmp = y - parameters.skipOffset;
          for(--y; y > tmp; y--)
            if(getColor(x,y) == col)
              break;

          if(y == tmp)
            {
              y++;
              break;
            }
        }
    }
  if(y > yEnd)
    y = yEnd;

  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //      {LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));}
//);
  return y;
}


int PointExplorer::findDown(int x, int yStart, ColorClasses::Color col, int yEnd)
{
  int y = yStart;
  int tmp;

  for(y+=parameters.skipOffset; y < yEnd; y+=parameters.skipOffset)
  {
    if(getColor(x,y) == col)
    {
      tmp = y - parameters.skipOffset;
      for(--y; y > tmp; y--)
        if(getColor(x,y) != col)
          break;

      if(y != tmp)
        ++y;
      break;
    }
  }
  if(y > yEnd)
    y = yEnd;

  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //      {LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));}
  //  );
  return y;
}

int PointExplorer::findDown2(int x, int yStart, ColorClasses::Color col1, ColorClasses::Color col2, int yEnd, ColorClasses::Color& foundCol)
{
  int y = yStart;
  int tmp;
  ColorClasses::Color col = ColorClasses::none;

  for(y+=parameters.skipOffset; y < yEnd; y+=parameters.skipOffset)
  {
    col = getColor(x,y);
    if(col == col1 || col == col2)
    {
      tmp = y - parameters.skipOffset;
      for(--y; y > tmp; y--)
        if(getColor(x,y) != col)
          break;

      if(y != tmp)
        ++y;
      break;
    }
  }
  if(y > yEnd)
    y = yEnd;

  foundCol = col;
  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //      {LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));}
  //  );
  return y;
}

int PointExplorer::findUp2(int x, int yStart, ColorClasses::Color col1, ColorClasses::Color col2, int yEnd, ColorClasses::Color& foundCol)
{
  int y = yStart;
  int tmp;
  ColorClasses::Color col = ColorClasses::none;

  for(y-=parameters.skipOffset; y > yEnd; y-=parameters.skipOffset)
  {
    col = getColor(x,y);

    if(col == col1 || col == col2)
    {
      tmp = y + parameters.skipOffset;
      for(++y; y < tmp; y++)
        if(getColor(x,y) != col)
          break;

      if(y != tmp)
        y--;
      break;
    }

  }
  if(y < yEnd)
    y = yEnd;


  if(col == col1 || col == col2)
    foundCol = col;
  else foundCol = ColorClasses::none;

  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //      {LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));}
  //  );
  return y;
}

int PointExplorer::runUp(int x, int yStart, ColorClasses::Color col, int yEnd)
{
  int y = yStart;
  int tmp;

  for(y-=parameters.skipOffset; y > yEnd; y-=parameters.skipOffset)
  {
    if(getColor(x,y) != col)
    {
      tmp = y + parameters.skipOffset;
      for(++y; y < tmp; y++)
        if(getColor(x,y) == col)
          break;

      if(y == tmp)
      {
        y--;
        break;
      }
    }
  }
  if(y < yEnd)
    y = yEnd;

  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //    {LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));}
  //  );
  return y;
}

int PointExplorer::runRight(int xStart, int y, ColorClasses::Color col, int xMax)
{
  int x = xStart;
  int tmp;

  for(x += parameters.skipOffset; x < xMax; x += parameters.skipOffset)
  {
    if(getColor(x,y) != col)
    {
      tmp = x - parameters.skipOffset;
      for(--x; x > tmp; x--)
        if(getColor(x,y) == col)
          break;

      if(x == tmp)
      {
        x++;
        break;
      }
    }
  }
  if(x > xMax)
    x = xMax;

  //COMPLEX_DRAWING("module:PointExplorer:runs",
  //    {LINE("module:PointExplorer:runs", xStart, y, x, y, 0, draw, getOnFieldDrawColor(col));}
  //  );
  return x;
}

float PointExplorer::colorRatio(const ColorClasses::Color& col, const Boundary<int>& area, int stepSize)
{
  const float numberOfPixels(float((area.x.max + 1 - area.x.min) * (area.y.max + 1 - area.y.min)));
  if (numberOfPixels == 0.0)
  {
    return 0.0;
  }
  const int stepSize2 = stepSize*stepSize;
  int sameColor(0);
  const int xStart(std::max(area.x.min, 0)),
        xEnd(std::min(area.x.max, theImage->m_Width-1));
  const int yStart(std::max(area.y.min, 0)),
        yEnd(std::min(area.y.max, theImage->m_Height-1));
  const int lastColStepSize = (xEnd - xStart) % stepSize + 1;
  const int lastColStepSize2 = lastColStepSize * stepSize;
  const int lastRowStepSize = (yEnd - yStart) % stepSize + 1;
  const int lastRowStepSize2 = lastRowStepSize * stepSize;
  for(int x(xStart); x <= xEnd; x+=stepSize)
  {
    bool lastCol = x+stepSize > xEnd;
    for(int y(yStart); y <= yEnd; y+=stepSize)
    {
      bool lastRow = y+stepSize > yEnd;
      if(getColor(x, y) == col)
      {
        sameColor += lastCol ? (lastRow ? lastColStepSize * lastRowStepSize : lastColStepSize2) : (lastRow ? lastRowStepSize2 : stepSize2);
        //DOT("module:PointExplorer:blob", x, y, col, col);
      }
    }
  }
  return static_cast<float>(sameColor) / numberOfPixels;
}
