#include "DebugDrawings.h"
#include <algorithm>
#include <cstdlib>
#include <stdio.h>

using namespace Robot;
using namespace std;



void Draw::gambarSampel(Image* img, vector<Sample> Samples)
{
  ColorRGB clr;
  Point2D hline;

  for (int i=0; i<Samples.size(); i++)
    {
      switch (Samples[i].cluster_i)
        {
        case 0 :
          clr = ColorClasses::blue;
          break;
        case 1 :
          clr = ColorClasses::orange;
          break;
        case 2 :
          clr = ColorClasses::red;
          break;
        case 3 :
          clr = ColorClasses::white;
          break;
        case 4 :
          clr = ColorClasses::black;
          break;
        case 5 :
          clr = ColorClasses::robotBlue;
          break;
        case 6 :
          clr = ColorClasses::rgb100;
          break;
        case 7 :
          clr = ColorClasses::rb100;
          break;
        case 8 :
          clr = ColorClasses::rg100;
          break;
        case 9 :
          clr = ColorClasses::gb100;
          break;
        default :
          clr = ColorClasses::red;
          break;
        }
      //Circle(img, Point2D(Samples[i].PositionAndOrientation.x, Samples[i].PositionAndOrientation.y), 5, clr);
      rasterCircle(img, Point2D(Samples[i].Pose.translation.x, Samples[i].Pose.translation.y), 5, clr);

      hline.X = Samples[i].Pose.translation.x+7*cos(MATH::DegreesToRadians(Samples[i].Pose.rotation));
      hline.Y = Samples[i].Pose.translation.y-7*sin(MATH::DegreesToRadians(Samples[i].Pose.rotation));
      Line(img, Point2D(Samples[i].Pose.translation.x, Samples[i].Pose.translation.y), hline, clr);
    }
}

void Draw::gambarCluster(Image *img, vector<Cluster> Clusters)
{
  Point2D hline;
  for (int i=0; i< Clusters.size(); i++)
  {
    rasterCircle(img, Point2D(Clusters[i].mean_centroid.translation.x,Clusters[i].mean_centroid.translation.y), 15, ColorRGB(ColorClasses::robotBlue));
    hline.X = Clusters[i].mean_centroid.translation.x+7.5*cos(MATH::DegreesToRadians(Clusters[i].mean_centroid.rotation));
    hline.Y = Clusters[i].mean_centroid.translation.y-7.5*sin(MATH::DegreesToRadians(Clusters[i].mean_centroid.rotation));
    Line(img, Point2D(Clusters[i].mean_centroid.translation.x, Clusters[i].mean_centroid.translation.y), hline, ColorRGB(ColorClasses::robotBlue));  
  }
}

void Draw::gambarLandmark(Image* img, vector<Landmark> Landmark)
{
  for (int i=0; i< Landmark.size(); i++)
    Circle(img, Point2D(Landmark[i].x, Landmark[i].y), 10, ColorRGB(ColorClasses::white));
}

void Draw::gambarRobot(Image* img, Pose2D robot)
{

  rasterCircle(img, Point2D(robot.translation.x, robot.translation.y), 10,2, ColorRGB(ColorClasses::red));
  // HEADING LINE
  Point2D hline;
  hline.X = robot.translation.x+15*cos(MATH::DegreesToRadians(robot.rotation));
  hline.Y = robot.translation.y-15*sin(MATH::DegreesToRadians(robot.rotation));
  Arrow(img, Point2D(robot.translation.x, robot.translation.y), hline, ColorRGB(ColorClasses::red)); //orientasi robot

}

void Draw::gambarPosPredicted(Image* img, Pose2D posPredicted)
{
  Point2D hline;
  //Circle(img, Point2D(pos.X, pos.Y), 10, ColorRGB(ColorClasses::yellow));
  rasterCircle(img, Point2D(posPredicted.translation.x, posPredicted.translation.y), 20, 3, ColorRGB(ColorClasses::yellow));

  // HEADING LINE
  hline.X = posPredicted.translation.x+25*cos(MATH::DegreesToRadians(posPredicted.rotation));
  hline.Y = posPredicted.translation.y-25*sin(MATH::DegreesToRadians(posPredicted.rotation));
  Arrow(img, Point2D(posPredicted.translation.x, posPredicted.translation.y), hline, ColorRGB(ColorClasses::yellow)); //orientasi robot

}

void Draw::LineField(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb)
{
  Point2D pt_a((pt0.X+img->m_Width/2), (pt0.Y+img->m_Height/2));
  Point2D pt_b((pt1.X+img->m_Width/2), (pt1.Y+img->m_Height/2));
  printf("%f,%f %f, %f\n", pt0.X, pt0.Y, pt1.X, pt1.Y);
  Line(img, pt_a, pt_b, rgb);
}

void Draw::LineField(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb)
{
  Point2D pt_a((pt0.X+img->m_Width/2), (pt0.Y+img->m_Height/2));
  Point2D pt_b((pt1.X+img->m_Width/2), (pt1.Y+img->m_Height/2));
  Line(img, pt_a, pt_b, thickness, rgb);
}

void Draw::SetPixel(Image* img, int PixelNumber, ColorRGB rgb)
{
  img->m_ImageData[PixelNumber*img->m_PixelSize + 0] = rgb.r;
  img->m_ImageData[PixelNumber*img->m_PixelSize + 1] = rgb.g;
  img->m_ImageData[PixelNumber*img->m_PixelSize + 2] = rgb.b;
}

void Draw::SetPixel(Image* img, Point2D pt, ColorRGB rgb)
{
  img->m_ImageData[((int)pt.X*img->m_Width+(int)pt.Y)*img->m_PixelSize + 0] = rgb.r;
  img->m_ImageData[((int)pt.X*img->m_Width+(int)pt.Y)*img->m_PixelSize + 1] = rgb.g;
  img->m_ImageData[((int)pt.X*img->m_Width+(int)pt.Y)*img->m_PixelSize + 2] = rgb.b;
}

void Draw::Rectangle(Image* img, Point2D begin, Point2D end, ColorRGB rgb)
{
  Point2D pt0, pt1, pt2, pt3;
  pt0 = begin;
  pt1.X = end.X;
  pt1.Y = begin.Y;
  pt2.X = begin.X;
  pt2.Y = end.Y;
  pt3 = end;
  Line(img, pt0, pt1, rgb);
  Line(img, pt1, pt3, rgb);
  Line(img, pt3, pt2, rgb);
  Line(img, pt2, pt0, rgb);
}

void Draw::FullRect(Image* img, Point2D center, int width, ColorRGB rgb)
{
}

void Draw::Line(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb)
{
  bool steep = abs(int(pt1.Y - pt0.Y)) > abs(int(pt1.X - pt0.X));
  if(steep)
    {
      swap(pt0.X, pt0.Y);
      swap(pt1.X, pt1.Y);
    }
  if(pt0.X > pt1.X)
    {
      swap(pt0.X, pt1.X);
      swap(pt0.Y, pt1.Y);
    }
  int deltax = pt1.X - pt0.X;
  int deltay = abs(int(pt1.Y - pt0.Y));
  int error = deltax / 2;
  int ystep;
  int y = pt0.Y;
  if(pt0.Y < pt1.Y)
    ystep = 1;
  else
    ystep = -1;
  for(int x = pt0.X; x < pt1.X; x++)
    {
      if(steep)
        {
          int PixelNumber = x*img->m_Width+y;
          SetPixel(img, PixelNumber, rgb);
        }
      else
        {
          int PixelNumber = y*img->m_Width+x;
          SetPixel(img, PixelNumber, rgb);
        }
      error = error - deltay;
      if(error < 0)
        {
          y = y + ystep;
          error = error + deltax;
        }
    }
}

void Draw::Line(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb)
{
  bool steep = abs(int(pt1.Y - pt0.Y)) > abs(int(pt1.X - pt0.X));
  if(steep)
    {
      swap(pt0.X, pt0.Y);
      swap(pt1.X, pt1.Y);
    }
  if(pt0.X > pt1.X)
    {
      swap(pt0.X, pt1.X);
      swap(pt0.Y, pt1.Y);
    }
  int deltax = pt1.X - pt0.X;
  int deltay = abs(int(pt1.Y - pt0.Y));
  int error = deltax / 2;
  int ystep;
  int y = pt0.Y;
  if(pt0.Y < pt1.Y)
    ystep = 1;
  else
    ystep = -1;
  for(int x = pt0.X; x < pt1.X; x++)
    {
      if(steep)
        {
          Circle(img, Point2D(x,y), thickness/2, rgb);
        }
      else
        {
          Circle(img, Point2D(x,y), thickness/2, rgb);
        }
      error = error - deltay;
      if(error < 0)
        {
          y = y + ystep;
          error = error + deltax;
        }
    }
}

void Draw::Arrow(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb)
{
  double dx = (pt0.X - pt1.X)/3;
  double dy = (pt0.Y - pt1.Y)/3;
  const double cos = 0.866;
  const double sin = 0.500;
  Point2D end1(
    (float)(pt1.X + (dx * cos + dy * -sin)),
    (float)(pt1.Y + (dx * sin + dy * cos)));
  Point2D end2(
    (float)(pt1.X + (dx * cos + dy * sin)),
    (float)(pt1.Y + (dx * -sin + dy * cos)));
  Line(img, pt0, pt1, rgb);
  Line(img, pt1, end1, rgb);
  Line(img, pt1, end2, rgb);
}

void Draw::Arrow(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb)
{
  double dx = (pt0.X - pt1.X)/3;
  double dy = (pt0.Y - pt1.Y)/3;
  const double cos = 0.866;
  const double sin = 0.500;
  Point2D end1(
    (float)(pt1.X + (dx * cos + dy * -sin)),
    (float)(pt1.Y + (dx * sin + dy * cos)));
  Point2D end2(
    (float)(pt1.X + (dx * cos + dy * sin)),
    (float)(pt1.Y + (dx * -sin + dy * cos)));
  Line(img, pt0, pt1,  thickness, rgb);
  Line(img, pt1, end1, thickness, rgb);
  Line(img, pt1, end2, thickness, rgb);
}

void Draw::Ellipse(Image* img, Point2D center, int width, int height, ColorRGB rgb)
{
  int hh = height * height;
  int ww = width * width;
  int hhww = hh * ww;
  int x0 = width;
  int dx = 0;

  // do the horizontal diameter
  for (int x = -width; x <= width; x++)
    {
      int PixelNumber = center.Y*img->m_Width+center.X+x;
      SetPixel(img, PixelNumber, rgb);
    }

  // now do both halves at the same time, away from the diameter
  for (int y = 1; y <= height; y++)
    {
      int x1 = x0 - (dx - 1);  // try slopes of dx - 1 or more
      for ( ; x1 > 0; x1--)
        if (x1*x1*hh + y*y*ww <= hhww)
          break;
      dx = x0 - x1;  // current approximation of the slope
      x0 = x1;

      for (int x = -x0; x <= x0; x++)
        {
          int PixelNumber = (center.Y-y)*img->m_Width+(center.X+x);
          SetPixel(img, PixelNumber, rgb);

          PixelNumber = (center.Y+y)*img->m_Width+(center.X+x);
          SetPixel(img, PixelNumber, rgb);
        }
    }
}

void Draw::Circle(Image* img, Point2D Center, int Radius, ColorRGB rgb)
{
  Ellipse(img, Center, Radius, Radius, rgb);
}

void Draw::Cross(Image* img, Point2D pt, int size, ColorRGB rgb)
{
  Line(img, Point2D(pt.X+size, pt.Y+size), Point2D(pt.X-size, pt.Y-size), rgb);
  Line(img, Point2D(pt.X+size, pt.Y-size), Point2D(pt.X-size, pt.Y+size), rgb);
}

void Draw::rasterCircle(Image* img, Point2D Center, int radius, ColorRGB rgb)
{
  double temp;
  temp = Center.X;
  Center.X = Center.Y;
  Center.Y = temp;

  int error = 1 - radius;
  int errorY = 1;
  int errorX = -2 * radius;
  int x = radius, y = 0;

  SetPixel(img, Point2D(Center.X, Center.Y + radius), rgb);
  SetPixel(img, Point2D(Center.X, Center.Y- radius), rgb);
  SetPixel(img, Point2D(Center.X + radius, Center.Y), rgb);
  SetPixel(img, Point2D(Center.X - radius, Center.Y), rgb);

  while(y < x)
    {
      if(error > 0) // >= 0 produces a slimmer circle. =0 produces the circle picture at radius 11 above
        {
          x--;
          errorX += 2;
          error += errorX;
        }
      y++;
      errorY += 2;
      error += errorY;
      SetPixel(img, Point2D(Center.X + x, Center.Y + y), rgb);
      SetPixel(img, Point2D(Center.X - x, Center.Y + y), rgb);
      SetPixel(img, Point2D(Center.X + x, Center.Y - y), rgb);
      SetPixel(img, Point2D(Center.X - x, Center.Y - y), rgb);
      SetPixel(img, Point2D(Center.X + y, Center.Y + x), rgb);
      SetPixel(img, Point2D(Center.X - y, Center.Y + x), rgb);
      SetPixel(img, Point2D(Center.X + y, Center.Y - x), rgb);
      SetPixel(img, Point2D(Center.X - y, Center.Y - x), rgb);
    }
}

void  Draw::rasterCircle(Image* img, Point2D Center, int radius, int thick, ColorRGB rgb)
{

  if (thick==1)
  {
    rasterCircle(img, Center, radius, rgb);
  }
  else if (thick < radius)
  {
    for (int i=0; i<thick; i++)
      rasterCircle(img, Center, radius-i, rgb);
  }
}