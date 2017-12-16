/*
 * ColorFinder.h
 *
 *  Created on: 2018. 2. 18.
 *      Author: vaguz
 */

#ifndef DEBUGDRAWINGS_H_
#define DEBUGDRAWINGS_H_

#include "ColorClasses.h"
#include "Point.h"
#include "Image.h"
#include "Localization.h"

namespace Robot
{
class Drawings
{
public:
  /** IDs for shape types
  * shapes are the basic drawings that can be sent.
  */
  enum ShapeType
  {
    circle, polygon, ellipse, line, dot, midDot, largeDot,
    arrow, text, tip, origin, gridRGBA, gridMono
  };

  /** The pen style that is used for basic shapes*/
  enum PenStyle
  {
    ps_solid, ps_dash, ps_dot, ps_null
  };

  /** The fill style that is used for basic shapes*/
  enum FillStyle
  {
    bs_solid, bs_null
  };
};

class Draw : public Drawings
{
public:
 /**draw for lokalisasi*/
  static void gambarSampel(Image* img, vector<Sample> Samples);
  static void gambarCluster(Image* img, vector<Cluster> Clusters);
  static void gambarLandmark(Image* img,vector<Landmark> Landmark);
  //static void gambarRobot(Image* img, Point2D pos, double direction);
  //static void gambarPosPredicted(Image* img, Point2D pos, double direction);
  static void gambarRobot(Image* img, Pose2D robot);
  static void gambarPosPredicted(Image* img, Pose2D posPredicted);

 /**draw on Image */
  static void FullRect(Image* img, Point2D center, int width, ColorRGB rgb);
  static void Cross(Image* img, Point2D pt, int size, ColorRGB rgb);
  static void Cross(Image* img, Point2D pt, int size, int thickness, ColorRGB rgb);
  static void Arrow(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb);
  static void Arrow(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb);
  static void Line(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb);
  static void Line(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb);
  static void Rectangle(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb);
  static void Circle(Image* img, Point2D Center, int Radius, ColorRGB rgb); //full circle
  static void rasterCircle(Image* img, Point2D Center, int radius, ColorRGB rgb); //pinggiran lingkaran
  static void rasterCircle(Image* img, Point2D Center, int radius, int thick, ColorRGB rgb); //pinggiran lingkaran
  static void Ellipse(Image* img, Point2D center, int width, int height, ColorRGB rgb);
  static void SetPixel(Image* img, int PixelNumber, ColorRGB rgb);
  static void SetPixel(Image* img, Point2D pt, ColorRGB rgb);

/** draw on field */
  static void LineField(Image* img, Point2D pt0, Point2D pt1, ColorRGB rgb);
  static void LineField(Image* img, Point2D pt0, Point2D pt1, int thickness, ColorRGB rgb);

private:
  float transformwidth(float pt, float width);
  float transformheight(float pt, float height);
};
}

#endif /* DEBUGDRAWINGS_H_ */
