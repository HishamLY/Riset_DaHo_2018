/*
 * ColorFinder.h
 *
 *  Created on: 2018. 2. 18.
 *      Author: vaguz
 */

#ifndef COLORCLASSES_H_
#define COLORCLASSES_H_

namespace Robot
{
    class ColorClasses
    {
    public:
      enum Color
      {
        none,                   /*<! all other objects */
        orange,                 /*<! ball */
        yellow,                 /*<! yellow goal */
        blue,                   /*<! blue goal */
        white,                  /*<! lines */
        green,                  /*<! field */
        black,                  /*<! most probably: nothing */
        red,                    /*<! color of red robots> */
        robotBlue,              /*<! color of blue robots> */
        rgb100,                  /*<!red, green, blue 100 */
        rg100,
        rb100,
        gb100,
        numOfColors             /*<! number of colors */
      };

      /**
      * Returns the name of a color class
      * @param color The color class
      * @return The name
      */
      static const char* getColorName(Color color)
      {
        switch(color)
        {
        case none: return "none";
        case orange: return "orange";
        case yellow: return "yellow";
        case blue: return "blue";
        case green: return "green";
        case white: return "white";
        case black: return "black";
        case red: return "red";
        case robotBlue: return "robotBlue";
        case rgb100: return "rgb100";
        case rg100: return "rg100";
        case rb100: return "rb100";
        case gb100: return "gb100";
        default: return "unknown color!";
        };
      }
    };

    class ColorRGB
    {
    public:
      ColorRGB()
      {
        r = g = b = 0;
      }

      ColorRGB(unsigned char r,
                unsigned char g,
                unsigned char b):
        r(r), g(g), b(b)
      {
      }

      ColorRGB(const ColorRGB& other)
      {
        *this = other;
      }

      ColorRGB(ColorClasses::Color colorClass)
      {
        switch(colorClass)
        {
        case ColorClasses::orange:      r = 255; g = 128; b = 0;  break;
        case ColorClasses::yellow:      r = 255; g = 255; b = 0;   break;
        case ColorClasses::blue:        r = 0;   g = 0;   b = 128; break;
        case ColorClasses::green:       r = 0;   g = 255; b = 0;   break;
        case ColorClasses::white:       r = 255; g = 255; b = 255; break;
        case ColorClasses::black:       r = 0; g = 0; b = 0;       break;
        case ColorClasses::robotBlue:   r = 0; g = 255; b = 255;   break;
        case ColorClasses::red:         r = 255; g = 0; b = 0;     break;
        case ColorClasses::rgb100:      r = 100; g =100; b = 100;  break;
        case ColorClasses::rg100:       r = 100; g =100; b=0;       break;
        case ColorClasses::rb100:       r = 100; g=0; b=100;        break;
        case ColorClasses::gb100:       r = 0; g=100; b=100;        break;
        default: r = 0; g = 96; b = 128;                           break;
        };
      }

      unsigned char r;
      unsigned char g;
      unsigned char b;
    };
}

#endif /* COLORCLASSES_H_ */
