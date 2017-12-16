#ifndef _MATHFUNCTION_H_
#define _MATHFUNCTION_H_

#include <math.h>
#include <vector>
#include <algorithm>
#include <time.h>

using namespace std;

class MATH
{
    public :
        /**
        * return angle in degree
        */
        static double DegreesToRadians( double degrees );

        /**
        * return angle in radian
        */
        static double RadiansToDegrees( double radian );

         /**
        * find x and y using line equation
        */
        static int find_x(int x1, int y1, int x2, int y2, int y);
        static int find_y(int x1, int y1, int x2, int y2, int x);

        /**
        * fungsi untuk menghitung nilai probabilatas berdasarkan distribusi normal(gaussian)
        * @param x itu yang mau dihitung nilai probabilitasnya
        * @param mean
        * @param variance
        */
        static double normal_distribution_function(double x, double mean, double variance);
        
	/**
        * @return nilai numToRound yang udah dibulatin (multiplie == 1)
        */
        static int roundNumber(double numToRound, int multiplie);
        
	/**
        * @return distance between two points
        */
        static double Jarak(double x1,double y1,double x2, double y2);

        /**
        * @return distance between two pose (x,y,teta)
        */
        static double EuclideanDistance(double p1, double p2, double p3, double q1, double q2,double q3);

        /**
        * @return sudut yang dibentuk oleh dua titik. Sudut berdasarkan vector (1,0)
        */
        static double Sudut(double x1, double y1, double x2, double y2);

        /**
        * @return angle yang dibentuk oleh vector heading robot dan vector posisi dari titik x,y
        */
        static double getAngleToPoint(double x, double y);

	/**
	* generating random value from normal distribution
	* @param mean of the normal distribution
	* @param stdev standar deviation (square root of variance)
	* return random value from normal distribution
	*/
	static double randomGaussian(double mean, double stdev);

	/**
	* @param mean
	* @return random value from exponential distribution
	*/
	static double randomExponential(double mean);

    
};

#endif
