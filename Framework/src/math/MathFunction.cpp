#include "MathFunction.h"
#include <stdio.h>
#include <stdlib.h>


double MATH::DegreesToRadians( double degrees )
{
    return degrees * ( M_PI/180 );
}

double MATH::RadiansToDegrees( double radian )
{
    return ( radian/M_PI ) * 180 ;
}

int MATH::find_y(int x1, int y1, int x2, int y2, int x)
{
    int temp = (x2 - x1) == 0? 1 : (x2 - x1);
    return((y2-y1)*(x-x1)/temp+y1);
}

int MATH::find_x(int x1, int y1, int x2, int y2, int y)
{
    int temp = (y2 - y1) == 0? 1 : (y2 - y1);
    return((x2-x1)*(y-y1)/temp+x1);
}

/**
* @param x
* @param mean, titik tengah kurva ketika kurva distribusi max
* @param variance, variansi dari distribusi
* digunakan untuk menghitung peluang dari P(x)
*/
double MATH::normal_distribution_function(double x, double mean, double variance)
{
    double MeanKuadrat = (x - mean)*(x - mean);
    return (1/sqrt(2*M_PI*variance)) * exp (-1*MeanKuadrat/(2*variance));
}

double MATH::Jarak(double x1,double y1,double x2, double y2)
{
    	return sqrt((x1-x2)*(x1-x2) + (y1 - y2)*(y1 - y2));
}

double MATH::EuclideanDistance(double p1, double p2, double p3, double q1, double q2,double q3)
{
	double x = (p1 - q1)*(p1 - q1);
	double y = (p2 - q2)*(p2 - q2);
	double z = pow(min(fabs(min(p3, q3) - max(p3, q3)), (360+min(p3, q3)) - max(p3, q3)),2);
	return sqrt(x+y+z);
}

double MATH::Sudut(double x1, double y1, double x2, double y2)
{
    double deltax = (x2-x1);
    double deltay = (y2-y1);
    if (deltax == 0)
    {
        if (deltay < 0)
            return 90;
        else if (deltay >= 0)
            return -90;
    }
    else if (deltax < 0)
    {
        if (deltay < 0)
            return 180 - RadiansToDegrees(atan(fabs(deltay/deltax)));
        else if (deltay >= 0)
            return RadiansToDegrees(atan(fabs(deltay/deltax))) - 180;
    }
    else if (deltax > 0)
    {
        if (deltay < 0)
            return RadiansToDegrees(atan(fabs(deltay/deltax)));
        else if (deltay >= 0)
            return 0 - RadiansToDegrees(atan(fabs(deltay/deltax)));
    }
}

/**
* @param numToRound , variable that wanted to be rounded.
* @param multiplie, increament variabel
* fungsi ini gunanya untuk ngebuletin nilai
*/
int MATH::roundNumber(double numToRound, int multiplie)
{
    if(multiplie == 0)
        return int(numToRound);

    //printf(numToRound+(multiplie/2));
    if(numToRound+double(multiplie)/2 < int(numToRound)+multiplie)
        return int(numToRound);
    else
        return int(numToRound) + multiplie;
}

double MATH::getAngleToPoint(double x, double y)
{
    if (x == 0 && y == 0)
	return 0;

    double sudut = acos(x/(sqrt(x*x+y*y)));
    if (y < 0)
    {
        sudut *= -1;
    }
    sudut = sudut/M_PI*180;
    if (sudut < 0)
        sudut += 360;
    return sudut;
}

double MATH::randomGaussian(double mean, double stdev)
{
	double R1,R2;
	//R1 = (double)rand()/(double)(RAND_MAX);
	//R2 = (double)rand()/(double)(RAND_MAX);
	R1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
	R2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
	//printf("R1, R2 = %f, %f \n", R1, R2);
	//return mean + stdev*cos(2*3.14*R1)*sqrt(-log(R2));
	return mean + stdev*sin(2*3.14*R2)*sqrt(-2*log(R1));
}

