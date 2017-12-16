
/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

//#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
//#include <string.h>

//#include "LinuxMotionTimer.h"
//#include "LinuxCamera.h"
#include "jpeg_utils.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "DebugDrawings.h"
#include "MathFunction.h"

using namespace Robot;
using namespace std;

#define INI_FILE_PATH       "odometry.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Odometry for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

//=========== LOADING FILE JPEG ===================

    Image* img = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    //Image* frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    if( jpeg_utils::read_jpeg_file( img, "Field.jpg" ) > 0 )
	{
	    printf("Success Read File Image\n");
	}
	else return -1;

	mjpg_streamer* streamer = new mjpg_streamer(img->m_Width, img->m_Height);

//============ LOOPING PROCESS ====================
    while(1)
    {
        jpeg_utils::read_jpeg_file( img, "Field.jpg" );

        //frame = img;
        Draw::Circle(img, Point2D(615, 65), 8, ColorRGB(255, 0, 0));
	Draw::Circle(img, Point2D(615, 365), 8, ColorRGB(255, 0, 0));
//        double x = x0 + 40 * cos (MATH::DegreesToRadians(angleHeading));
//        double y = y0 + 40 * sin (MATH::DegreesToRadians(angleHeading));
        streamer->send_image(img);
        usleep(10);
    }

    return 0;
}
