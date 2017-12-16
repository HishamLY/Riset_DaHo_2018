
/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Localization.h"
#include "LinuxMotionTimer.h"
#include "LinuxNetwork.h"
#include "jpeg_utils.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "DebugDrawings.h"

#define INI_FILE_PATH       "localization.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Self Localization for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Localization* Localize = new Localization();

    Localize->LoadINISettings(ini, "Localization");
    // INIT SAMPLES
    Localize->generateRandomSamples( 1.000, Localize->total_samples);

//========== CREATING UDP SOCKET =================
    LinuxSocket* sock = new LinuxSocket();

    char receivedData[256];

    if(!sock->createUdp())
    {
        printf("Error creating socket!");
        return 1;
    }

    if(!sock->bind(3131))
    {
        printf("Error binding!");
        return 1;
    }

   // sock->set_non_blocking(true);
//=========== LOADING FILE JPEG ===================

    Image* img = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
//    Image* frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    if( jpeg_utils::read_jpeg_file( img, "Field.jpg" ) > 0 )
	{
	    printf("Success Read File Image\n");
	}
	else return -1;

	mjpg_streamer* streamer = new mjpg_streamer(img->m_Width, img->m_Height);
//============ LOOPING PROCESS ====================
    while(1)
    {
        if(sock->recv(receivedData, 256) != 0)
        {
            char *Delimit;
            jpeg_utils::read_jpeg_file( img, "Field.jpg" );
            Delimit = strtok((char*)receivedData,"#");
            int TypeOfGoal = atoi(Delimit);
            Delimit = strtok(NULL, "#");
            int PartOfGoalPostSeen = atoi(Delimit);
            Delimit = strtok(NULL, "#");
            double Distance = atof(Delimit);
	    Delimit = strtok(NULL, "#");
            int Dis = atof(Delimit);
	    Delimit = strtok(NULL, "#");
	    double Movement = atoi(Delimit);

            // MOVEMENT MODEL
            Localize->checkMovementModel(Movement);

            // SENSOR MODEL
            Localize->checkSensorModel(TypeOfGoal, PartOfGoalPostSeen, Distance, Dis);

            // RESAMPLING
            Localize->resampling();

            // ADD LOSS SAMPLE
            Localize->addSamples();

            // Get The Solution
            Localize->getSolution();

            int color, radius = 4;
            double heading_x, heading_y;
            printf("Solution (%i,%i,%i)\n",Localization::Solution.PositionAndOrientation.x,
                                            Localization::Solution.PositionAndOrientation.y,
                                             Localization::Solution.PositionAndOrientation.angle);
            for(int index=0; index<Localization::Samples.size(); index++)
            {
                Point2D Center(Localization::Samples[index].PositionAndOrientation.x, Localization::Samples[index].PositionAndOrientation.y);
                ColorRGB rgb(ColorClasses::black);
                Draw::Circle(img, Center, 3, rgb);
             }

            Draw::Circle(img, Point2D(Localization::Solution.PositionAndOrientation.x, Localization::Solution.PositionAndOrientation.y), 4, ColorRGB(ColorClasses::red));
            heading_x = Localization::Solution.PositionAndOrientation.x + 10 * cos(MATH::DegreesToRadians(Localization::Solution.PositionAndOrientation.angle));
            heading_y = Localization::Solution.PositionAndOrientation.y + 10 * sin(MATH::DegreesToRadians(Localization::Solution.PositionAndOrientation.angle));
            Draw::Line(img, Point2D(Localization::Solution.PositionAndOrientation.x, Localization::Solution.PositionAndOrientation.y), Point2D(heading_x, heading_y), ColorRGB(ColorClasses::red));
        }


        streamer->send_image(img);
    }

    return 0;
}

