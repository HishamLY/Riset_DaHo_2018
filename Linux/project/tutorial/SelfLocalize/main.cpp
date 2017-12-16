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

#include "SelfLocalize.h"
#include "LinuxMotionTimer.h"
#include "LinuxNetwork.h"
#include "jpeg_utils.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "DebugDrawings.h"
#include <unistd.h>

#define INI_FILE_PATH       "localization.ini"
#define delaytime 100

void delay(unsigned long ms)
{
    usleep(ms*1000);
}

void change_current_dir()
{
  char exepath[1024] = {0};
  if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    chdir(dirname(exepath));
}

int main(void)
{
  srand((unsigned)time(0));

  printf( "\n===== Self Localization for DARwIn =====\n\n");

  change_current_dir();

  minIni* ini = new minIni(INI_FILE_PATH);

  //=========== LOADING FILE JPEG ===================
  Image* img = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
  if( jpeg_utils::read_jpeg_file( img, "Field.jpg" ) > 0 )
    {
      printf("Success Read File Image\n");

    }
  else return -1;
  mjpg_streamer* streamer = new mjpg_streamer(img->m_Width, img->m_Height);

  SelfLocalize* Localize = new SelfLocalize();
  Localize->LoadINISettings(ini, "Localization");
  Localize->Init();
  Localize->generateRandomSamples( 1.000, Localize->total_samples);

  delay(delaytime);

  while(1)
    {
      jpeg_utils::read_jpeg_file( img, "Field.jpg" );

      Localize->checkMovementModel(Movement);
      // SENSOR MODEL
      Localize->checkSensorModel(TypeOfGoal, PartOfGoalPostSeen, Distance, Dis);
      // RESAMPLING
      Localize->resampling();
      // ADD LOSS SAMPLE
      Localize->addSamples();
      
      int color, radius = 4;
      for(int index=0; index<Localization::Samples.size(); index++)
      {
        Point2D Center(Localization::Samples[index].PositionAndOrientation.x, Localization::Samples[index].PositionAndOrientation.y);
        ColorRGB rgb(ColorClasses::black);
        Draw::Circle(img, Center, 3, rgb);
      }
      streamer->send_image(img);
      delay(500);
    }

  return 0;
}
