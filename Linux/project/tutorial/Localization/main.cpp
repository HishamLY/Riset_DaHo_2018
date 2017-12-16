/**
 * @file :main.cpp
 * @author Imre Nagi
 *  Created on: 2013. Des
 *  Main program for localization SImulation. Just see how it works :p
 */

#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "Localization.h"
#include "LinuxMotionTimer.h"
#include "LinuxNetwork.h"
#include "jpeg_utils.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "DebugDrawings.h"
#include <unistd.h>

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define delaytime 100

using namespace std;

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
  ofstream myfile;
  myfile.open ("output.txt");

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

  int targetx = 0;
  int targety = 130;


  Localization* Localize = new Localization(50.0f,200.0f,600,400);
  Localize->LoadINISettings(ini, "Localization");

  // INIT SAMPLES
  Localize->Init();
  Draw::gambarLandmark(img, Localize->Landmarks);
  
 // printf("%f\n", Localize->robotPosition.body.x);  
  Draw::gambarSampel(img, Localize->Samples);
  Draw::gambarRobot(img, Point2D(Localize->robotPosition.body.x, Localize->robotPosition.body.y), Localize->robotPosition.body.angle);
  Draw::gambarCluster(img, Localize->Clusters);
  Draw::gambarPosPredicted(img, Point2D(Localize->hipotesis.body.x, Localize->hipotesis.body.y), Localize->hipotesis.body.angle);
  
  vector<double> tes;
  double total = 0;
  double error;

  tes = Localize->VariansiCluster();
  error = MATH::EuclideanDistance(Localize->hipotesis.body.x, Localize->hipotesis.body.y, Localize->hipotesis.body.angle, Localize->robotPosition.body.x, Localize->robotPosition.body.y, Localize->robotPosition.body.angle);
  for (int i=0; i<tes.size(); i++)
    {
      total += tes[i];
    }
  //printf("%.2f, %.2f\n",total, error);

  double arah = (double)(rand()%359);
  int k = 0, j = 0;
  int selector=0, enter=0;
  delay(delaytime);

  myfile << "real.x , real.y , real.angle, est.x , est.y , est.angle , error, variance\n";
  while(1)
  //for (int loop=0; loop<100; loop++)
    {
      jpeg_utils::read_jpeg_file( img, "Field.jpg" );
      k +=1;


      if (Localize->bobotmax < 1e-20)
      {
          Localize->BuatSampel(Localize->total_samples);
          Localize->K = 10;
      }

      fprintf(stderr, "Real(%.2f,%.2f,%.2f) Estimated (%.2f,%.2f,%.2f) Error(%.2f), Variansi(%.2f)     \r", Localize->robotPosition.body.x, Localize->robotPosition.body.y, Localize->robotPosition.body.angle,Localize->hipotesis.body.x, Localize->hipotesis.body.y, Localize->hipotesis.body.angle, error, total);
      myfile << Localize->robotPosition.body.x <<","<<Localize->robotPosition.body.y<<","<<Localize->robotPosition.body.angle<<","<<Localize->hipotesis.body.x<<","<<Localize->hipotesis.body.y<<","<<Localize->hipotesis.body.angle<<","<<error<<","<<total<<"\n";

      if (j==5){
          j = 0;
          Localize->Odometri(0,0, arah + (double)(rand()%359));
        }
      //else if (j ==4)
        //Localize->Odometri(20, 0, 70);
      else
        Localize->Odometri(20, 0, 0);

      j++;

      Localize->BacaSensor();
      Localize->Pembobotan();
      Localize->SamplingUlang();
      Localize->BuatCluster(); 
      Localize->IsiCluster();  

      
      total = 0;
      
      tes = Localize->VariansiCluster();
      error = MATH::EuclideanDistance(Localize->hipotesis.body.x, Localize->hipotesis.body.y, Localize->hipotesis.body.angle, Localize->robotPosition.body.x, Localize->robotPosition.body.y, Localize->robotPosition.body.angle);
      for (int i=0; i<tes.size(); i++)
        {
          total += tes[i];
        }
      //printf("%.2f, %.2f\n",total, error);


      Draw::gambarLandmark(img, Localize->Landmarks);
      Draw::gambarSampel(img, Localize->Samples);
      Draw::gambarRobot(img, Point2D(Localize->robotPosition.body.x, Localize->robotPosition.body.y), Localize->robotPosition.body.angle);
      //Draw::gambarCluster(img, Localize->Clusters);
      Draw::gambarPosPredicted(img, Point2D(Localize->hipotesis.body.x, Localize->hipotesis.body.y), Localize->hipotesis.body.angle);

      if (MATH::Jarak(Localize->hipotesis.body.x, Localize->hipotesis.body.y, targetx, targety) < 10)
        targetx = (targetx + 600)%1200;

      arah = MATH::Sudut(Localize->hipotesis.body.x, Localize->hipotesis.body.y, targetx, targety) - Localize->hipotesis.body.angle;


      streamer->send_image(img);
      delay(300);

    }

  return 0;
}

/* //testing transformasi dari titik di kamera ke lapangan

  Point2D pfOnField1(Localize->getRobotToPointOnField(pf1, Point2D(200,200), angles));
  Point2D pfOnField2(Localize->getRobotToPointOnField(pf2, Point2D(200,200), angles));
  Point2D pfOnField3(Localize->getRobotToPointOnField(pf3, Point2D(200,200), angles));
  Point2D pfOnField4(Localize->getRobotToPointOnField(pf4, Point2D(200,200), angles));

  Draw::Line(img, pfOnField1, pfOnField2, ColorRGB(ColorClasses::red));
  Draw::Line(img, pfOnField2, pfOnField4, ColorRGB(ColorClasses::red));
  Draw::Line(img, pfOnField3, pfOnField4, ColorRGB(ColorClasses::red));
  Draw::Line(img, pfOnField1, pfOnField3, ColorRGB(ColorClasses::red));
*/

/*
      tes.clear();
      tes = Localize->VariansiCluster();  //nilai variansi masih salah
      error = MATH::EuclideanDistance(Localize->hipotesis.body.x, Localize->hipotesis.body.y, Localize->hipotesis.body.angle, Localize->robotPosition.body.x, Localize->robotPosition.body.y, Localize->robotPosition.body.angle);

      total = 0;
      for (int i=0; i<tes.size(); i++)
        {
          total += tes[i];
        }
      printf("variansi %.1f error %.1f\n",total, error);
*/

      /*
if (selector==1)
      {
      if (j==5){
          j = 0;
          Localize->Odometri(0,0, arah + (double)(rand()%360));
        }
      else if (j ==4)
        Localize->Odometri(0, 10, 70);
      else
        Localize->Odometri(20, 0, 10);
      j = j + 1;

        Localize->BacaSensor();
        printf("baca sensor\n");
      }
      else if(selector==2 )
      {
        printf("Sebelum Pembobotan\n");
        Localize->Pembobotan();
        printf("Pembobotan\n");
      }
      else if (selector==3 )
      {
        printf("Sebelum Sampling ulang\n");
        Localize->SamplingUlang();
        printf("Sampling ulang\n");
      }
      else if (selector==4 )
      {
        Localize->BuatCluster();
        printf("Buat Cluster\n");
      }
      else if (selector==5 )
      {
        Localize->IsiCluster();
        printf("Isi Cluster\n");
      }

      if (selector==5)
        selector =0;
      else
        selector++;

*/

/*
  vector<Point2D> camera;
  Point2D pf1(20,20); camera.push_back(pf1);
  Point2D pf2(20,-20); camera.push_back(pf2);
  Point2D pf3(150,100); camera.push_back(pf3);
  Point2D pf4(15gimp0,-100); camera.push_back(pf4);
*/
