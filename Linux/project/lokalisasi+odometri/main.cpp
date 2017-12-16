/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>                  // STanDard Input Output library
#include <unistd.h>                 // provides access to the POSIX operating system API. function declaration includes chdir(), readlink()
#include <limits.h>                 // defines constants with the limits of fundamental integral types for the specific system and compiler implementation used.
#include <string.h>
#include <libgen.h>                 // declaration includes function dirname
#include <signal.h>                 // declaration includes signal SIGABRT, SIGTERM, SIGQUIT and SIGINT and function signal()

#include "mjpg_streamer.h"
#include "jpeg_utils.h"
#include "LinuxDARwIn.h"
#include "Localization.h"
#include "DebugDrawings.h"
#include "StatusCheck.h"
#include "VisionMode.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

void change_current_dir()
{
  char exepath[1024] = {0};
  if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)      // int readlink(const char *path, char *buf, size_t bufsize); The readlink() function places the contents of the symbolic link referred to by path in the buffer buf which has size bufsize. If the number of bytes in the symbolic link is less than bufsize, the contents of the remainder of buf are unspecified; Upon successful completion, readlink() returns the count of bytes placed in the buffer. Otherwise, it returns a value of -1, leaves the buffer unchanged, and sets errno to indicate the error.
    {
      if(chdir(dirname(exepath)))                 // int chdir(const char *path); chdir - change working directory; The chdir() function causes the directory named by the pathname pointed to by the path argument to become the current working directory; that is, the starting point for path searches for pathnames not beginning with /.
        fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
  exit(0);
}

int main(void)
{
  // void (*signal(int sig, void (*func)(int)))(int); ==> function to handle signal in a multi-threaded process.
  // this function handle signal, and then execute function signalhandler to exit program
  signal(SIGABRT, &sighandler);                   // SIGABRT : Process abort signal
  signal(SIGTERM, &sighandler);                   // SIGTERM : Termination signal
  signal(SIGQUIT, &sighandler);                   // SIGQUIT : Terminal quit signal
  signal(SIGINT, &sighandler);                    // SIGINT  : Terminal interrupt signal

  change_current_dir();

  minIni* ini = new minIni(INI_FILE_PATH);
  Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
  Image* img = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

  LinuxCamera::GetInstance()->Initialize(0);
  LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
  LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

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

  Localize->BuatSampel(800);
  Localize->BuatCluster();
  Localize->IsiCluster();

  int k = 0, j = 0;

  Localize->robotPosition.body.x = 85;
  Localize->robotPosition.body.y = 270;
  Localize->robotPosition.body.angle = 0;

  // INIT SAMPLES
  Localize->BuatSampel(800);
  Localize->BuatCluster();
  Localize->IsiCluster();

  Draw::gambarRobot(img, Point2D(Localize->robotPosition.body.x, Localize->robotPosition.body.y), Localize->robotPosition.body.angle, 0);


  BallTracker tracker = BallTracker();
  BallFollower follower = BallFollower();

  httpd::ini = ini;

  //////////////////// Framework Initialize ////////////////////////////
  if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
      linux_cm730.SetPortName(U2D_DEV_NAME1);
      if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
          printf("Fail to initialize Motion Manager!\n");
          return 0;
        }
    }

  Walking::GetInstance()->LoadINISettings(ini);

  MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
  MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
  MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  motion_timer->Start();
  /////////////////////////////////////////////////////////////////////

  MotionManager::GetInstance()->LoadINISettings(ini);

  int firm_ver = 0;
  if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
      fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
      exit(0);
    }

  if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
      Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
      fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
      fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
      exit(0);
#endif
    }
  else if(27 <= firm_ver)
    {
#ifdef MX28_1024
      fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
      fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
      exit(0);
#else
      Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
  else
    exit(0);

  Action::GetInstance()->m_Joint.SetEnableBody(true, true);
  MotionManager::GetInstance()->SetEnable(true);

  cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

  LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
  Action::GetInstance()->Start(15);
  while(Action::GetInstance()->IsRunning()) usleep(8*1000);

  double xmove=0, ymove=0, amove=0;

  while(1)
    {
      StatusCheck::Check(cm730);
      jpeg_utils::read_jpeg_file( img, "Field.jpg" );

      Point2D ball_pos, red_pos, yellow_pos, blue_pos;

      printf("robot's position   = %.1f,%.1f,%.1f \n", Localize->robotPosition.body.x, Localize->robotPosition.body.y, Localize->robotPosition.body.angle);

      LinuxCamera::GetInstance()->CaptureFrame();
      memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

      if(StatusCheck::m_cur_mode == READY || StatusCheck::m_cur_mode == VISION)
        {

        }
      else if(StatusCheck::m_cur_mode == SOCCER)
        {

        }

      Draw::gambarRobot(img, Point2D(Localize->robotPosition.body.x, Localize->robotPosition.body.y), Localize->robotPosition.body.angle, 0 );
      streamer->send_image(img);

      if(StatusCheck::m_is_started == 0)
        continue;

      switch(StatusCheck::m_cur_mode)
        {
        case READY:
          break;
        case SOCCER:
          if(Action::GetInstance()->IsRunning() == 0)
            {
              /*
              Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
              Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

              if(Walking::GetInstance()->IsRunning() == false)
              {
                  Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                  Walking::GetInstance()->Start();
              }
              else
              {
                  Walking::GetInstance()->X_MOVE_AMPLITUDE = 10;
              }
              */
              xmove = Walking::GetInstance()->Get_X_Moved()/10;
              ymove = Walking::GetInstance()->Get_Y_Moved()/10;
              amove = Walking::GetInstance()->Get_A_Moved() *180/31.4;
              printf("moved (x,y,a)= %.1f,%.1f,%.1f\n");
              Localize->Odometri(xmove, ymove, amove);
            }
          break;
        case MOTION:
          break;
        case VISION:
          break;
        }
    }

  return 0;
}
