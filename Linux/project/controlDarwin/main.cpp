/**
 * Created on: 12 Januari 2014
 * @author Imre Nagi (13210024)
 * @file main.cpp
 * Use this fuckin' demo to control the robot manually.
 * See the control hints on web-browser
 */

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include "jpeg_utils.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "Image.h"

#include "StatusCheck.h"
#include "VisionMode.h"
#include "Camera.h"

#include "LinuxNetwork.h"
#include "MassCalibration.h"

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
  if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
      if(chdir(dirname(exepath)))
        fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
  exit(0);
}

void update_LED_EYE(Point2D obj)
{
}

int main(void)
{
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  change_current_dir();

  minIni* ini = new minIni(INI_FILE_PATH);

  Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

  LinuxCamera::GetInstance()->Initialize(0);
  LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
  LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

  mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

  ColorFinder* ball_finder = new ColorFinder();
  ball_finder->LoadINISettings(ini, "Orange");
  httpd::ball_finder = ball_finder;

  ColorFinder* yellow_finder = new ColorFinder();
  yellow_finder->LoadINISettings(ini, "Yellow");
  httpd::yellow_finder = yellow_finder;

  ColorFinder* blue_finder = new ColorFinder();
  blue_finder->LoadINISettings(ini, "Blue");
  httpd::blue_finder = blue_finder;

  ColorFinder* field_finder = new ColorFinder();
  field_finder->LoadINISettings(ini, "Green");
  httpd::field_finder = field_finder ;

  ColorFinder* white_finder = new ColorFinder();
  white_finder->LoadINISettings(ini, "White");
  httpd::white_finder = white_finder;

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
  Action::GetInstance()->Start(9);
  while(Action::GetInstance()->IsRunning()) usleep(8*1000);

  Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
  Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

  while(1)
    {
	 StatusCheck::Check(cm730);
	

      LinuxCamera::GetInstance()->CaptureFrame();
      memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

      switch(StatusCheck::m_cur_mode)
        {
        case READY:
          printf("MODE READY\n");
          break;
        case SOCCER_YELLOW_POST:
          printf("MODE SOCCER TARGET YELLOW POST\n");
          break;
        case SOCCER_BLUE_POST:
          printf("MODE SOCCER TARGET BLUE POST\n");
          break;
        case PENALTY_KICK:
          printf("MODE PENALTY KICK\n");
          break;
        case GAME_CONTROLLER:
          printf("MODE GAME CONTROLLER\n");
          break;
        }

     
     if (StatusCheck::m_cur_mode == READY
	 || StatusCheck::m_cur_mode == SOCCER_YELLOW_POST)
     {
        ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
          yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
          blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
          white_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
          field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

	  for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
              if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
              else if(blue_finder->m_result->m_ImageData[i] == 1)
                {
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 0;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 255;
                }
              else if(yellow_finder->m_result->m_ImageData[i] == 1)
                {
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
            else if(white_finder->m_result->m_ImageData[i] == 1)
                {
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 0;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
              else if(field_finder->m_result->m_ImageData[i] == 1)
                {
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                  rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
            }
     }
     else if(StatusCheck::m_cur_mode == PENALTY_KICK)
        {}
      else if(StatusCheck::m_cur_mode == SOCCER_YELLOW_POST
              || StatusCheck::m_cur_mode == SOCCER_BLUE_POST
              || StatusCheck::m_cur_mode == GAME_CONTROLLER)
        {}

     streamer->send_image(rgb_output);

	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;

//     if(StatusCheck::m_is_started == 0)
//         continue;

    }

  return 0;
}
