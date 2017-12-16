/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: KRSBI ITB
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

#include "StatusCheck.h"
#include "VisionMode.h"
#include "Camera.h"

#include "Behavior.h"
#include "GameController.h"
#include "LinuxNetwork.h"
#include "MassCalibration.h"
#include "TeamCommunication.h"

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

bool startKickOff(true);
bool ObstacleDetected(false);
bool afterStatic(false);
bool beforeStatic(false);

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

  //------additional image processing object declaration-----
  Regionizer* regionizer = new Regionizer();
  regionizer->LoadINISettings(ini, "Regionizer");
  RegionPercept* regionpercept = new RegionPercept();
  RegionAnalyzer* regionanalyzer = new RegionAnalyzer();
  BallSpots* ballspots = new BallSpots();
  LineSpots* linespots = new LineSpots();
  LinePercept* linepercept = new LinePercept();
  LinePerceptor* lineperceptor = new LinePerceptor();

  //------additional field dimensions object declaration----
  FieldDimensions* field_dim = new FieldDimensions();
  field_dim->load(ini);


  //------additional robot's model object declaration------
  TorsoMatrix* torsomatrix = new TorsoMatrix();
  TorsoMatrixPrev* torsomatrixprev = new TorsoMatrixPrev();
  TorsoMatrixProvider* torsomatrixprovider = new TorsoMatrixProvider();

  MassCalibration masses;
  RobotModelProvider* rbtmodelprov = new RobotModelProvider();
  RobotModel* rbtmodel = new RobotModel();

  //------additional code's for camera 3D equation-------
  RobotCameraMatrixProvider* robotcameramatrixprovider = new RobotCameraMatrixProvider();
  RobotCameraMatrix* robotcameramatrix = new RobotCameraMatrix();
  RobotCameraMatrixPrev* robotcameramatrixprev = new RobotCameraMatrixPrev();
  CameraMatrixProvider* cameramatrixprov = new CameraMatrixProvider();
  CameraMatrix* cameramatrix = new CameraMatrix();
  CameraMatrixPrev* cameramatrixprev = new CameraMatrixPrev();

  httpd::ini = ini;

  BallTracker tracker = BallTracker();
  BallFollower follower = BallFollower();

// === ADDITIONAL IMAGE PROCESSING ===

  GoalPerceptor* goalPerceptor = new GoalPerceptor();

// ===================================

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

  int value;
  cm730.WriteWord(JointData::ID_HEAD_PAN ,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_HEAD_TILT,  MX28::P_TORQUE_ENABLE, 0, 0);

  while(1)
    {
      //=============read the position of servo=====================//
      printf(" ID[%d]:", JointData::ID_HEAD_PAN);
		if(cm730.ReadWord(JointData::ID_HEAD_PAN, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
		}
		else
            printf("----");

      printf(" ID[%d]:", JointData::ID_HEAD_TILT);
		if(cm730.ReadWord(JointData::ID_HEAD_TILT, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
		}
		else
            printf("----");

      printf("\n");
      //=============================================================//

      LinuxCamera::GetInstance()->CaptureFrame();
      memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

      StatusCheck::Check(cm730);

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

      if(StatusCheck::m_is_started == 0)
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

          cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 0, 0), 0);


          //-----robot model----------
          rbtmodelprov->update(*rbtmodel);

          //-----camera matrix and robotcameramatrix -------
          //robotcameramatrixprovider->update(*robotcameramatrixprev);
          robotcameramatrixprovider->update(*robotcameramatrix);
          cameramatrixprov->update(*cameramatrix,*torsomatrix, *robotcameramatrix);

          //-----Torso and TorsoProvider -------
          //di torsomatrixprovider masih pakai robotmodel dan torsomatrix sendiri. masukin dari sini.
          torsomatrixprovider->update(*torsomatrix, *rbtmodel); //disini angleX dan Y masih konstanta

          regionizer->update(*regionpercept,rgb_output);
          regionpercept->draw(rgb_output);
          regionanalyzer->update(*ballspots, *regionpercept, rgb_output);
          regionanalyzer->update(*linespots); //linespots is updated

          //lineperceptor->update(*linepercept, *linespots, rgb_output, field_display);

          streamer->send_image(rgb_output);
          continue;
        }

      if(StatusCheck::m_cur_mode == PENALTY_KICK)
        {

        }
      else if(StatusCheck::m_cur_mode == SOCCER_YELLOW_POST || StatusCheck::m_cur_mode == SOCCER_BLUE_POST || StatusCheck::m_cur_mode == GAME_CONTROLLER)
        {

        }
      streamer->send_image(rgb_output);

    }

  return 0;
}
