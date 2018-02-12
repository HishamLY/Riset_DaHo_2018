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

#include "Localization.h"
#include "MassCalibration.h"
#include "TeamCommunication.h"

#include "LinePercept.h"
#include "RegionPercept.h"
#include "Regionizer.h"
#include "RegionAnalyzer.h"
#include "LineSpots.h"


#include "BallSpots.h"
#include "LinePerceptor.h"
#include "Serial.h"
#include "Compass.h"
#include "Imu.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/CM730"
#define U2D_DEV_NAME1       "/dev/Compass"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

////////////////////////////
// Beginning of Function and Procedure Declaration
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
  if(obj.X != -1 && obj.Y != -1)
    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 255, 255), 0);
  else
    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 0, 0), 0);
}

void update_LED_EYE_1(Point2D obj)
{
  if(obj.X != -1 && obj.Y != -1)
    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 0, 0), 0);
  else
    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 0, 0), 0);
}

void delay(unsigned long ms)
{
  usleep(ms*1000);
}


// End of Function and Procedure Declaration
//////////////////////////

// Variabel Declaration
//Team Comunication Variables
TeamCommunication *teamCom;
char receivedDataTeamCom[64];
//Serial
Serial *serial = new Serial();
char *serialRecv = (char*)malloc(sizeof(char));
//Image Variables
Image *rgb_output;
Image *img;
//Listen Game Controller, used in StatusCheck.cpp & GameController.cpp
extern bool ListenGameController;
bool startKickOff(true);//used for kick off, extern in GameController.cpp & Behavior.cpp
bool afterStatic(false);//used for team com
bool beforeStatic(false);//used for team com
//threading
pthread_t tid;//team comunication
pthread_t tid1;//localization & compass
// Variabel Declaration
std::vector<Point2D> point_of_balls, border, blobCenter;
Point2D center_of_ball;

void* TeamComListening(void *arg)
{
  int resetCounter = 0;
  while(teamCom->receiver->recv(receivedDataTeamCom, 64) > 0)
  {
    char temp[10];
    memset(temp, '\0', 10);
    memcpy(temp, receivedDataTeamCom, 4);

    if(strcmp(temp, teamCom->DataHeader.c_str()) == 0)
      teamCom->Process(receivedDataTeamCom);

    if(resetCounter > 50)
    {
      resetCounter = 0;
      teamCom->InitTeamStatus();
    }
    resetCounter++;
  }
}

void* LocalizationUpdate(void *arg)
{
  while(1)
  {
    Compass::GetInstance()->processData(serial);
    Imu::GetInstance()->ReadIMU();
  }
}

int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
		
		// Maybe useless code, Refactor 12-02-18
    /* img = new Image(740, 540, Image::RGB_PIXEL_SIZE);
    if( jpeg_utils::read_jpeg_file( img, "Field.jpg" ) > 0 )
    {
      printf("Success Read File Image\n");

    }
    else
      return -1; */

//========== Serial Initialize ========================
   	int Ret; // Used for return values
//================================================

    Head::GetInstance()->LoadINISettings(ini);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // Set default camera settings
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ColorFinder* yellow_finder = new ColorFinder();
    ColorFinder* blue_finder = new ColorFinder();
    ColorFinder* field_finder = new ColorFinder();
    ColorFinder* white_finder = new ColorFinder();

    ball_finder->LoadINISettings(ini, "Orange");
    httpd::ball_finder = ball_finder;
    yellow_finder->LoadINISettings(ini, "Yellow");
    httpd::yellow_finder = yellow_finder;
    blue_finder->LoadINISettings(ini, "Blue");
    httpd::blue_finder = blue_finder;
    field_finder->LoadINISettings(ini, "Green");
    httpd::field_finder = field_finder;
    white_finder->LoadINISettings(ini, "White");
    httpd::white_finder = white_finder;

  CameraCalibration* cam_calib = new CameraCalibration();
  cam_calib->LoadINISettings(ini, "CameraCalibration");
  httpd::cam_calibration = cam_calib;

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
  ImageCoordinateSystem* imgcoord = new ImageCoordinateSystem();
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
      linux_cm730.SetPortName(U2D_DEV_NAME0);
      if(MotionManager::GetInstance()->Initialize(&cm730) == false)
      {
          printf("Fail to initialize Motion Manager!\n");
          return 0;
      }
    }

/* ================== Compass Initialize============ */

    // Open serial port
    Ret = serial->Open(U2D_DEV_NAME1,9600);// Open serial link at 57600 bauds 9600
    if (Ret!=1) {      // If an error occured...
        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
        Ret = serial->Open(U2D_DEV_NAME1,9600);
 				//return Ret;                                                         // ... quit the application
    }
    Compass::GetInstance()->LoadINISettings(ini);
    printf ("Serial port opened successfully!\n");
    int sp(0), kompensator, cmps_now, error, compassError;//variables for compass

/* =================End of Initialize ======================*/

/* ========== Imu Initialize ===================+ */
 Imu::GetInstance()->Initialize(&cm730);

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

/* ============ Imu Calibration ================== */
  Imu::GetInstance()->CalibrateIMU();

// action Initialization
// LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
  Action::GetInstance()->Start(9);
  while(Action::GetInstance()->IsRunning()) usleep(8*1000);
// Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

// ======== GAME CONTROLLER ========
  GameController* gameController = new GameController();
  gameController->LoadINISettings(ini);
  char receivedData[256];

	// Port 3838 used for communication with Game Controller
  Receiver* receiver = new Receiver(3838);
  receiver->LoadINISettings(ini);
  receiver->set_non_blocking(true);
// ==================================

// ====== TEAM COMMUNICATION ========
// Code has not been used until this time
  /* teamCom = new TeamCommunication();
  teamCom->LoadINISettings(ini, "Team Communication");
  teamCom->CreateConnection();

  char sendDataTeamCom[256];
  int TeamComCounter = 0;
  bool TeamComData = false;
  teamCom->receiver->set_non_blocking(false); */
//===================================

// ==== TEAM COMMUNICATION THREAD ====
  /* int err = pthread_create(&(tid), NULL, &TeamComListening, NULL);
  if (err != 0)
    printf("\ncan't create thread : [%s]", strerror(err));
  else
    printf("\n Thread created succesfully\n"); */

// ===== Compass Disability ===========
int CountCompass = 0;
static int MaxCountCompass = 15;//20
// ====================================

//========= PAN & TILT ==============
//USED FOR TEAM COMUNICATION, COMPARE TILT AMONG ROBOTS
double tilt = 0;
double pan = 0;

//========== edge field ===============
	int CountEdgeOfField = 0;

 //=========== OBJECT DISTANCE ============
  Vector2<int> ppfball;
 //=======================================

	timeval ti;
	double tStart = ti.tv_sec+(ti.tv_usec/1000000.0), tEnd;
  while(1)
  {

    printf("\t\t\t\t\t Tilt =%lf\tPan = %lf\n",MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT),MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN));
    //printf("\t\t\t\t\t Tilt = %lf\n",MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT));
    gettimeofday(&ti, NULL);
    tEnd=ti.tv_sec+(ti.tv_usec/1000000.0);
    tStart = tEnd;

    LinuxCamera::GetInstance()->CaptureFrame();
    memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
    Behavior::GetInstance()->CheckStatus();
    StatusCheck::Check(cm730);
    Compass::GetInstance()->processData(serial);
       // =================================GAME CONTROLLER ==========================================
    if(receiver->recv(receivedData, 256) == 116 && ListenGameController)//116
    {
//    if(receiver->check_receiver())
//    {
      char temp[10];
      memset(temp, '\0', 10);
      memcpy(temp, receivedData, 4);

      if(strcmp(temp, GAMECONTROLLER_STRUCT_HEADER) == 0)
      {
        gameController->parseData(receivedData);
        gameController->showInterface();
        gameController->Process();
      }
//    }
    }
  // =======================EOF Game Controller==============================================

  printf("Setpoint Kalibrasi = %d\n ",sp);

  //=========== Check Status Button ==================//
  //NANTI SET POINT DI SET LAGI PAS TANDING
  if(StatusCheck::m_is_started == 1)
  {
    switch(StatusCheck::m_cur_mode)
    {
      case READY:
        { gameController->m_old_gamestate = STATE_INITIAL;
        gameController->m_last_penalty = PENALTY_NONE;
        break; }
    case SOCCER_YELLOW_POST:
        gameController->m_target_goalcolour = GOAL_YELLOW;
        break;
    case SOCCER_BLUE_POST:
        gameController->m_target_goalcolour = GOAL_YELLOW;
        break;
    case PENALTY_KICK:
        break;
    case GAME_CONTROLLER:
        break;
    }

    if (sp > 128)
    {
      kompensator = sp - 128;
    }
    else
    {
      kompensator = 128 - sp;
    }
  }
  //=================================================//

  //================ m_is_started == 0 ==========================
  else if(StatusCheck::m_is_started == 0)
  {
    //=================== Compass Set point =========================
    //UNTUK NGESET SET POINT KOMPASS

    sp = Compass::GetInstance()->GetCompassOrientation8Bit();
    if (sp > 128)
    {
      kompensator = sp - 128;
    }
    else
    {
      kompensator = 128 - sp;
    }

    cmps_now = (int) Compass::GetInstance()->GetCompassOrientation8Bit();
    if (sp > 128)
    {
      if (cmps_now >= kompensator)
        error = cmps_now - kompensator - 128;
      else
        error = 127 + cmps_now - kompensator;
    }
    else
    {
      if (cmps_now >= 255 - kompensator)
        error = cmps_now + kompensator - 383;
      else
        error = cmps_now + kompensator -128;
    }

    //===============================================================

    //========== filtering image ================================
    ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    white_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

    for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
    {
      if(ball_finder->m_result->m_ImageData[i] == 1)
      {
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
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
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 255;
      }
      else if(field_finder->m_result->m_ImageData[i] == 1)
      {
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
      }
      else if(blue_finder->m_result->m_ImageData[i] == 1)
      {
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
        rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
      }
    }
    //============= EOF filtering image =====================================

    //================== BEHAVIOR ===============================

    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 0, 0), 0);

    Behavior::GetInstance()->set_ready_state();
    Behavior::GetInstance()->Process(Point2D(-1,-1));
    //=======================================================

    // ============ WHEN READY STATE, SEND Point (-1,-1) ==========
    /* int n = sprintf(sendDataTeamCom,"%s#%d#%lf#%lf#%lf", teamCom->DataHeader.c_str(), teamCom->PlayerStatus, Head::GetInstance()->GetTiltAngle(), -1.0, -1.0);
    if(n > 0)
    {
      if(teamCom->sender->send(sendDataTeamCom, n));
    }
    teamCom->InitTeamStatus(); */

    //============= CEK HORIZON ====================================
    double yHorizon = Head::GetInstance()->GetHorizon();
    //===================================================

    //================= KALIBRASI JARAK ===========================
  	border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
    blobCenter = ball_finder->getBlobCenter(rgb_output, border);
		if (point_of_balls.size() > 0) {
			center_of_ball = blobCenter.front();
		} else {
			center_of_ball = point_of_balls.back();
		}
    point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);

    //======== edge detection =================
    bool EdgeOfField = false;

    int Xmin = field_finder->Xmin;
    int Xmax = field_finder->Xmax;
    int Ymin = field_finder->Ymin;
    int Ymax = field_finder->Ymax;

    if(abs(pan) < 90)
    {
      if(yHorizon < 5)//jika horizon di atas, artinya robot agak nunduk
      {
        if(Ymin > Camera::HEIGHT*0.25)
        {
          CountEdgeOfField++;
        }
      }
      else if(Xmin > 100 || Xmax < Camera::WIDTH - 100)
      {
        CountEdgeOfField++;
      }
      else if(Head::GetInstance()->GetTiltAngle() < -10 && yHorizon < 5)
      {
        if(Ymin >10)
        {
          CountEdgeOfField++;
        }
      }
    }
    else
    {
      CountEdgeOfField = 0;
    }

    if(CountEdgeOfField > 0)
    { 
			//benar-benar di pinggir lapangan
      EdgeOfField = true;
      CountEdgeOfField = 0;
    }

    if(EdgeOfField)
      update_LED_EYE_1(Point2D(10,10));
    else
      update_LED_EYE_1(Point2D(-1,-1));

      //========== eof edge detection =========

    gameController->m_target_goalcolour = GOAL_YELLOW;
    //================ IMage processing ===========================
    goalPerceptor->Process(rgb_output, yHorizon, gameController->m_target_goalcolour);
    update_LED_EYE(GoalPercept::GetInstance()->Center);
    //================================
    Draw::Line(rgb_output,Point2D(0,yHorizon), Point2D(Camera::WIDTH,yHorizon), ColorRGB(255,0,0));
    Draw::Line(rgb_output,Point2D(0,yHorizon-1), Point2D(Camera::WIDTH,yHorizon-1), ColorRGB(255,0,0));
    Draw::Line(rgb_output,Point2D(0,yHorizon-2), Point2D(Camera::WIDTH,yHorizon-2), ColorRGB(255,0,0));

    //========================== Behavior ================================

      if(GoalPercept::GetInstance()->Status == GoalPercept::UNKNOWN_POST)// || GoalPercept::GetInstance()->Owner == GoalPercept::OWN_GOAL)
      {
        GoalPercept::GetInstance()->Center.X = -1;
        GoalPercept::GetInstance()->Center.Y = -1;
      }

      Vector2<int> ppfGoal;
      Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->Center.X, (int)GoalPercept::GetInstance()->Center.Y, *cameramatrix, ppfGoal);

    streamer->send_image(rgb_output);

    continue; //langsung ke while satu lagi, ga usah ke bawah lagi
  }
  //======================= EOF  m_is_started == 0 ==========================

  //========================== Walking Enable ============================
  if(Behavior::GetInstance()->get_state() != GOBACK_STATE && Behavior::GetInstance()->get_state() != DEFENSE_STATE && Behavior::GetInstance()->get_state() != CHECK_POST_STATE  && StatusCheck::m_cur_mode != PENALTY_KICK && Behavior::GetInstance()->get_state() != KICK_STATE  && Behavior::GetInstance()->get_state() != PENALTY_KICK)
  {
		//kalo state defense, check post dan passball, walkingnya ga di Start.
    Walking::GetInstance()->Start();
  }
  //================== EOF WALKING ENABLE =============================

  //=============== YHorizon ======================
  double yHorizon = Head::GetInstance()->GetHorizon();
  //============================================

  if(StatusCheck::m_cur_mode == PENALTY_KICK)
  {
    field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
    Vector2<int> ppfball;
    Geometry::calculatePointOnField((int)ball_finder->m_center.X, (int)ball_finder->m_center.Y, *cameramatrix, ppfball);

    Behavior::GetInstance()->set_penalty_kick_state();

    border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
    blobCenter = ball_finder->getBlobCenter(rgb_output, border);
		if (point_of_balls.size() > 0) {
			center_of_ball = blobCenter.front();
		} else {
			center_of_ball = point_of_balls.back();
		}
    point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);

    tracker.Process(ball_finder->m_center);

		//process(center object, compass, jarak bola)
    Behavior::GetInstance()->Process(ball_finder->m_center, 0, ppfball); 
    update_LED_EYE(ball_finder->m_center);
  }
  else if(StatusCheck::m_cur_mode == SOCCER_YELLOW_POST
    || StatusCheck::m_cur_mode == SOCCER_BLUE_POST
    || StatusCheck::m_cur_mode == GAME_CONTROLLER)
  {
    //================= LOCALIZE PENALTY MARK ================================
    /*else*/  
		if (Behavior::GetInstance()->get_state() == LOCALIZE_PENALTY_MARK)
		{
			field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
      white_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
      ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

			for(int i = 1; i < rgb_output->m_NumberOfPixels; i++)//luqman
			{
      	if(ball_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
				else if(field_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
				else if(white_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 0;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
			}
			std::vector<Point2D> border = field_finder->getConvexFieldBorders(rgb_output, 10, 10, yHorizon);
      white_finder->getBlobLine(rgb_output, border);
      goalPerceptor->Process(rgb_output, yHorizon, gameController->m_target_goalcolour);
      Behavior::GetInstance()->Process(white_finder->m_center);
      update_LED_EYE(white_finder->m_center);
    }
    //============================ EOF LOCALIZE PENALTY MARK ==================================

    //=========== CHECK_POST =+======================
		else if (Behavior::GetInstance()->get_state() == CHECK_POST_STATE
				|| Behavior::GetInstance()->get_state() == LOCALIZE_GOALPOST)
		{
			//======== FILTERING IMAGE ===========================
			yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

			for(int i = 1; i < rgb_output->m_NumberOfPixels; i++)//luqman
			{

				if(yellow_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
				else if(field_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
				else if(blue_finder->m_result->m_ImageData[i] == 1)
				{
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
					rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
				}
      }

    //============== Compass ================================

			cmps_now = (int) Compass::GetInstance()->GetCompassOrientation8Bit();
			if (sp > 128)
			{
				if (cmps_now >= kompensator)
					error = cmps_now - kompensator - 128;
				else
					error = 127 + cmps_now - kompensator;
			}
			else
			{
				if (cmps_now >= 255 - kompensator)
					error = cmps_now + kompensator - 383;
				else
					error = cmps_now + kompensator -128;
			}

			compassError = error;

    //========================== EOF COMPASS ==============================

    //================ IMage processing ===========================
      goalPerceptor->Process(rgb_output, yHorizon, gameController->m_target_goalcolour);
      update_LED_EYE(GoalPercept::GetInstance()->Center);
    //================================

    //========================== Behavior ================================
      if(GoalPercept::GetInstance()->Status == GoalPercept::UNKNOWN_POST)
      {
        GoalPercept::GetInstance()->Center.X = -1;
        GoalPercept::GetInstance()->Center.Y = -1;
      }

      Vector2<int> ppfGoal;
      Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->Center.X, (int)GoalPercept::GetInstance()->Center.Y, *cameramatrix, ppfGoal);
      Behavior::GetInstance()->Process(GoalPercept::GetInstance()->Center, compassError, ppfGoal);
			Draw::Line(rgb_output,Point2D(0,yHorizon), Point2D(Camera::WIDTH,yHorizon), ColorRGB(255,0,0));
	    Draw::Line(rgb_output,Point2D(0,yHorizon-1), Point2D(Camera::WIDTH,yHorizon-1), ColorRGB(255,0,0));
  	  Draw::Line(rgb_output,Point2D(0,yHorizon-2), Point2D(Camera::WIDTH,yHorizon-2), ColorRGB(255,0,0));

    //================================ EOF BEHAVIOR


		}
    //=================== EOF CHECK POST =

    //========== OTHER BEHAVIOR =====================
      else if(Behavior::GetInstance()->get_state() == NORMAL_STATE
      || Behavior::GetInstance()->get_state() == DEFENSE_STATE
      || Behavior::GetInstance()->get_state() == FIRST_PENALTY_SEARCHING
      || Behavior::GetInstance()->get_state() == STATIC_STATE
      || Behavior::GetInstance()->get_state() == PASS_BALL_STATE)
      {
        field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

				for(int i = 1; i < rgb_output->m_NumberOfPixels; i++)//luqman
				{
      		if(ball_finder->m_result->m_ImageData[i] == 1)
					{
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
					}
      		else if(field_finder->m_result->m_ImageData[i] == 1)
					{
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
						rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
					}
      	}

      //=============== image processing  =======================
			border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
			blobCenter = ball_finder->getBlobCenter(rgb_output, border);
			if (point_of_balls.size() > 0) {
				center_of_ball = blobCenter.front();
			} else {
				center_of_ball = point_of_balls.back();
			}
			point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);
      //=====================================================

      //======== edge detection =================
      bool EdgeOfField = false;

      int Xmin = field_finder->Xmin;
      int Xmax = field_finder->Xmax;
      int Ymin = field_finder->Ymin;
      int Ymax = field_finder->Ymax;
    
      if(abs(pan) < 90)
      {
	      if(Xmin > 75 || Xmax < Camera::WIDTH - 75)
        {
					CountEdgeOfField++;
        }
        if(Head::GetInstance()->GetTiltAngle() < -10 && yHorizon < 5)
        {
          if(Ymin > Camera::HEIGHT*0.25)
          {
						CountEdgeOfField++;
          }
          else
          {
            CountEdgeOfField--;
            if(CountEdgeOfField <= 0)
              CountEdgeOfField = 0;
          }
        }
        if(Ymin > Camera::HEIGHT*0.8 && yHorizon < Camera::HEIGHT* 0.4)
        {
					CountEdgeOfField++;
        }
        else
          {
            CountEdgeOfField--;
            if(CountEdgeOfField <= 0)
              CountEdgeOfField = 0;
          }
      }
      else
      {
        CountEdgeOfField = 0;
      }
      if(CountEdgeOfField > 2)
      {
				//benar-benar di pinggir lapangan
        EdgeOfField = true;
      }

      if(EdgeOfField)
        update_LED_EYE_1(Point2D(10,10));
      //========== eof edge detection =========

    //============== Compass ================================

			cmps_now = (int) Compass::GetInstance()->GetCompassOrientation8Bit();
			if (sp > 128)
				{
					if (cmps_now >= kompensator)
						error = cmps_now - kompensator - 128;
					else
						error = 127 + cmps_now - kompensator;
				}
				else
				{
					if (cmps_now >= 255 - kompensator)
            error = cmps_now + kompensator - 383;
          else
            error = cmps_now + kompensator -128;
				}

        compassError = error;

    //========================== EOF COMPASS ==============================

      //======= tilt & pan ==================
      if(ball_finder->m_center.X != -1 && ball_finder->m_center.Y != -1)
      {
				//KALO NEMU BOLA, BARU CEK TILT
        tilt = Head::GetInstance()->GetTiltAngle();//ambil tilt robot sekarang
        pan = Head::GetInstance()->GetPanAngle();//ambil tilt robot sekarang
				//==================== hitung jarak =========================
				Geometry::calculatePointOnField((int)ball_finder->m_center.X, (int)ball_finder->m_center.Y, *cameramatrix, ppfball);
	      //====================================================
      }
      else
      {
        pan = 0;
        tilt = 1000;//tilt dibuat besar, supaya yang lain ngejar
      }

      Behavior::GetInstance()->Process(ball_finder->m_center, EdgeOfField, ppfball);//process(center object, compass, jarak bola)
      update_LED_EYE(ball_finder->m_center);
      //=================================================

    }//end of behavioral state
    //============== EOF BEHAIVORAL ==========================

    //=============== GO BACK =========================

    else if( Behavior::GetInstance()->get_state() == APROACHING_BALL
      || Behavior::GetInstance()->get_state() == DRIBBLE_STATE
      || Behavior::GetInstance()->get_state() == KICK_OFF
      || Behavior::GetInstance()->get_state() == KICK_STATE
      || Behavior::GetInstance()->get_state() == TURN_AROUND_BALL_STATE)
    {
      field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
      ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

      for(int i = 1; i < rgb_output->m_NumberOfPixels; i++)
       {
        if(ball_finder->m_result->m_ImageData[i] == 1)
        {
          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
        }
        else if(field_finder->m_result->m_ImageData[i] == 1)
        {
           rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
           rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
           rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
        }
      }

      //======= Compass Disable ===============

           cmps_now = Compass::GetInstance()->GetCompassOrientation8Bit();
        if (sp > 128)
                          {
                                  if (cmps_now >= kompensator)
            error = cmps_now - kompensator - 128;
          else
            error = 127 + cmps_now - kompensator;
                          }
                    else
                          {
                                  if (cmps_now >= 255 - kompensator)
            error = cmps_now + kompensator - 383;
          else
            error = cmps_now + kompensator -128;
                          }

        compassError = error;

      //=============== image processing  =======================
      border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
			blobCenter = ball_finder->getBlobCenter(rgb_output, border);
			if (point_of_balls.size() > 0) {
				center_of_ball = blobCenter.front();
			} else {
				center_of_ball = point_of_balls.back();
			}
			point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);
      //=====================================================

      //==================== hitung jarak =========================
      Geometry::calculatePointOnField((int)ball_finder->m_center.X, (int)ball_finder->m_center.Y, *cameramatrix, ppfball);
      //====================================================

      //======= tilt & pan ==================
      if(ball_finder->m_center.X != -1 && ball_finder->m_center.Y != -1)
      {//KALO NEMU BOLA, BARU CEK TILT
        tilt = Head::GetInstance()->GetTiltAngle();//ambil tilt robot sekarang
        pan = Head::GetInstance()->GetPanAngle();//ambil tilt robot sekarang

      }
      else
      {
        pan = 0;
        tilt = 1000;//tilt dibuat besar, supaya yang lain ngejar

      }
      //========== eof ===============

      //============ Behavior =============================
      Behavior::GetInstance()->Process(ball_finder->m_center, compassError, ppfball);//process(center object, compass, jarak bola)
      update_LED_EYE(ball_finder->m_center);
      //=================================================

    }


    else if(Behavior::GetInstance()->get_state() == GOBACK_STATE)
    {
                  yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
                  blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
      field_finder->FilteringImageErotionDilation(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
      ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            for(int i = 1; i < rgb_output->m_NumberOfPixels; i++)
            {
      if(ball_finder->m_result->m_ImageData[i] == 1)
                        {
                                rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                                rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
                                rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                        }
                  else if(blue_finder->m_result->m_ImageData[i] == 1)
                  {
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                  }
                  else if(yellow_finder->m_result->m_ImageData[i] == 1)
                  {
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                  }
      else if(field_finder->m_result->m_ImageData[i] == 1)
                  {
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                          rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                  }
      }

    //printf("State adalah Go Back\n");

      if(Behavior::GetInstance()->get_goback_state() == TURNING)
      {
            //printf("State Go Back - CHECK_TURNING\n");

        border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
				blobCenter = ball_finder->getBlobCenter(rgb_output, border);
				if (point_of_balls.size() > 0) {
					center_of_ball = blobCenter.front();
				} else {
					center_of_ball = point_of_balls.back();
				}
				point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);

        cmps_now = (int)Compass::GetInstance()->GetCompassOrientation8Bit();
        if (sp > 128)
                           {
                                   if (cmps_now >= kompensator)
            error = cmps_now - kompensator - 128;
          else
            error = 127 + cmps_now - kompensator;
                           }
                     else
                           {
                                   if (cmps_now >= 255 - kompensator)
            error = cmps_now + kompensator - 383;
          else
            error = cmps_now + kompensator -128;
                           }
        compassError = error;

        if(ball_finder->m_center.X != -1 && ball_finder->m_center.Y != -1)
        {//KALO NEMU BOLA, BARU CEK TILT
          tilt = Head::GetInstance()->GetTiltAngle();//ambil tilt robot sekarang
          pan = Head::GetInstance()->GetPanAngle();//ambil tilt robot sekarang
        }
        else
        {
          pan = 0;
          tilt = 1000;//tilt dibuat besar, supaya yang lain ngejar
        }
      //==================== hitung jarak =========================
      Geometry::calculatePointOnField((int)ball_finder->m_center.X, (int)ball_finder->m_center.Y, *cameramatrix, ppfball);
      //====================================================

      Behavior::GetInstance()->Process(ball_finder->m_center, compassError, ppfball);//process(center object, compass, jarak bola)
      }
      else if(Behavior::GetInstance()->get_goback_state() == CHECK_OWN_GOAL)
      {
      //================ IMage processing ===========================
        //printf("State Go Back - CHECK_OWN GOAL\n");

        goalPerceptor->Process(rgb_output, yHorizon, gameController->m_target_goalcolour);
        update_LED_EYE(GoalPercept::GetInstance()->Center);

      //========================== Behavior ================================
        Draw::Line(rgb_output,Point2D(0,yHorizon), Point2D(Camera::WIDTH,yHorizon), ColorRGB(255,0,0));
        Draw::Line(rgb_output,Point2D(0,yHorizon-1), Point2D(Camera::WIDTH,yHorizon-1), ColorRGB(255,0,0));
        Draw::Line(rgb_output,Point2D(0,yHorizon-2), Point2D(Camera::WIDTH,yHorizon-2), ColorRGB(255,0,0));

        if(GoalPercept::GetInstance()->Status == GoalPercept::UNKNOWN_POST)
        {
          GoalPercept::GetInstance()->Center.X = -1;
          GoalPercept::GetInstance()->Center.Y = -1;
        }

        Vector2<int> ppfGoal;
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->CenterFoot.X, (int)GoalPercept::GetInstance()->CenterFoot.Y, *cameramatrix, ppfGoal);
        Behavior::GetInstance()->Process(GoalPercept::GetInstance()->CenterFoot, compassError, ppfGoal);
      }
      else if(Behavior::GetInstance()->get_goback_state() == SEARCHING
        || Behavior::GetInstance()->get_goback_state() == GOING_BACK)
      {

        border = field_finder->getConvexFieldBorders(rgb_output, 60, 60, yHorizon);
				blobCenter = ball_finder->getBlobCenter(rgb_output, border);
				if (point_of_balls.size() > 0) {
					center_of_ball = blobCenter.front();
				} else {
					center_of_ball = point_of_balls.back();
				}
				point_of_balls = ball_finder->detect(rgb_output, 20, 100, center_of_ball, yHorizon);

        //======== edge detection =================
        bool EdgeOfField = false;

        int Xmin = field_finder->Xmin;
        int Xmax = field_finder->Xmax;
        int Ymin = field_finder->Ymin;
        int Ymax = field_finder->Ymax;
        /*
      printf("XXXXXXmin = %d \n", Xmin);
      printf("XXXXXXmax = %d \n", Xmax);
      printf("YYYYYYmin = %d \n", Ymin);
      printf("YYYYYYmax = %d \n", Ymax);
      */
        if (abs(pan) < 70)
        {
          /*        if(yHorizon < 5)//jika horizon di atas, artinya robot nunduk
        {
          if(Ymin > Camera::HEIGHT*0.25)
          {
            printf("PINGGIR LAPANGAN GUYS..............\n");
            CountEdgeOfField++;
          }
        }
*/ if (Xmin > 75 || Xmax < Camera::WIDTH - 75)
          {
            //printf("PINGGIR LAPANGAN GUYS..............\n");
            CountEdgeOfField++;
        }
        if(Head::GetInstance()->GetTiltAngle() < -10 && yHorizon < 5)
        {
          if(Ymin >Camera::HEIGHT*0.25)
          {
                              //printf("PINGGIR LAPANGAN GUYS..............\n");
            CountEdgeOfField++;
          }
        }

        if(Ymin > Camera::HEIGHT*0.8 && yHorizon < Camera::HEIGHT * 0.4)
        {
                              //printf("PINGGIR LAPANGAN GUYS..............\n");
            CountEdgeOfField++;

        }

      }
      else
      {
        CountEdgeOfField = 0;
      }
      if(CountEdgeOfField > 3)
      {//benar-benar di pinggir lapangan
        EdgeOfField = true;
        CountEdgeOfField = 0;
      }

      if(EdgeOfField)
        update_LED_EYE_1(Point2D(10,10));
      //========== eof edge detection =========

        if(ball_finder->m_center.X != -1 && ball_finder->m_center.Y != -1)
        {//KALO NEMU BOLA, BARU CEK TILT
          tilt = Head::GetInstance()->GetTiltAngle();//ambil tilt robot sekarang
          pan = Head::GetInstance()->GetPanAngle();//ambil tilt robot sekarang
          EdgeOfField = false;
        }
        else
        {
          pan = 0;
          tilt = 1000;//tilt dibuat besar, supaya yang lain ngejar
        }
//==================== hitung jarak =========================
      Geometry::calculatePointOnField((int)ball_finder->m_center.X, (int)ball_finder->m_center.Y, *cameramatrix, ppfball);
      //====================================================
//        EdgeOfField = false;
        Behavior::GetInstance()->Process(ball_finder->m_center, EdgeOfField, ppfball);//process(center object, compass, jarak bola)
      }
    }

    //================= TEAM COMUNICATION SENDING DATA ================================
       if(Behavior::GetInstance()->get_state() == NORMAL_STATE
      || Behavior::GetInstance()->get_state() == STATIC_STATE
      || Behavior::GetInstance()->get_state() == GOBACK_STATE)
       {
    
       }
       else if(Behavior::GetInstance()->get_state() == DRIBBLE_STATE
      || Behavior::GetInstance()->get_state() == TURN_AROUND_BALL_STATE
      || Behavior::GetInstance()->get_state() == FIRST_PENALTY_SEARCHING
      || Behavior::GetInstance()->get_state() == KICK_STATE
      || Behavior::GetInstance()->get_state() == CHECK_POST_STATE
      || Behavior::GetInstance()->get_state() == APROACHING_BALL
      || Behavior::GetInstance()->get_state() == PASS_BALL_STATE)
       {
      
       }
       else
       {
        
       }
          //======================= EOF TEAM COMUNOCATION SENDING DATA====================================
      }//end of m_cur_mode == kuning, biru, ato game controller
      streamer->send_image(rgb_output);
    }
    // end of while(1)

    return 0;
}
// main(void)
