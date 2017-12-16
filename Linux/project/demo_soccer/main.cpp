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

#include "Behavior.h"
#include "GameController.h"
#include "LinuxNetwork.h"

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
/*	if(obj.X != -1 && obj.Y != -1)
        	cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 255, 255), 0);
        else
                cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 0, 0), 0);
*/
}

TeamCommunication *teamCom;
char receivedDataTeamCom[64];

extern bool ListenGameController;

pthread_t tid;

bool startKickOff(true);
bool ObstacleDetected(false);
bool afterStatic(false);
bool beforeStatic(false);

void* TeamComListening(void *arg)
{
	while(teamCom->receiver->recv(receivedDataTeamCom, 64) > 0)
	{
		char temp[10];
		memset(temp, '\0', 10);
		memcpy(temp, receivedDataTeamCom, 4);

//		printf("%s\n",receivedDataTeamCom);

		if(strcmp(temp, teamCom->DataHeader.c_str()) == 0)
			teamCom->Process(receivedDataTeamCom);
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

    ColorFinder* white_finder = new ColorFinder();
    white_finder->LoadINISettings(ini, "White");

//    httpd::ini = ini;

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

    //LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    Action::GetInstance()->Start(15);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

// ======== GAME CONTROLLER ========
    GameController* gameController = new GameController();
    gameController->LoadINISettings(ini);
    char receivedData[256];

    Receiver* receiver = new Receiver(3838);
    receiver->LoadINISettings(ini);
    receiver->set_non_blocking(true);
//  ====== TEAM COMMUNICATION ======
    teamCom = new TeamCommunication();
    teamCom->LoadINISettings(ini, "Team Communication");
    teamCom->CreateConnection();

    char sendDataTeamCom[256];

    teamCom->receiver->set_non_blocking(false);

//  ====== TEAM COMMUNICATION THREAD ======
	int err = pthread_create(&(tid), NULL, &TeamComListening, NULL);

        if (err != 0)
            printf("\ncan't create thread :[%s]", strerror(err));
        else
            printf("\n Thread created successfully\n");
// =====================================

timeval ti;
double tStart = ti.tv_sec+(ti.tv_usec/1000000.0), tEnd;

    while(1)
    {
/*	gettimeofday(&ti, NULL);
        tEnd=ti.tv_sec+(ti.tv_usec/1000000.0);
	printf("time elapsed = %lf\n",tEnd - tStart);
	tStart = tEnd;
*/
	//printf("Pan = %lf\n",Head::GetInstance()->GetPanAngle());
//	Behavior::GetInstance()->CheckStatus();
//	printf("X = %lf, Y = %lf, A = %lf\n", Walking::GetInstance()->X_MOVE_AMPLITUDE, Walking::GetInstance()->Y_MOVE_AMPLITUDE, Walking::GetInstance()->A_MOVE_AMPLITUDE );

        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

	if(receiver->recv(receivedData, 256) == 116 && ListenGameController)
        {
		//if(receiver->check_receiver())
		//{
			char temp[10];
			memset(temp, '\0', 10);
                	memcpy(temp, receivedData, 4);

                	if(strcmp(temp, GAMECONTROLLER_STRUCT_HEADER) == 0)
                	{
                        	gameController->parseData(receivedData);
                        	//gameController->showInterface();
                        	gameController->Process();
                	}
		//}
        }

	StatusCheck::Check(cm730);

	switch(StatusCheck::m_cur_mode)
	{
		case READY:
			//printf("MODE READY\n");
			gameController->m_old_gamestate = STATE_INITIAL;
			gameController->m_last_penalty = PENALTY_NONE;
			break;
		case SOCCER_YELLOW_POST:
			//printf("MODE SOCCER TARGET YELLOW POST\n");
			gameController->m_target_goalcolour = GOAL_YELLOW;
			break;
		case SOCCER_BLUE_POST:
			//printf("MODE SOCCER TARGET BLUE POST\n");
			gameController->m_target_goalcolour = GOAL_BLUE;
			break;
		case PENALTY_KICK:
			//printf("MODE PENALTY KICK\n");
                        break;
		case GAME_CONTROLLER:
			//printf("MODE GAME CONTROLLER\n");
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

		Behavior::GetInstance()->set_ready_state();
		Behavior::GetInstance()->Process(Point2D(-1,-1));

		streamer->send_image(rgb_output);
		continue;
	}

	if(Walking::GetInstance()->IsRunning() == false && StatusCheck::m_cur_mode != PENALTY_KICK && Behavior::GetInstance()->get_state() != PENALTY_KICK_STATE && Behavior::GetInstance()->get_state() != PASS_BALL_STATE)
                        Walking::GetInstance()->Start();

	double yHorizon = Head::GetInstance()->GetHorizon();

	if(StatusCheck::m_cur_mode == PENALTY_KICK)
        {
		field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

		Behavior::GetInstance()->set_penalty_kick_state();
                //Draw::Line(rgb_output,Point2D(0,yHorizon), Point2D(Camera::WIDTH,yHorizon), ColorRGB(255,0,0));

                std::vector<Point2D> border = field_finder->getConvexFieldBorders(rgb_output, 10, 10, yHorizon);
                std::vector<Point2D> ball = ball_finder->getBlobCenter(rgb_output, border);
                Behavior::GetInstance()->Process(ball_finder->m_center);
		
		update_LED_EYE(ball_finder->m_center);
        }
        else if(StatusCheck::m_cur_mode == SOCCER_YELLOW_POST 
		|| StatusCheck::m_cur_mode == SOCCER_BLUE_POST 
		|| StatusCheck::m_cur_mode == GAME_CONTROLLER)
        {
	  if(Behavior::GetInstance()->get_state() == LOCALIZE_PENALTY_MARK)
	  {
		yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
                blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		white_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

		for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
                {
                        if(blue_finder->m_result->m_ImageData[i] == 1)
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
		goalPerceptor->Process(rgb_output, yHorizon   );
		//Draw::Cross(rgb_output, GoalPercept::GetInstance()->Center, 10, ColorRGB(255,255,0));
		//Draw::Cross(rgb_output, white_finder->m_center, 10, ColorRGB(255,255,0));
		Behavior::GetInstance()->Process(white_finder->m_center);
		update_LED_EYE(white_finder->m_center);
	  }
	  else if(Behavior::GetInstance()->get_state() == CHECK_POST_STATE || Behavior::GetInstance()->get_state() == LOCALIZE_GOALPOST)
          {
		yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

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
			else if(field_finder->m_result->m_ImageData[i] == 1)
                	{
                        	rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 0;
                        	rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                        	rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                	}
		}

		goalPerceptor->Process(rgb_output, yHorizon);

		update_LED_EYE(GoalPercept::GetInstance()->Center);

		if(GoalPercept::GetInstance()->Status == GoalPercept::UNKNOWN_POST)
		{
			GoalPercept::GetInstance()->Center.X = -1;
			GoalPercept::GetInstance()->Center.Y = -1;
		}

		Behavior::GetInstance()->Process(GoalPercept::GetInstance()->Center);

		//printf("CENTER(%lf,%lf)\n",GoalPercept::GetInstance()->Center.X, GoalPercept::GetInstance()->Center.Y);

		if(startKickOff)
		{
			std::vector<Point2D> border = field_finder->getConvexFieldBorders(rgb_output, 10, 10, yHorizon);

		        //for(std::vector<Point2D>::iterator iter = border.begin(); iter!=border.end(); iter++)
		        //{
		        //        Draw::Circle(rgb_output, Point2D((*iter).X,(*iter).Y), 1, ColorRGB(255,0,0));
		        //}
		        field_finder->getObstacle(rgb_output, border);

			if(field_finder->m_center.X != -1 && field_finder->m_center.Y != -1)
				ObstacleDetected = true;
		}
          }	
	  else if(Behavior::GetInstance()->get_state() == NORMAL_STATE
		|| Behavior::GetInstance()->get_state() == DRIBBLE_STATE
		|| Behavior::GetInstance()->get_state() == TURN_AROUND_BALL_STATE
		|| Behavior::GetInstance()->get_state() == FIRST_PENALTY_SEARCHING
		|| Behavior::GetInstance()->get_state() == STATIC_STATE
		|| Behavior::GetInstance()->get_state() == PENALTY_KICK_STATE
		|| Behavior::GetInstance()->get_state() == PASS_BALL_STATE)
          {
		//Draw::Line(rgb_output,Point2D(0,yHorizon), Point2D(320,yHorizon), ColorRGB(255,0,0));

		ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

		//ball_finder->Erode_Dilate();

		std::vector<Point2D> border = field_finder->getConvexFieldBorders(rgb_output, 10, 10, yHorizon);
        std::vector<Point2D> ball = ball_finder->getBlobCenter(rgb_output, border);
		Behavior::GetInstance()->Process(ball_finder->m_center);

		update_LED_EYE(ball_finder->m_center);
	  }

	}

        streamer->send_image(rgb_output);
    }

    return 0;
}
