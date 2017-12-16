/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <math.h>
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "GameController.h"

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/CM730"

using namespace Robot;

bool checkPost(false);
bool checkBall(false);

extern int countStep;
extern bool startCountStep;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

bool normalState(true);
bool checkPostPosition(false);
bool turnAroundBall(false);
bool fitBallToKick(false);

Point2D middleborder(-1,-1);

int main(void)
{
    printf( "\n===== Ball following Tutorial for DARwIn =====\n\n");

    change_current_dir();

    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ColorFinder* field_finder = new ColorFinder();
    ColorFinder* yellow_post_finder = new ColorFinder();
    ColorFinder* blue_post_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini,"Orange");
    field_finder->LoadINISettings(ini, "Green");
    yellow_post_finder->LoadINISettings(ini, "Yellow");
    blue_post_finder->LoadINISettings(ini, "Blue");
    httpd::ball_finder = ball_finder;
    httpd::yellow_finder = yellow_post_finder;
    httpd::blue_finder = blue_post_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
        follower.DEBUG_PRINT = true;

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

	printf("Press the ENTER key to begin!\n");
	getchar();
	
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	GoalPerceptor* goalPerceptor = new GoalPerceptor();

    while(1)
    {
	Head::GetInstance()->MoveByAngle(0, 50.0);
        LinuxCamera::GetInstance()->CaptureFrame();

	ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        field_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        yellow_post_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        blue_post_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

//	ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        
//	tracker.Process(ball_finder->GetPositionMoment());
//	if(!checkPost)
//        	follower.Process(tracker.ball_position);

	rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
            else if(blue_post_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255;
            }
            else if(yellow_post_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
            else if(field_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }
/*	
	if(checkPost)
	{
		if(Walking::GetInstance()->IsRunning() == false)
			Walking::GetInstance()->Start();

		if(tracker.ball_position.X > 0 || tracker.ball_position.Y > 0)
                {
			double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			double pan_range = Head::GetInstance()->GetLeftLimitAngle();
			double pan_percent = pan / pan_range;

			double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
			double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
			double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
			double tilt_percent = (tilt - tilt_min) / tilt_range;
			if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE + 5))
			{
				Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
			}
			else
			{
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
			}

			if(pan > -7 && pan < 7)
			{
                        	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 25;
                        	Walking::GetInstance()->A_MOVE_AMPLITUDE = 15;
				Walking::GetInstance()->A_MOVE_AIM_ON = true;
			}
			else if(pan > 7)
			{
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 25;
                                Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
                                Walking::GetInstance()->A_MOVE_AIM_ON = true;
			}
			else if(pan < -7)
			{
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
                                Walking::GetInstance()->A_MOVE_AMPLITUDE = 15;
                                Walking::GetInstance()->A_MOVE_AIM_ON = true;
			}
		}
	}*/
/*
//	Point2D pos;
//        LinuxCamera::GetInstance()->CaptureFrame();

//        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
//	ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

//        tracker.Process(ball_finder->GetPositionMoment());
//	startCountStep = true;
	Walking::GetInstance()->Start();
//	printf("TOTAL STEP = %d\r",countStep);
//        follower.Process(tracker.ball_position);

        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }
*/
	Head::GetInstance()->MoveByAngle(0, 50.0);
        double NewTiltAngle = Head::GetInstance()->GetTiltAngle();
        double yHorizon = (Camera::HEIGHT/2.0) +  (NewTiltAngle - 50) * ((double)Camera::HEIGHT / Camera::VIEW_V_ANGLE);
        if(yHorizon < 0)
                yHorizon = 2;

	goalPerceptor->Process(rgb_ball, yHorizon, GOAL_YELLOW);

//	std::vector<Point2D> border = field_finder->getConvexFieldBorders(rgb_ball, 10, 10, yHorizon);

//        Draw::Line(rgb_ball, Point2D(0, yHorizon), Point2D(Camera::WIDTH, yHorizon), ColorRGB(ColorClasses::red));
//        Draw::Cross(rgb_ball, border[(int)border.size()/2], 10, ColorRGB(ColorClasses::yellow));

        double angle_degree = (GoalPercept::GetInstance()->Foot.Y - yHorizon) * (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);
//        double angle_degree = (border[(int)border.size()/2].Y - yHorizon) * (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);
        double distance = 36.0 / tan(angle_degree * 3.14159265 / 180.0);
        printf("Distance = %lf\n",distance);

        streamer->send_image(rgb_ball);
    }

    return 0;
}
