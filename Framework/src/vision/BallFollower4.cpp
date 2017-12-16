/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "BallFollower.h"
#include "MotionStatus.h"
#include "Behavior.h"
#include "StatusCheck.h"
#include "Vector2.h"
#include <math.h>

#define PI (3.14159265)


using namespace Robot;

extern int countHeadSearchingCycle;
extern double panLastSeenPost;
int RandomKick = 0; 
static const double TiltCheckPost = 27.5;//dipake di processLocalizeGoalPost
int flag = 0;
bool FirstTouch = false;

BallFollower::BallFollower()
{

	CmpsSPTurn = 0;
	CmpsErrTurn = 0;
	
	// variabel dribble ball
	PostDist;
	pan_now;

	MaxBallTurn = 0; //dipake turning ball
	CountMaxTurnBall = 0; //dipake truning ball

	PostEqualZero = true;
	flagNemuBola = false;
	flagUdahMuter = false;
	turn = false;
	Goback = true;
	CountFirstTilt = 0; //dipake pas process aproaching ball
	MaxCountFirstTilt = 10;

	m_NoBallMaxCount = 10;
	m_NoBallCount = 0;
	m_KickBallMaxCount = 10;
	m_KickBallCount = 0;
	m_CountBallFound = 0;
	m_MaxCountBallFound = 10;

	panLastSeenBall = 10;
	tiltLastSeenBall = 40;
	LostCounter = 0;

	MaxLostCounter = 0;
	m_KickTopAngle = 0.0;	// -3.0, 40
	m_KickRightAngle = -25.0;
	m_KickLeftAngle = 25.0;

	m_FollowMaxFBStep = 20.0;	// 45, 55.0 Nilai max yang mungkin tercapai
 	m_FollowMinFBStep = 5.0;	// awalnya 10.0
	m_FollowMaxRLTurn = 20.0; // 30
	m_FollowMaxRLStep = 20;
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 2.0;	// 1.5	Increment
	m_UnitRLTurn = 5.0;	// 3.0
	m_UnitRLStep = 3;

	m_GoalFBStep = 0; // Pengganti X Moved
	m_GoalRLTurn = 0; // Pengganti A Moved
	m_GoalRLStep = 0; // Pengganti Y Moved
	m_FBStep = 0;
	m_RLTurn = 0;
	m_RLStep = 0;

	onesegitiga = 0; // ini variabel buat nentuin apakah  daritem udah bikin 1 segitiga
	MaxBallTurn = 0;
	FirstTilt = false;
	TiltAngle = 0;
	PanAngle = 0;
	Sizeball = 0;
	DEBUG_PRINT = false;
	KickDir = 3; // netral
	KickBall = 0;
	X_Moved = 0;
	Y_Moved = 0;
	A_Moved = 0;
	countCheckBall = 0;
	counterGetBall = 0;
	counterGetBallFound = 0;

}

BallFollower::~BallFollower() {}

void BallFollower::ProcessFirstPenaltySearching(Point2D ball_pos)
{
	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
	{
		if(m_NoBallCount > m_NoBallMaxCount && countHeadSearchingCycle > 2)
		{
			m_GoalFBStep = 20;
			m_GoalRLTurn = 0;
		}
		else
		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			m_NoBallCount++;
		}
	}
	else
	{
		m_NoBallCount = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle) //KALO BOLA BERADA DI TENGAH PAN
		{
			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE + 10))
			{
				if(ball_pos.Y < m_KickTopAngle + 10) //SUDAH DEKET DENGAN BOLA
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount) //KALO SUDAH BERADA DI DEKET BOLA, CARI GAWANG
					{
						m_FBStep = 0;
						m_RLTurn = 0;
						Behavior::GetInstance()->set_check_post_position();

						//Head::GetInstance()->LookUp();
						Head::GetInstance()->MoveByAngle(0, 40);
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					}
					else //KALO BELUM SIAP UNTUK TENDANG
					{
						m_KickBallCount++;
					}
				}
				else //BELUM DEKET
				{
					m_KickBallCount = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
				}
			}
			else
			{
				m_KickBallCount = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			}
		}
		else //MASIH JAUH
		{
			m_KickBallCount = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
		}
	}
	if(Walking::GetInstance()->IsRunning() == false)
	{
		m_FBStep = 0;
		m_RLTurn = 0;
		m_KickBallCount = 0;
		Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
		Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLTurn;
		Walking::GetInstance()->Start();
	}
	else
	{
		if(m_FBStep < m_GoalFBStep)
			m_FBStep += m_UnitFBStep;
		else if(m_FBStep > m_GoalFBStep)
			m_FBStep = m_GoalFBStep;
		Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

		if(m_RLTurn > 0 && m_GoalRLTurn < 0)
			m_RLTurn = 0;
		else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
			m_RLTurn = 0;

		if(m_RLTurn < m_GoalRLTurn)
			m_RLTurn += m_UnitRLTurn;
		else if(m_RLTurn > m_GoalRLTurn)
			m_RLTurn -= m_UnitRLTurn;
		Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
		
		//um
		// printf("UMMMM UMMMM UMMMM %f %f\n", m_RLStep, m_GoalRLTurn);
		// if (m_RLStep < m_GoalRLStep)
		// 	m_RLStep += m_UnitRLStep;
		// else if (m_RLStep > m_GoalRLStep)
		// 	m_RLStep -= m_UnitRLStep;
		// Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLStep;
	
	}
}

void BallFollower::ProcessSearchingBall()
{
	Walking::GetInstance()->Start();
	if(Behavior::GetInstance()->get_state() == NORMAL_STATE || Behavior::GetInstance()->get_state() == DEFENSE_STATE || Behavior::GetInstance()->get_state() == GOBACK_STATE)
	{
		if(EdgeOfField)
		{
			if(TurnDirection == KANAN)
			{
				panLastSeenBall = 1.0f; // diset agar nanti muternya jadi berubah arah kalo udah ketemu pinggir lapangan
				m_GoalRLTurn = 20;
				m_GoalRLStep = 0;
				m_GoalFBStep = 0;
			}
			else if(TurnDirection == KIRI)
			{
				panLastSeenBall = -1.0f;
				m_GoalRLTurn = -20;
				m_GoalRLStep = 0;
				m_GoalFBStep = 0;
			}
		}
		else
		{
			printf("CARI BOLA -----------\n");
			CountFirstTilt=0;

			if(KickDir == -1)
			{
				if(A_Moved > 90)
					KickDir = 0;
				else
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 20;
				}
			}
			else if(KickDir == 1)
			{
				if(A_Moved < -90)
					KickDir = 0;
				else
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = -20;
				}
			}
			else if(KickDir == 0)
			{
				if(X_Moved > (tiltLastSeenBall - Head::GetInstance()->GetBottomLimitAngle()) * 8 )//75 adalah konstanta
				{
					KickDir = 3;
					CountMaxTurnBall = 0;
					turn = true;
				}
				else
				{
					m_GoalFBStep = 20;
					m_GoalRLTurn = 0;
				}
			}
			else if(KickDir == 2)
			{
				if(abs(A_Moved) < panLastSeenBall)
				{
					m_GoalFBStep = 0;
					m_GoalRLStep = 0;
					m_GoalRLTurn = panLastSeenBall;
				        if(abs(m_GoalRLTurn) > 20)
						m_GoalRLTurn = panLastSeenBall * 20 /abs(panLastSeenBall);
				}
				else if (X_Moved < (tiltLastSeenBall - Head::GetInstance()->GetBottomLimitAngle()) * 8)
				{
					m_GoalFBStep = 20;
					m_GoalRLStep = 0;
					m_GoalRLTurn = 0;
				}
				else if(X_Moved >= (tiltLastSeenBall - Head::GetInstance()->GetBottomLimitAngle()) * 8)
				{
					turn = true;
					CountMaxTurnBall = 0;
					KickDir = 3;
				}
			}
			else
			{
				if(turn)
				{
					MaxBallTurn = 180;
					if(abs(CountMaxTurnBall) < MaxBallTurn)
						{
						m_GoalFBStep = 0;
						m_GoalRLStep = 0;
						m_GoalRLTurn = panLastSeenBall * 20 / abs(panLastSeenBall);
						CountMaxTurnBall += Walking::GetInstance()->Get_A_Moved() * 180 /PI;
					}
					else
					{
						MaxBallTurn = 0;
						CountMaxTurnBall = 0;
						turn = false;
					}
				}
				else
				{
					turn = false;
					if( countHeadSearchingCycle > 6)// kalo 3 cycle sekitaran 7 detik
					{
						m_GoalFBStep = 0;
						if(panLastSeenBall < 0)
						{
							m_GoalRLTurn = -20;
						}
						else if(panLastSeenBall > 0)
						{
							m_GoalRLTurn = 20;
						}
						else
						{
							X_Moved = 0;
							m_NoBallCount = 0;
							countHeadSearchingCycle = 0;
						}

						if(abs(A_Moved) > 120)
						{
							X_Moved = 0;
							A_Moved = 0;
							m_NoBallCount = 0;
							m_RLTurn = 0;
							m_GoalRLTurn = 0;
							countHeadSearchingCycle = 0;
							onesegitiga++;
							if(onesegitiga==3) {
								Behavior::GetInstance()->Bingung = true;
								onesegitiga = 0;
							}
						}
					}
					else   //MAJU SEKARANG
					{
						A_Moved = 0;
						m_NoBallCount++;
						m_GoalFBStep = 20;
						m_GoalRLTurn = 0;
					}
				}
			}

			if(m_GoalRLTurn < 0)
				TurnDirection = KANAN;
			else
				TurnDirection = KIRI;
		}
		// Kondisi biar masuk ke lokalisasi kasar
		//Behavior::GetInstance()->Bingung = true;

	}
	else if(Behavior::GetInstance()->get_state() == TURN_AROUND_BALL_STATE ||
		Behavior::GetInstance()->get_state() == APROACHING_BALL)
	{
		printf("BOLA HILANG\n");
		if(m_NoBallCount < 3)//untuk mengnolkan  X_Moved 
		{
			X_Moved = Walking::GetInstance()->Get_X_Moved();
			X_Moved = 0;
		}
		else
		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			m_GoalRLStep = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = -10;//diubah
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			if(panLastSeenBall < -5)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 10;
			else if(panLastSeenBall > 5)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = -10;

			X_Moved += Walking::GetInstance()->Get_X_Moved();
			if(X_Moved < -500)
			{
				Behavior::GetInstance()->set_normal_state();
				m_NoBallCount = 0;
				X_Moved = 0;
				KickDir = 2;
				turn = true;
			}	
		}
		m_NoBallCount++;
	}
	else if(Behavior::GetInstance()->get_state() == KICK_STATE || Behavior::GetInstance()->get_state() == DRIBBLE_STATE)
	{
		printf("BOLA HILANG\n");
		if(m_NoBallCount < 3)//untuk mengnolkan  X_Moved 
		{
			X_Moved = Walking::GetInstance()->Get_X_Moved();
			X_Moved = 0;
		}
		else
		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			m_GoalRLStep = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = -10;//diubah
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			if(panLastSeenBall < -5)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 10;
			else if(panLastSeenBall > 5)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = -10;

			X_Moved += Walking::GetInstance()->Get_X_Moved();
			if(X_Moved < -1000)
			{
				if(StatusCheck::m_cur_mode == SOCCER_YELLOW_POST
				|| StatusCheck::m_cur_mode == SOCCER_BLUE_POST
				|| StatusCheck::m_cur_mode == GAME_CONTROLLER)
				{
					Behavior::GetInstance()->set_normal_state();
					Walking::GetInstance()->A_MOVE_AIM_ON = false;
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					X_Moved = 0;
				}
			}
		}
		m_NoBallCount++;
	}
}

void BallFollower::ProcessCariBola(Point2D ball_pos, bool *IsOneCycle, Vector2<int> Jarak)
{
	double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	if((ball_pos.X  == -1.0 || ball_pos.Y == -1.0) && flagNemuBola == false)
	{
		if(*IsOneCycle == true)
		{
			X_Moved = 0;
			Y_Moved = 0;

			if(Walking::GetInstance()->IsRunning() == false)
			{
				Walking::GetInstance()->Start();
			}
			if(abs(A_Moved) < 120) //Awalnya dari pake ketu 180
			{
				m_GoalFBStep = 0;
				m_GoalRLStep = 0;
				m_GoalRLTurn = 30;
			}
			else
			{
				*IsOneCycle = false;
				A_Moved = 0;
			}
		}
		else
		{
			m_GoalFBStep = 0;
			m_GoalRLStep = 0;
			m_GoalRLTurn = 0;
			A_Moved = 0;
			if(Walking::GetInstance()->IsRunning() == true)
			{
				Walking::GetInstance()->Stop();
			}
		}
	}
	else
	{
		flagNemuBola = true;
		panLastSeenBall = pan + ball_pos.X;
		tiltLastSeenBall = Jarak.x;
		if(Walking::GetInstance()->IsRunning() == false)
		{
			Walking::GetInstance()->Start();
		}
		if(fabs(A_Moved) < fabs(panLastSeenBall)+5 && flagUdahMuter == false) //Muter sebesar PanLastSeenBall selama A/Moved  kurang dari pan last seen Ball
		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = panLastSeenBall;
			m_GoalRLStep = 0;
		}
		else
		{
			flagUdahMuter = true;
			if(X_Moved < 100) // 100
			{
				m_GoalFBStep = 15; // awalnya dari pak ketu 20 
				m_GoalRLTurn = 0;
				m_GoalRLStep = 0;
				panLastSeenBall = 0;
			}
			else
			{
				flagUdahMuter = false;
				flagNemuBola = false;
				Behavior::GetInstance()->flagLagiSwitch = true;
				//Biar sewaktu switch resolusi diam dulu
				if(Walking::GetInstance()->IsRunning() == true)
				{
					Walking::GetInstance()->Stop();
				}
				Behavior::GetInstance()->Bingung = false;
				Behavior::GetInstance()->set_switching_transition();
			}
		}
	}
	X_Moved += Walking::GetInstance()->Get_X_Moved();
	A_Moved += Walking::GetInstance()->Get_A_Moved() * 180 /PI;
	Y_Moved += Walking::GetInstance()->Get_Y_Moved();
	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep >= m_GoalFBStep)
		m_FBStep -= m_UnitFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;

	if(m_RLTurn > m_FollowMaxRLTurn)
		m_RLTurn = m_FollowMaxRLTurn;
	else if(m_RLTurn < -m_FollowMaxRLTurn)
		m_RLTurn = -m_FollowMaxRLTurn;

	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
 	Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLStep;
}

void BallFollower::ProcessTurnAroundBall(Point2D ball_pos, double PanPost, int CompassError, int PostPosition)
{
	//sebelumnya variabel A-Moved dan CountMaxTurnBall telah di nolkan di state AproachingBall2
	if(Action::GetInstance()->IsRunning() == 1)
		Walking::GetInstance()->Stop();
	else
		Walking::GetInstance()->Start();

	double TurningAngle = 500;//mula2 dibuah besar agar ga langsung pindah state
  	double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
  	double pan_range = Head::GetInstance()->GetLeftLimitAngle();
  	double pan_percent = pan / pan_range;
	if (pan_percent < 0)
		pan_percent = -pan_percent;

	double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
  	double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
  	double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
  	double tilt_percent = (tilt - tilt_min) / tilt_range;
  	if(tilt_percent < 0)
  		tilt_percent = -tilt_percent;
	Walking::GetInstance()->A_MOVE_AIM_ON = false;

	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)//kalo ga ketemu
	{
		if(LostCounter > MaxLostCounter)
		{
			ProcessSearchingBall();
		}
		else
		{
			A_Moved = 0;
			X_Moved = 0;
			LostCounter++;
		}
	}
	else
	{
		LostCounter= 0;
		panLastSeenBall = pan + ball_pos.X;//ini di assign setelah diassign di aproachingball1
		tiltLastSeenBall = tilt + ball_pos.Y;
		if(abs(panLastSeenBall) > 90) // Ini parameter apa?
			panLastSeenBall *= 180 /abs(panLastSeenBall); //ubah ke derajat?, atau untuk apa

		MaxBallTurn = PanPost;// + A_Moved * 3/4; //sudut maksimum untuk muter
  		TurningAngle = 0.1*MaxBallTurn + (CompassError * 360 /255);//sudut putar - (sudut saat ini - sudut setpoint)
  		printf("turning angle = %lf\n", TurningAngle);
		printf("A_Moved = %lf\n", A_Moved);

		if(tilt >= tilt_min + 70) //Bola masih jauh di depan //50
		{
			printf("\t\t\tBOLA JAUH BANGET_ATAS, MAJU 20 DAN MUTER\n");
			m_GoalFBStep = 20; //maju
			m_GoalRLTurn = -m_FollowMaxRLTurn * pan_percent; //muter
			m_GoalRLStep = 0;
		}
		else if(tilt >= tilt_min + 40) //kalo bolanya jauh//DIUBAH tambahin else, awalnya if doang 30-50
		{
			if(ball_pos.Y > m_KickTopAngle + 10)//kalo posisi Y bola jauh, ditambah tiltnya jauh
			{
				printf("\t\t\tBOLA JAUH AJA_BAWAH MAJU 20 DAN MUTER\n");
				m_GoalFBStep = 20;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				m_GoalRLStep = 0;
			}
			else	//tiltnya jauh, tapi posisinya di kamera lumayan di bawah, ga ganti state
			{
				printf("\t\t\tBOLA JAUH_ATAS, MAJU 10 DAN MUTER\n");
				m_GoalFBStep = 10;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				m_GoalRLStep = 0;
			}
		}
		else if(tilt > tilt_min + 20) //DIUBAH//kalo bolanya berada agak ke depan 20-30
		{
			if(ball_pos.Y < m_KickTopAngle + 10)
			{
				printf("\t\t\tBOLA JAUH_BAWAH, MAJU 10 DAN MUTER\n");
				m_GoalFBStep = 10;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				m_GoalRLStep = 0;
			}
			else
			{
				printf("\t\t\tBOLA JAUH_ATAS, MAJU 5 DAN MUTER\n");
				m_GoalFBStep = 5;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				m_GoalRLStep = 0;
			}
		}
		else //kalo bolanya udah deket
		{
			m_NoBallCount = 0;
			//################### PENENTUAN BESARAN UNTUK TURNING BALL (MAXBALLTURN) #########################
			//================= EOF BOLA DEKET========================================
			printf("MaxBallTurn = %lf \n", MaxBallTurn);
			printf("Pan = %lf \n", pan);
			//################## proses turning ball ############################ 
			if(abs(pan) <= 20) //kalo bolanya berada lurus di depan
			{
				if(abs(TurningAngle) < 20)//sudut putarnya udah kecil
				{
					if(Walking::GetInstance()->IsRunning() == 1)
						Walking::GetInstance()->Stop();
					else
						Walking::GetInstance()->Start();
					usleep(8000);
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					Action::GetInstance()->Start(7);
					usleep(8000);
				}
				else if(TurningAngle < 0)
				{
					Walking::GetInstance()->A_MOVE_AIM_ON = false;
					m_GoalFBStep = 5;//ubah upi -2 //-(pan_percent * 30);
					if(Walking::GetInstance()->IsRunning() == 1)
						Walking::GetInstance()->Stop();
					else
						Walking::GetInstance()->Start();
					usleep(8000);
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					Action::GetInstance()->Start(7);
					usleep(8000);
					m_GoalRLTurn = 20; //awal 15, ubah upi
				}
				else if(TurningAngle > 0)	
				{
					m_GoalFBStep = 5; //ubah upi -2 //-(pan_percent * 30);
					m_GoalRLStep = 10;
					m_GoalRLTurn = -10; //awal -15, ubah upi
					Walking::GetInstance()->A_MOVE_AIM_ON = false;
				}
				else
				{
					m_GoalFBStep = 0.0;
					m_GoalRLStep = 0.0;
					m_GoalRLTurn = 0.0;
					Walking::GetInstance()->A_MOVE_AIM_ON = false;
				}
			}
			else if((abs(pan) > 20)) //kalo bolanya ada di samping
			{
				if(abs(pan) > 45) //kalo bola disamping banget, maka mundur//DIUBAH 210414
				{
					m_GoalFBStep = 0; //ubah upi -2 //awalnya -5
					if(pan < 0) { //diubah 210414
						if(Walking::GetInstance()->IsRunning() == 1)
							Walking::GetInstance()->Stop();
						else
							Walking::GetInstance()->Start();
						usleep(8000);
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
						Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Action::GetInstance()->Start(7);
						usleep(8000);
					} else
						m_GoalRLTurn = 0; //it was 20
					m_GoalRLStep = 20;
				}
				else
				{
					m_GoalFBStep = 0; //ubah upi -5
					if (pan < 0){
						if(Walking::GetInstance()->IsRunning() == 1)
							Walking::GetInstance()->Stop();
						else
							Walking::GetInstance()->Start();
						usleep(8000);
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
						Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Action::GetInstance()->Start(7);
						usleep(8000);
					} else
						m_GoalRLStep = 0; //it was 20
					m_GoalRLTurn = 20;
				}
			}
			//=====================================================================

			//################# debugging #################
			printf("pan_percent %lf \n", pan_percent);
			printf("Error = %d \n", CompassError);
			printf("FBStep %lf \n", m_GoalFBStep);
			//==============================================
	
			//################## pergantian state ###################			
			if(abs(TurningAngle) < 10)//3 derajat sebagai error untuk kelurusan
			{
				Behavior::GetInstance()->set_kick_state();
				CountMaxTurnBall = 0;
				MaxBallTurn = 0;
				X_Moved = 0;
				Y_Moved = 0;
				A_Moved = 0;
				KickBall = 0;
				KickDir = 0;

				if(PostPosition == 0 && PostEqualZero == true) // kalo ga nemu gawang, maka setelah turning ball akan ngecek gawang lagi
				{
					Behavior::GetInstance()->set_check_post_position();
					BallDistance = 0;
					PostEqualZero = false;
				}	
				return;
			}
		}
		
		X_Moved += Walking::GetInstance()->Get_X_Moved();
		A_Moved += Walking::GetInstance()->Get_A_Moved() * 180 /PI;
		Y_Moved += Walking::GetInstance()->Get_Y_Moved();

		if(m_FBStep < m_GoalFBStep)
			m_FBStep += m_UnitFBStep;
		else if(m_FBStep >= m_GoalFBStep)
			m_FBStep -= m_UnitFBStep;
		Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

		if(m_RLTurn > 0 && m_GoalRLTurn < 0)
			m_RLTurn = 0;
		else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
			m_RLTurn = 0;

		if(m_RLTurn < m_GoalRLTurn)
			m_RLTurn += m_UnitRLTurn;
		else if(m_RLTurn > m_GoalRLTurn)
			m_RLTurn -= m_UnitRLTurn;

		if(m_RLTurn > m_FollowMaxRLTurn)
			m_RLTurn = m_FollowMaxRLTurn;
		else if(m_RLTurn < -m_FollowMaxRLTurn)
			m_RLTurn = -m_FollowMaxRLTurn;

		Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
				
		Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_GoalRLStep;
	}
	//============= EOF BALL FOUND ==========================
}

void BallFollower::ProcessDefense(Point2D ball_pos, Vector2<int> ppfBall)//DIUBAH
{
	printf("DEFENSE STATE \n");
	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)//asumsi teman masih berkutat dengan bola
	{
		if(LostCounter > MaxLostCounter)
		{
			Walking::GetInstance()->Start();
			ProcessSearchingBall();
		}
		else
		{
			LostCounter++;
			X_Moved = 0;
			A_Moved = 0;
		}
	}
	else
	{
		LostCounter = 0;
		CountMaxTurnBall = 0;
		MaxBallTurn = 0;
		//biar pas searchingball, langsung masuk ke yang cari bola.
		KickDir = 2;
 		m_NoBallCount = m_NoBallMaxCount + 25;
		countHeadSearchingCycle = 0;
		turn = false;
		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;
		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		tiltLastSeenBall = tilt + ball_pos.Y;
		panLastSeenBall = pan + ball_pos.X;
		if(abs(panLastSeenBall) > 90)
			panLastSeenBall *= 180 /abs(panLastSeenBall);

		if(abs(pan) > 35)//kalo range pan terlalu gedhe, asumsi bolanya deket ato jauh
		{
			if(Walking::GetInstance()->IsRunning() ==  0)
				Walking::GetInstance()->Start();
			m_GoalFBStep = 0;
			m_GoalRLTurn = (m_FollowMaxRLTurn * pan_percent);
		}
		else if(ppfBall.x < 1000)//kalo jaraknya udah deket sama bola, mundur dikit lah...
		{
			if(Walking::GetInstance()->IsRunning() ==  0)
				Walking::GetInstance()->Start();
			panLastSeenBall = pan;
			tiltLastSeenBall = tilt;
			BallDistance	= 8;		
			m_GoalFBStep = -10;
			m_GoalRLTurn = 0;
		}
		else if (abs(pan) > 10)//asumsi jarak udah lebih dari 1000, trus mgepasin ke bola
		{
			if(Walking::GetInstance()->IsRunning() ==  0)
				Walking::GetInstance()->Start();
			m_GoalFBStep = 0;
			m_GoalRLTurn = (m_FollowMaxRLTurn * pan_percent);
		}
		else if(ppfBall.x >= 1500) //Bola masih cukup jauh atau bola berada di kiri atau di kanan
		{
			if(Walking::GetInstance()->IsRunning() ==  0)
				Walking::GetInstance()->Start();
			BallDistance = 1;
			m_KickBallCount = 0;
			m_CountBallFound = 0;
			m_GoalFBStep = m_FollowMaxFBStep * tilt_percent - (pan_percent * 40.0);
			if(m_GoalFBStep < m_FollowMinFBStep)
				m_GoalFBStep = m_FollowMinFBStep;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			A_Moved = 0;
		}
		else
		{
			panLastSeenBall = pan;
			tiltLastSeenBall = tilt;
			BallDistance	= 8;		
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			if(Walking::GetInstance()->IsRunning() == 1)
				Walking::GetInstance()->Stop();
		}
	}

	Walking::GetInstance()->A_MOVE_AIM_ON = false;

	//process untuk memberikan parameter jalan
	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep >= m_GoalFBStep)
		m_FBStep -= m_UnitFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;


	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;

	if(m_RLTurn > m_FollowMaxRLTurn)
		m_RLTurn = m_FollowMaxRLTurn;
	else if(m_RLTurn < -m_FollowMaxRLTurn)
		m_RLTurn = -m_FollowMaxRLTurn;

	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AIM_ON = false;

	X_Moved += Walking::GetInstance()->Get_X_Moved();
	Y_Moved += Walking::GetInstance()->Get_Y_Moved();
	A_Moved += Walking::GetInstance()->Get_A_Moved() * 180 / PI;
}

void BallFollower::ProcessAproachingBall(Point2D ball_pos, Vector2<int> ppfBall, bool EdgeField)
{
	Walking::GetInstance()->Start();
	EdgeOfField = EdgeField;

	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)//ALGORITMA MENCARI BOLA
 	{
		if(LostCounter > MaxLostCounter)
		{
			counterGetBall = 0;
			ProcessSearchingBall();
 		}
		else
		{
			A_Moved = 0;
			X_Moved = 0;
			LostCounter++;
		}
	}
 	else //BOLA SUDAH KETEMU
 	{
		PostEqualZero = true;
		KickDir = 2;
		LostCounter = 0;
 		m_NoBallCount = m_NoBallMaxCount + 25;
		countHeadSearchingCycle = 0;
		turn = false;

		counterGetBall++;
		counterGetBallFound = counterGetBall;
	    double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	    double pan_range = Head::GetInstance()->GetLeftLimitAngle();
	    double pan_percent = pan / pan_range;

	    double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	    double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
	    double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
	    double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
 	   		tilt_percent = -tilt_percent;

		printf("tilt = %lf \n", tilt);
		printf("pan = %lf \n", pan);

		tiltLastSeenBall = tilt + ball_pos.Y;
		panLastSeenBall = pan + ball_pos.X;
		
		if(abs(panLastSeenBall) > 90)
		{
			panLastSeenBall = panLastSeenBall * 180 /abs(panLastSeenBall);
			//tiltLastSeenBall = 40;		
		}
		
		if(tilt >= 30) //Bola masih cukup jauh
		{
			printf("\t\t\t\t\t\t\t\tBOLA JAUH\n");
			m_KickBallCount = 0;
			m_CountBallFound = 0;
			if(abs(pan) > 60)//bola terlalu di samping
			{
				m_GoalFBStep = 0;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			}
			else
			{
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			}

			A_Moved = 0;
		}
		else if (abs(pan) > 15)//bola udah deket, tapi masih di samping
    		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = (m_FollowMaxRLTurn * pan_percent);
			printf("\t\t\t\t\t\t\t\tBOLA DI SAMPING\n %f\n, %f, %f\n", m_FollowMaxRLTurn, pan_percent, m_GoalRLTurn);

		}
		else		//bola udah deket dan lurus
		{
			printf("\t\t\t\t\t\t\t\tBOLA DI DEPAN\n");
			panLastSeenBall = pan;//menyimpan nilai pan terakhir dapet bola
			tiltLastSeenBall = tilt;
			
			if(m_CountBallFound > m_MaxCountBallFound && ball_pos.Y < 5)
			{
				BallDistance = ppfBall.x - 700;
				if(BallDistance <= 0)
					BallDistance = 0;

				MaxBallTurn = 0;
				CountMaxTurnBall = 0;
				countHeadSearchingCycle = 0;
				m_CountBallFound = 0;

				
				EdgeOfField = false;

				m_NoBallCount = 0;				

				//ngenolin semua parameter jalan
				X_Moved += Walking::GetInstance()->Get_X_Moved();
				Y_Moved += Walking::GetInstance()->Get_Y_Moved();
				A_Moved += Walking::GetInstance()->Get_A_Moved();
				X_Moved = 0;
				Y_Moved = 0;
				A_Moved = 0;
				//===========================
				Behavior::GetInstance()->set_check_post_position();

				return;
			}
			else
			{
					m_CountBallFound++;
					m_GoalFBStep = 10;
					m_GoalRLTurn = 0;
			}
		}
	}

	Walking::GetInstance()->A_MOVE_AIM_ON = false;

	printf("\t\t\t\tFBStep = %lf \n", m_FBStep);
	printf("\t\t\t\tRLStep = %lf \n", m_RLStep);

	//process untuk memberikan parameter jalan
	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep >= m_GoalFBStep)
		m_FBStep -= m_UnitFBStep;

	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;

	if(m_RLTurn > m_FollowMaxRLTurn)
		m_RLTurn = m_FollowMaxRLTurn;
	else if(m_RLTurn < -m_FollowMaxRLTurn)
		m_RLTurn = -m_FollowMaxRLTurn;


	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;

	X_Moved += Walking::GetInstance()->Get_X_Moved();
	Y_Moved += Walking::GetInstance()->Get_Y_Moved();
	A_Moved += Walking::GetInstance()->Get_A_Moved() * 180 / PI;

/*
	if(MotionStatus::FALLEN == FORWARD || MotionStatus::FALLEN == BACKWARD)
	{
		m_FBStep = 0;
		m_RLStep = 0;
		m_RLTurn = 0;
	}
*/

}


void BallFollower::ProcessLocalizeGoalPost(Point2D ball_pos)
{
	double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
	double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
	double tilt_percent = (tilt - tilt_min) / tilt_range;

	if(tilt_percent < 0)
		tilt_percent = -tilt_percent;
	printf("X_Moved = %lf \n ", X_Moved);

	if(X_Moved >= BallDistance)//jika udah deket dengan bola
	{
		Walking::GetInstance()->Stop();
		m_GoalFBStep = 0;
		m_GoalRLStep = 0;
		m_GoalRLTurn = 0;
	}
	else if(abs(A_Moved) < abs(panLastSeenBall))//jika tadi putarannya belum bener
	{
		m_GoalFBStep = 10;
		m_GoalRLTurn = panLastSeenBall/4;//DIUBAH lagi
		m_GoalRLStep = 0;
	}
	else
	{
		m_GoalFBStep = 10;
		m_GoalRLStep = 0;
		m_GoalRLTurn = 0;
	}

	if(m_FBStep > m_GoalFBStep)
		m_FBStep -= m_UnitFBStep;
	else if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_GoalFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
	X_Moved += Walking::GetInstance()->Get_X_Moved();

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
	A_Moved += Walking::GetInstance()->Get_A_Moved() * 180 /PI;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_GoalRLStep;
}


void BallFollower::ProcessAproachingBall2(Point2D ball_pos, double panpost, double CompassError, double CompassPost)
{
	Walking::GetInstance()->Start();
	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
	{
		printf("\t\t\t\t\t\t\t\t\t\tBOLA HILANG\n");
		// habis waituntilready <- false, kalo bola ilang, maka akan searching dulu
		if(LostCounter > MaxLostCounter)
		{
			CountMaxTurnBall = 0;
			ProcessSearchingBall();
		}
		else
		{
			LostCounter++;
			X_Moved = 0;
			A_Moved = 0;
		}
	}
	else //BOLA SUDAH KETEMU
	{
		m_NoBallCount = 0;
		LostCounter = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;
		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		double temp = panpost + CompassPost;

		printf("Tilt=%f \t Pan=%f \t PanPost=%f\n",tilt,pan,panpost);
		printf("Compass Post = %f\n",CompassPost);
		printf("Compass Error = %f\n",CompassError);

		if(fabs(CompassError + pan) >= fabs(temp) + 5) //robot belum lurus ke arah bola-gawang
		{
			printf("BOLA BELUM LURUS - ");
			if( tilt > 15) //bola masih jauh di depan //Al 0
			{
				printf("JAUH - ");
				//m_GoalFBStep = 4;
				if(fabs(pan) > 40) //bola ada jauh di samping
				{
					printf("DI SAMPING BANGET\n");
					m_GoalRLTurn = pan*0.5; //putar max
					m_GoalFBStep = 4;
				}
				else
				{
					printf("DI SAMPING SAJA\n");
					m_GoalRLTurn = fabs(CompassError + pan) - fabs(temp); //putar pelan karena bola sudut robot-bola-gawang sudah kecil
					m_GoalFBStep = 4;
				}
			}
			else //bola sudah dekat
			{
				printf("DEKAT - ");
				m_GoalRLTurn = m_FollowMaxRLTurn*pan_percent; // meluruskan lagi secara perlahan
				m_GoalFBStep = 5; //maju pelan //Al 8
				//printf("DRIBBLE BALL\n");


				//siap-siap masuk state dribble
				Behavior::GetInstance()->set_dribble_ball();


				PostDist = 0; // inisialisasi odometry di ProcessToKick();
				X_Moved = 0;
				flagDribble = 0;
				return;
			}
		}
		else //bola sudah lurus
		{
			printf("BOLA SUDAH LURUS - ");
			if (flagDribble == 0)
			{
				printf("00000000000000000000\n");
				pan_now = pan;
				flagDribble = 1;
			}
			//printf("APROACHING BALL 2\n");
			if ((pan <= (CompassError + pan_now)+5) && (pan >= (CompassError + pan_now)-5)) //sudut bola-robot kecil
			{
				printf("11111111111111111111\n");
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				m_GoalFBStep = 0; //meluruskan bola
			}
			else if (tilt > 5)  //bola jauh (?)
			{
				printf("22222222222222222222\n");
				m_GoalRLTurn = 0;
				m_GoalFBStep = 5;
			}
			else //masuk dribble
			{
				printf("33333333333333333333\n");

				Behavior::GetInstance()->set_dribble_ball();
				PostDist = 0; // inisialisasi odometry di ProcessToKick();
				X_Moved = 0;
				flagDribble = 0;
				return;
			}
		}
/*
		if (tilt >= 5) // batas putas lingkaran gauss
	        {
	        	m_GoalFBStep = 12;
	        	m_GoalRLTurn = 0;
	        	m_GoalRLStep = 0;
	        	printf("bola jauh\n");
		}
        else if (tilt < 5)
    	{
			printf("bola dekat\n");
    		if (fabs (pan) > 5)
    		{
    			m_GoalFBStep = 0;
    			m_GoalRLTurn = 0.6*pan; //m_FollowMaxRLTurn*pan_percent;
    			m_GoalRLStep = 0;
    			printf("bola disamping\n");
			}
    		else
    		{ 
			printf("DRIBBLE BALL\n");
			Behavior::GetInstance()->set_dribble_ball();
			PostDist = 0;//inisialisasi odometry di ProcessToKick();
			X_Moved = 0;
			flagDribble = 0;
			return;
        	}
    	}
*/		
		printf("FBStep = %lf\n", m_FBStep);
		printf("RLStep = %lf\n", m_RLStep);

		//process untuk memberikan parameter jalan
		if(m_FBStep < m_GoalFBStep)
			m_FBStep += m_UnitFBStep;
		else if(m_FBStep >= m_GoalFBStep)
			m_FBStep -= m_UnitFBStep;

		Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

		if(m_RLTurn > 0 && m_GoalRLTurn < 0)
			m_RLTurn = 0;
		else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
			m_RLTurn = 0;

		if(m_RLTurn < m_GoalRLTurn)
			m_RLTurn += m_UnitRLTurn;
		else if(m_RLTurn > m_GoalRLTurn)
			m_RLTurn -= m_UnitRLTurn;

		if(m_RLTurn > m_FollowMaxRLTurn)
			m_RLTurn = m_FollowMaxRLTurn;
		else if(m_RLTurn < -m_FollowMaxRLTurn)
			m_RLTurn = -m_FollowMaxRLTurn;
		
		Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;


		if(m_RLStep > 0 && m_GoalRLStep < 0)
                        m_RLStep = 0;
                else if(m_RLStep < 0 && m_GoalRLStep > 0)
                        m_RLStep = 0;

                if(m_RLStep < m_GoalRLStep)
                        m_RLStep += m_UnitRLStep;
                else if(m_RLStep > m_GoalRLStep)
                        m_RLStep -= m_UnitRLStep;

                if(m_RLStep > m_FollowMaxRLStep)
                        m_RLStep = m_FollowMaxRLStep;
                else if(m_RLStep < -m_FollowMaxRLStep)
                        m_RLStep = -m_FollowMaxRLStep;		
		Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLStep;

		A_Moved += Walking::GetInstance()->Get_A_Moved() * 180/PI;
	}
}

void BallFollower::Process(Point2D ball_pos)
{
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 40;

	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
	{
		FirstTilt = false;
		if(m_NoBallCount > m_NoBallMaxCount && countHeadSearchingCycle > 2)
		{
			m_GoalFBStep = 0;

			if(Head::GetInstance()->CheckLastPanAngle() == RIGHT )
			{
				m_GoalRLTurn = -30;
			}
                	else if(Head::GetInstance()->CheckLastPanAngle() == LEFT )
			{
				m_GoalRLTurn = 30;
			}
		}
		else
		{
			m_NoBallCount++;
			m_GoalFBStep = 20;
		}
	}
	else
	{
		m_NoBallCount = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(FirstTilt == false) // mengassign tilt pertama saat ketemu bola...
		{
		     TiltAngle = tilt;
		     FirstTilt = true;
		}
		if(tilt <= TiltAngle*2/3)
		{
			Behavior::GetInstance()->set_check_post_position();
		}
		if(pan > -15 && pan < 15) // JIKA BOLA BERADA DI TENGAH
		{
			// artinya bola udah deket dengan kaki
			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE + 10))
			{
				if(ball_pos.Y < m_KickTopAngle + 10)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount) // BOLA SIAP TENDANG, CARI GAWANG
					{
						m_FBStep = 0;
						m_RLTurn = 0;
						Behavior::GetInstance()->set_check_post_position();

						Head::GetInstance()->MoveByAngle(0, 40);
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					}
					else
					{
						m_KickBallCount++;
					}
				}
				else
				{
					m_KickBallCount = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
				}
			}
			else
			{
				m_KickBallCount = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			}
		}
		else //KALO BOLA MASIH SANGAT JAUH
		{
		    if(pan > Head::GetInstance()->GetLeftLimitAngle()-10 || pan < Head::GetInstance()->GetRightLimitAngle()+10)
		    { //BOLA BERADA DI KANAN ATO KIRI, SEHINGGA PERLU MUTER DAHULU
			m_GoalFBStep = 0;
		    }
		    else
		    { //BOLA UDAH GA TERLALU JAUH UNTUK PERLU MUTER
			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE + 50))
			//KALO UDAH DEKET BOLA
			{
				m_GoalFBStep = 0;
			}
			else
			{
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
                                if(m_GoalFBStep < m_FollowMinFBStep)
                                    m_GoalFBStep = m_FollowMinFBStep;
			}
		    }
			m_KickBallCount = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
		}
	}

	//process untuk memberikan parameter jalan
	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep > m_GoalFBStep)
		m_FBStep = m_GoalFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

}


void BallFollower::ProcessToKick(Point2D ball_pos, double PanPost, double CompassError, double CompassPost)
{
	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)        // CEK JIKA BOLA TIDAK KETEMU
	{
		printf("\t\t\t\t\t\t\t\t\t\tBOLA HILANG\n");
		CountMaxTurnBall = 0;
		if(m_NoBallCount < 5)
		{
			X_Moved = Walking::GetInstance()->Get_X_Moved();
			X_Moved = 0;
		}
		else
		{
			Walking::GetInstance()->X_MOVE_AMPLITUDE = -10; //bahaya nilai x terlalu kecil
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;

			if(panLastSeenBall < -10)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 10;
			else if(panLastSeenBall > 10)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = -10;

			X_Moved = Walking::GetInstance()->Get_X_Moved();
			if(X_Moved > -500)
				Behavior::GetInstance()->set_normal_state();
		}
		m_NoBallCount++;
	}
	else	// CEK JIKA BOLA KETEMU
	{
		printf("\t\t\t\t\t\t\t\t\t\tBOLA ADA\n\n");

		printf("\t\t\t\tPan Post = %f\n",PanPost);
		//printf("\t\t\t\tPan = %f\n",pan);
		printf("\t\t\t\tCompass Post = %f\n",CompassPost);
		printf("\t\t\t\tCompass Error = %f\n\n",CompassError);

		m_NoBallCount = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;

/*
		if(PostDist >= 5000) // jarak terlalu jauh, check post lagi biar aman
		{
			printf("\t\t\t\t\t\t\t\t\t\tBOLA JAUH - CHECK POST LAGI\n");
			Behavior::GetInstance()->set_check_post_position();
			PostDist = 0;
		}
		printf("\t\t\t\t\t\t\t\t\t\t TILT : %lf", tilt);
		printf("\t\t\t\t\t\t\t\t\t\t PAN : %lf", pan);
		
		if((tilt >= -20) && (abs(pan) <= 15)) // BOLA MASIH JAUH
		{
			printf("\t\t\t\t\t\t\t\t\t\tMMAAAAAJJUUUUUUUUUUUUUUUUU\n");
			m_GoalFBStep = 10; //tadinya 10, edit yoga
			m_GoalRLTurn = 0;
		} 
		else if (tilt < -20) //BOLA SUDAH DEKAT
		{
			if(abs(pan) > 15) // JIKA DISAMPING 
			{	 
				printf("\t\t\t\t\t\t\t\t\t\tPPPUUUTTTAAAAAAAAAARRRRRRRR\n");
				m_GoalFBStep = 0;
				m_GoalRLTurn = pan*0.5;
			}
			else if (abs(pan) <= 15)
			{ 
				if ((abs(CompassError)) > (abs(CompassPost + PanPost) + 5)) 
				{
					printf("\t\t\t\t\t\t\t\t\t\tLLLLUUURRRRUUUSSSKAAAAAAANNN\n");
					m_GoalFBStep = 1; //tadinya 4
					m_GoalRLTurn =   ((CompassPost + PanPost) - (CompassError));
					if(m_GoalRLTurn > 10)
						m_GoalRLTurn = 10;
				
				}
				else
				{
					printf("\t\t\t\t\t\t\t\t\t\tSSSSSSIIIKKKKKKKKKKAAAAAATTTTTTTTTT\n");
					m_GoalFBStep = 15; //warning
					m_RLTurn = 0;
					m_GoalRLTurn = 0;
				}
			}
		}
*/
		printf("tilt = %f \t pan = %f\n",tilt,pan);
		if(PostDist >= 2000)
		{
			Behavior::GetInstance()->set_check_post_position();
			PostDist = 0;
		}

		if(tilt > -15 && abs(pan) <= 10)
		{
			printf("LURUS BOLA JAUH\n");
			m_GoalFBStep = 18;
			m_GoalRLTurn = 0;
		}
		else if (tilt < 15) 
		{
			if(abs(pan) > 10)
			{
				m_GoalFBStep = 0;
				m_GoalRLTurn = pan*0.5;
				printf("BOLA DISAMPING\n");
			}
			else 
			{
				if(CompassError >= 10 && CompassError <= 90)
				{
					m_GoalFBStep = 8;
					//m_RLTurn = 0;
					m_GoalRLTurn = -12;
					printf("LURUSKAN\n");
				}
				else if(CompassError < -10 && CompassError >= -90)
				{
					m_GoalFBStep = 0;
					//m_RLTurn = 0;
					m_GoalRLStep = 18;
					printf("LURUSKAN\n");
				}
				else if(CompassError > 90)
				{
					m_GoalFBStep = 0;
					//m_RLTurn = 0;
					m_GoalRLTurn = 0;
					m_GoalRLStep = 20;
					printf("LURUSKAN KE SAMPING\n");
				}
				else if(CompassError  < -90)
				{
					m_GoalFBStep = 0;
					//m_RLTurn = 0;
					m_GoalRLTurn = 0;
					m_GoalRLStep = -20;
					printf("LURUSKAN\n");
				}
				else
				{	
					m_GoalRLStep = 0;
					m_GoalRLTurn = 0;
					m_GoalFBStep = 18;
					printf("MAJUUUUUUU SIKAATTTTT BROOOOO\n");
				}
			}
		}

		PostDist += Walking::GetInstance()->Get_X_Moved();

		Walking::GetInstance()->A_MOVE_AIM_ON = false;

		//printf("\t\t\t\tFBStep = %lf\n", m_FBStep);
		//printf("\t\t\t\tRLStep = %lf\n", m_RLStep);

		//process untuk memberikan parameter jalan
		if(m_FBStep < m_GoalFBStep)
			m_FBStep += m_UnitFBStep;
		else if(m_FBStep >= m_GoalFBStep)
			m_FBStep -= m_UnitFBStep;
		Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

		if(m_RLTurn > 0 && m_GoalRLTurn < 0)
			m_RLTurn = 0;
		else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
			m_RLTurn = 0;

		if(m_RLTurn < m_GoalRLTurn)
			m_RLTurn += m_UnitRLTurn;
		else if(m_RLTurn > m_GoalRLTurn)
			m_RLTurn -= m_UnitRLTurn;

		if(m_RLTurn > m_FollowMaxRLTurn)
			m_RLTurn = m_FollowMaxRLTurn;
		else if(m_RLTurn < -m_FollowMaxRLTurn)
			m_RLTurn = -m_FollowMaxRLTurn;
		Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

		if (m_RLStep < m_GoalRLStep)
			m_RLStep += m_UnitRLStep;
		else if (m_RLStep > m_GoalRLStep)
		 	m_RLStep -= m_UnitRLStep;
		Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLStep;
	}
}

void BallFollower::ProcessKick(Point2D ball_pos)
{
	if(Action::GetInstance()->IsRunning() == 1)
		Walking::GetInstance()->Stop();
	else
		Walking::GetInstance()->Start();

	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0) // CEK JIKA BOLA TIDAK KETEMU
	{
		if(LostCounter > MaxLostCounter)
		{
			Walking::GetInstance()->Start();
                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			ProcessSearchingBall();
		}
		else
			LostCounter++;	
	}
	else // CEK JIKA BOLA KETEMU
	{
		m_NoBallCount = 0;
		LostCounter = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;
		tiltLastSeenBall = tilt + ball_pos.Y;
		panLastSeenBall = pan + ball_pos.X;
		if(abs(panLastSeenBall) > 90)
			panLastSeenBall *= 180 /abs(panLastSeenBall);

		//ball_pos.Y udah dalam bentuk angle. untuk pixel Y < center camera, maka ball_pos.Y-nya adalah positif
		
		if(panLastSeenBall >= m_KickRightAngle && panLastSeenBall <= m_KickLeftAngle && tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 30) 
		{ // CEK JIKA BOLA BERADA LURUS DI DEPAN
			if(KickDir == 0)
			{
				if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 5) //bola di deket kaki //tadinya + 15
				{
					if(ball_pos.Y < m_KickTopAngle) //tadinya -tilt
					{
						if(panLastSeenBall < 0)
						{
							KickBall = -1;
						}
						else
						{
							KickBall = 1;
						}
					}
					else
					{
						KickBall = 0;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
						m_KickBallCount=0;
					}
				}
				else
				{
					KickBall = 0;
					Walking::GetInstance()->Start();
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 10;
					Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					m_KickBallCount = 0;
				}
			}
			else if(KickDir == -1)
			{
				printf("Kick Direction equal minus 1\n");
				if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 10) //bola di deket kaki //tadinya + 15
				{
					if(ball_pos.Y < 5) //tadinya -tilt
					{	
						if(pan > 18)
						{        
							KickBall = 0;
							m_KickBallCount = 0;
							Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
							Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;//0
						}
						else if(pan < 10)
						{
							KickBall = 0;
							m_KickBallCount = 0;
							Walking::GetInstance()->Stop();
							usleep(8000);
							Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
							Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
							Action::GetInstance()->Start(7);
							usleep(8000);
						}
						else
						{
							flag = 1;
							if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 10)
							{
								if(ball_pos.Y < 5)
								{
									KickBall = 2;
								}
								else
								{
									KickBall = 0;
									Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
									m_KickBallCount = 0;
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;
									Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
								}
							}		
							else
							{			
								KickBall = 0;				
								Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
								m_KickBallCount = 0;
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							}
						}
					}
					else
					{
						if(pan > 18)
						{        
							KickBall = 0;
							Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
							m_KickBallCount = 0;
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
						}
						else if(pan < 10)
						{
							KickBall = 0;
							m_KickBallCount = 0;
							Walking::GetInstance()->Stop();
							usleep(8000);
							Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
							Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
							Action::GetInstance()->Start(7);
							usleep(8000);
						}
						else
						{
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
						}
					}		
				}
				else
				{
					if(pan > 18)
					{        
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					}
					else if(pan < 10)
					{
						KickBall = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->Stop();
						usleep(8000);
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
						Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Action::GetInstance()->Start(7);
						usleep(8000);
					}
					else
					{
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					}
				}
			}
			else if(KickDir == 1)
			{
				printf("Kick Direction equal 1\n");
				if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 10) //bola di deket kaki //tadinya + 15
				{
					if(ball_pos.Y < 5) //tadinya -tilt
					{ //flag digunakan biar setelah nendang, ga perlu ngepasin lagi buat nendang berikutnya	
						if(pan < -18)
						{        
							KickBall = 0;
							m_KickBallCount = 0;
							Walking::GetInstance()->Stop();
							usleep(8000);
							Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
							Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
							Action::GetInstance()->Start(7);
							usleep(8000);
						}
						else if(pan > -10)
						{
							KickBall = 0;
							Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
							m_KickBallCount = 0;
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			
						}
						else
						{
							flag = 1;
							if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 10)
							{
								if(ball_pos.Y < 5)
								{
									KickBall = -2;
								}
								else
								{
									KickBall = 0;
									Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
									m_KickBallCount = 0;
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;
									Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
								}
							}		
							else
							{			
								KickBall = 0;				
								Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
								m_KickBallCount = 0;
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							}
						}
					}
					else
					{
						if(pan < -18)
						{        
							KickBall = 0;
							m_KickBallCount = 0;
							Walking::GetInstance()->Stop();
							usleep(8000);
							Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
							Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
							Action::GetInstance()->Start(7);
							usleep(8000);
						}
						else if(pan > -10)
						{
							KickBall = 0;
							Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
							m_KickBallCount = 0;
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			
						}
						else
						{
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
						}
					}		
				}		
				else
				{
					if(pan < -18)
					{        
						KickBall = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->Stop();
						usleep(8000);
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
						Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Action::GetInstance()->Start(7);
						usleep(8000);
					}
					else if(pan > -10)
					{
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE =10;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
		
					}
					else
					{
						KickBall = 0;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						m_KickBallCount = 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					}
				}		
			}
			else if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE + 25) //bola agak ke depan
			{
				KickBall = 0;
				flag = 0;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 10;
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
				Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
		
			}
			else
			{
				KickBall = 0;
				flag = 0;
				Walking::GetInstance()->Start();
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				Behavior::GetInstance()->set_normal_state();
			}
		}
		else if(panLastSeenBall > m_KickLeftAngle && panLastSeenBall <Head::GetInstance()->m_LeftLimit - 15 && tilt < tilt_min+MX28::RATIO_VALUE2ANGLE + 30)
		{
			flag = 0;
			KickBall = 0;
			m_KickBallCount = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = -5;//diubah 210414
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 20;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
		}
		else if(panLastSeenBall<m_KickRightAngle && panLastSeenBall>Head::GetInstance()->m_RightLimit + 15 && tilt < tilt_min+MX28::RATIO_VALUE2ANGLE + 30)
		{
			flag = 0;
			KickBall = 0;
			m_KickBallCount = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = -5;//diubah 210414
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = -20;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 20;
		}
		else
		{
			KickBall = 0;
			flag = 0;
			Walking::GetInstance()->Start();
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			Behavior::GetInstance()->set_normal_state();
		}
	}
}

void BallFollower::ProcessLocalizePenaltyMark(Point2D ball_pos)
{

	if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)        // CEK JIKA BOLA TIDAK KETEMU
	{
		if(m_NoBallCount > m_NoBallMaxCount && countHeadSearchingCycle > 2)
		{
			// can not find a ball
			m_GoalFBStep = 0;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;

			if(Head::GetInstance()->CheckLastPanAngle() == RIGHT )
				m_GoalRLTurn = -30;
			else if(Head::GetInstance()->CheckLastPanAngle() == LEFT )
				m_GoalRLTurn = 30;
		}
		else
		{
			m_NoBallCount++;
		}
	}
	else                                        // CEK JIKA BOLA KETEMU
	{
		m_NoBallCount = 0;

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < 10)//m_KickLeftAngle)                                     // CEK JIKA BOLA BERADA LURUS DI DEPAN
		{
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;

			if(Head::GetInstance()->GetTiltAngle() < 35)
				m_GoalFBStep = -10;
			else
				m_GoalFBStep = 10;

			if(GoalPercept::GetInstance()->Status != GoalPercept::NONE)
			{
				ball_pos.X *= ((double)Camera::WIDTH / Camera::VIEW_H_ANGLE);
				ball_pos.X *= -1;
				Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
				ball_pos.X += center.X;

				if(ball_pos.X - GoalPercept::GetInstance()->Center.X > 5)
				{
					Walking::GetInstance()->Y_MOVE_AMPLITUDE = -25;
				}
				else if(ball_pos.X - GoalPercept::GetInstance()->Center.X < 5)
				{
					Walking::GetInstance()->Y_MOVE_AMPLITUDE = 25;
				}
			}
			else
			{
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = -25;
			}
		}
		else                                                                                    // CEK JIKA BOLA TIDAK BERADA LURUS DI DEPAN
		{
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
		}
	}

	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep > m_GoalFBStep)
		m_FBStep = m_GoalFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

}

void BallFollower::ProcessPenaltyKick(Point2D ball_pos)
{
        double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent = pan / pan_range;
	if (pan_percent < 0)
		pan_percent = -pan_percent;

        double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if(tilt_percent < 0)
               	tilt_percent = -tilt_percent;

	Goback = true;


	if(ball_pos.X == -1 || ball_pos.Y == -1)//bola ga ketemu, maka maju perlahan
	{
		printf("BALL IS NOT DETECTED\n");
		m_GoalFBStep = 13;
		m_GoalRLStep = 0;
		m_GoalRLTurn = 0;
	}
	else	
	{
		if(tilt < 10)   // cek jika bola dekat
                {
			printf("BALL IS PRETTY CLOSE\n");
			printf("tilt = %lf\n",tilt);
			printf("pan = %lf\n",pan);

			if(pan >= -30 && pan <= 30) //bola lurus di depan
			{
				if(tilt < tilt_min + MX28::RATIO_VALUE2ANGLE) //bola di deket kaki //tadinya + 15
				{
					//if(ball_pos.Y < 40) //tadinya -tilt
					if(tilt < -15.0)
					{
						if(pan < 0)
							KickBall = -1;
						else
							KickBall = 1;	
					}			
					//if(pan > 30) {
					//	if(Walking::GetInstance()->IsRunning() == 1)
					//		Walking::GetInstance()->Stop();
					//	else
					//		Walking::GetInstance()->Start();
					//	usleep(8000);
					//	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					//	Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					//	Action::GetInstance()->Start(7);
					//	usleep(8000);
					//}
					else
					{
						KickBall = 0;
						m_GoalFBStep = 5;
						m_GoalRLStep = 0;
						m_GoalRLTurn = 0;
					}
				}				
				else
				{
					KickBall = 0;
					m_GoalFBStep = 5;
					m_GoalRLStep = 0;
					m_GoalRLTurn = 0;
				}
			}
			else if(pan > 30)
			{
				printf("BALL ON THE LEFT SIDE\n");
				KickBall = 0;
				m_GoalFBStep = 0;
				m_GoalRLStep = 30;
				m_GoalRLTurn = 3;
			}
			else if(pan < 30)
			{
				printf("BALL ON THE RIGHT SIDE\n");
				KickBall = 0;
				m_GoalFBStep = 0;
				m_GoalRLStep = -32;
				// if(Walking::GetInstance()->IsRunning() == 1)
				// 	Walking::GetInstance()->Stop();
				// else
				// 	Walking::GetInstance()->Start();
				// usleep(8000);
				// Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				// Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				// Action::GetInstance()->Start(5);
				// usleep(8000);
				// m_GoalRLTurn = 0;
			}
		}
		else
		{
			printf("BALL'S FAR AWAY\n");
			printf("pan = %lf\n",pan);
			pan = Head::GetInstance()->GetPanAngle();
			KickBall = 0;
			m_GoalFBStep = 30 * tilt_percent;
			m_GoalRLStep = 30 * pan_percent;
			m_GoalRLTurn = 0;
		}
	}
	if(m_FBStep < m_GoalFBStep)
		m_FBStep += m_UnitFBStep;
	else if(m_FBStep >= m_GoalFBStep)
		m_FBStep -= m_UnitFBStep;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

	if(m_RLTurn > 0 && m_GoalRLTurn < 0)
		m_RLTurn = 0;
	else if(m_RLTurn < 0 && m_GoalRLTurn > 0)
		m_RLTurn = 0;

	if(m_RLTurn < m_GoalRLTurn)
		m_RLTurn += m_UnitRLTurn;
	else if(m_RLTurn > m_GoalRLTurn)
		m_RLTurn -= m_UnitRLTurn;

	Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

	Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_GoalRLStep;

	CountMaxTurnBall -= Walking::GetInstance()->Get_A_Moved() * 180 / PI;

}

void BallFollower::ProcessGoback(Point2D ball_pos, int CompassError)
{
	if(ball_pos.X == -1 || ball_pos.Y == -1)
	{
		if(Behavior::GetInstance()->get_goback_state() == CHECK_COMPASS)
		{
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
			MaxBallTurn = 0;
			CountMaxTurnBall = 0;
			return;
		}
		else if(Behavior::GetInstance()->get_goback_state() == TURNING)
		{
			MaxBallTurn = -CompassError;
			printf("MaxBallTurn = %lf\n", MaxBallTurn);
			if(abs(CompassError*360/255) < 170)
			{
				Walking::GetInstance()->A_MOVE_AMPLITUDE = -MaxBallTurn * 30 / (abs(MaxBallTurn));
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
			}
			else
			{
				CountMaxTurnBall = 0;
				MaxBallTurn = 0;
				Behavior::GetInstance()->goback_check_own_goal();
			}
			CountMaxTurnBall += Walking::GetInstance()->Get_A_Moved() * 180/PI;
		}
		else if(Behavior::GetInstance()->get_goback_state() == CHECK_OWN_GOAL)
		{
			Walking::GetInstance()->Stop();
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
		}
		else if(Behavior::GetInstance()->get_goback_state() == SEARCHING)
		{
			EdgeOfField = CompassError;
			ProcessSearchingBall();
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_GoalRLTurn;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_GoalFBStep;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_GoalRLStep;
		}
		else if(Behavior::GetInstance()->get_goback_state() == GOING_BACK)
		{
			MaxBallTurn = 2000;
			if(CompassError == 1 || CountMaxTurnBall >= MaxBallTurn)//edge of field = true;
			{
				EdgeOfField = CompassError;
				CountMaxTurnBall = MaxBallTurn;
				Behavior::GetInstance()->goback_searching();
				ProcessSearchingBall();
				Walking::GetInstance()->A_MOVE_AMPLITUDE = m_GoalRLTurn;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = m_GoalFBStep;
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_GoalRLStep;
				return;
			}
			else if(CountMaxTurnBall < MaxBallTurn)
			{
				Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 20;
				Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
			}
			CountMaxTurnBall += Walking::GetInstance()->Get_X_Moved();
		}
	}
	else //kalo bola ketemu
	{
		X_Moved = 0;
		CountMaxTurnBall = 0;
		MaxBallTurn = 0;
		LostCounter = 0;
		Behavior::GetInstance()->set_normal_state();
	}
}
