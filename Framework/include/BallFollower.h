/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _BALL_FOLLOWER_H_
#define _BALL_FOLLOWER_H_

#include "Point.h"
#include "BallTracker.h"
#include "LinuxDARwIn.h"

namespace Robot
{
	class BallFollower
	{
	private:

		enum{
			KANAN,
			KIRI
			};
		int TurnDirection;

		bool flagNemuBola; //ini dipake di state cari bola
		bool flagUdahMuter; //Nandain kalau udah muter
		double CmpsSPTurn;//nilai awal sebelum turning
		double CmpsErrTurn;//error compass, jika turning selesai, errornya = 0;

		int StateGoback;			
		int flagDribble;
		int m_NoBallMaxCount;
		int m_NoBallCount;
		int m_KickBallMaxCount;
		int m_KickBallCount;
		int m_CountBallFound;
		int m_MaxCountBallFound;
		int LostCounter;
		int MaxLostCounter;
		int searchCounter;

		int CountFirstTilt;
		int MaxCountFirstTilt;

		double MaxBallTurn;
		double CountMaxTurnBall; //untuk mengetahui A-Moved dalam turning ball
		bool PostEqualZero;
		int onesegitiga; //untuk mengetahui daritem udah bikin 1 lap segitiga sebelum lokalisasi kasar

		double X_Moved; //untuk mengetahui X_Moved
		double A_Moved;
		double Y_Moved;
		double BallDistance;
		bool turn;
		bool Goback;

		double m_MaxFBStep;
		double m_MaxRLStep;
		double m_MaxDirAngle;

		double m_KickTopAngle;
		double m_KickRightAngle;
		double m_KickLeftAngle;

		double m_FollowMaxFBStep;
	        double m_FollowMinFBStep;
		double m_FollowMaxRLTurn;
	        double m_FollowMaxRLStep;
		double m_FitFBStep;
		double m_FitMaxRLTurn;
		double m_UnitFBStep;
		double m_UnitRLTurn;
		double m_UnitRLStep;
		double m_UnitZMove;

		double m_FBStep;
		double m_RLTurn;
		double m_RLStep;
		double m_Zmove;

		double panLastSeenBall;
		double tiltLastSeenBall;
		double PostDist;
		double pan_now;

	protected:

	public:
		double m_GoalFBStep;
		double m_GoalRLTurn;
		double m_GoalRLStep;
		double m_GoalZMove;

	        bool FirstTilt; //untuk mendapatkan nilai sudut tilt pertama saat ketemu suatu object
        	double TiltAngle;//untuk mendapatkan nilai tilt pertama saat dapet bola
        	double PanAngle;
		double LastTilt; //untuk mendaptkan nilai tilt terakhir sebelum ngecek gawang
		int Sizeball; //ukuran bola
	
		bool DEBUG_PRINT;
		int KickBall;		// 0: No ball 1:Left -1:Right
		int KickDir;
		bool TurnBall;		
		bool EdgeOfField;

		BallFollower();
		~BallFollower();

		void ProcessCariBola(Point2D ball_pos, bool *IsOneCycle, Vector2<int> Jarak);
		void Process(Point2D ball_pos);
		void ProcessDefense(Point2D ball_pos, Vector2<int> ppfBall);
		void ProcessAproachingBall(Point2D ball_pos, Vector2<int> ppfBall, bool EdgeField);
		void ProcessAproachingBall2(Point2D ball_pos,double panpost, double CompassError, double CompassPost);
		//void ProcessToKick(Point2D ball_pos);
		void ProcessToKick(Point2D ball_pos, double PanPost, double CompassError, double compassPost);
		void ProcessTurnAroundBall(Point2D ball_pos, double PanPost, int CompassError, int PostPosition);
		void ProcessSearchingBall();
		void ProcessKick(Point2D ball_pos);
		void ProcessFirstPenaltySearching(Point2D ball_pos);
		void ProcessLocalizePenaltyMark(Point2D ball_pos);
		void ProcessLocalizeGoalPost(Point2D ball_pos);
		void ProcessPassBall(Point2D ball_pos);
		void ProcessPenaltyKick(Point2D ball_pos);
		void ProcessGoback(Point2D ball_pos, int CompassError);
		void initMember() { m_GoalFBStep = 0; m_GoalRLTurn = 0; m_GoalRLStep = 0;}

		void InitKickBallCount() { m_KickBallCount = 0; }

		int countCheckBall;
		int counterGetBall;
		int counterGetBallFound;
	};
}

#endif
