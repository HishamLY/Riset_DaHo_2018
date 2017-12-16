/*
 *   Head.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _HEAD_H_
#define _HEAD_H_

#include <string.h>

#include "minIni.h"
#include "MotionModule.h"
#include "Point.h"

#define HEAD_SECTION    "Head Pan/Tilt"
#define INVALID_VALUE   -1024.0

namespace Robot
{
	/********* MODIFICATION ********/
	enum LastPanAngle{
		LEFT = 1,
		RIGHT = -1
	};
	/*******************************/

	enum HeadState{
		STATE0 = 0, STATE1, STATE2, STATE3, STATE4, STATE5, STATE6, STATE7, STATE8
	};

	class Head : public MotionModule
	{
	private:
	/********* MODIFICATION ********/
		int inc;
		int inc_pan;
		int inc_tilt;
		int headState;
		
	/*******************************/
		static Head* m_UniqueInstance;
		double m_LeftLimit1;
		double m_RightLimit1;
		double m_TopLimit;
		double m_BottomLimit;
		double m_Pan_Home;
		double m_Tilt_Home;
		double m_Pan_err;
		double m_Pan_err_diff;
		double m_Pan_p_gain;
		double m_Pan_d_gain;
		double m_Tilt_err;
		double m_Tilt_err_diff;
		double m_Tilt_p_gain;
		double m_Tilt_d_gain;
		double m_PanAngle;
		double m_TiltAngle;
		
		Head();
		void CheckLimit();

	public:
		static Head* GetInstance() { return m_UniqueInstance; }
		double m_LeftLimit;
		double m_RightLimit;
		double m_LastPanAngle;
		
		~Head();

		void Initialize();
		void Process();

		void MoveRight(int increment);
		void MoveLeft(int increment);
		double GetTopLimitAngle()		{ return m_TopLimit; }
		void MoveUp(int increment);
		void MoveDown(int increment);
		double GetBottomLimitAngle()	{ return m_BottomLimit; }
		double GetRightLimitAngle()		{ return m_RightLimit; }
		double GetLeftLimitAngle()		{ return m_LeftLimit; }

		double GetPanAngle()		{ return m_PanAngle; }
		double GetTiltAngle()		{ return m_TiltAngle; }

		void MoveToHome();
		void MoveByAngle(double pan, double tilt);
		void MoveByAngleOffset(double pan, double tilt);
		void InitTracking();
		void MoveTracking(Point2D err); // For image processing
		void MoveTracking();

	/******** MODIFICATION *********/
		void MoveTrackingFar(Point2D err);
		void MoveTrackingFar();
		int dir;
		bool step1, step2, step3, step4;
		void initSearching(bool phase1, bool phase2, bool phase3, bool phase4);
		void initSearchingPost(int direction);
		void MoveSearching();
		void MoveSearchingTurn();
		void SetLimit(int CompassError);
		void MoveSearchingPostLeft();
		void MoveSearchingPostRight();
		void MoveSearchingPost(int CompassError);
		void MoveSearchingPost();
		void MoveSearchingPostOneCycle();
		void MoveSearchingPenaltyKick();
		double GetLastPanAngle()	{ m_LastPanAngle = m_PanAngle; }
		LastPanAngle CheckLastPanAngle()	{ if(m_LastPanAngle < m_Pan_Home) return RIGHT; else return LEFT; }
		int SearchingDir() { return dir;}
		void MoveSearchingPenaltyMark();
		void LookUp();
		void LookDown();
		double GetHorizon();
	/*******************************/

        void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif
