#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <string.h>

#include "minIni.h"
#include "LinuxDARwIn.h"

#define BEHAVIOR_SECTION "Behavior"
#define INVALID_VALUE   -1024.0

namespace Robot
{
	enum{
		//Behavior State
		READY_STATE = 0,
		KICK_OFF,
		NORMAL_STATE,
		DEFENSE_STATE,
		GOBACK_STATE,
    		CHECK_POST_STATE,
		APROACHING_BALL,
    		TURN_AROUND_BALL_STATE,
    		DRIBBLE_STATE,
		KICK_STATE,
		STATIC_STATE,
		FIRST_PENALTY_SEARCHING,
		LOCALIZE_PENALTY_MARK,
		LOCALIZE_GOALPOST,
		PASS_BALL_STATE,
		PENALTY_KICK_STATE,
		CARI_BOLA_STATE, //Cari bola kalau bingung
    		SWITCHING_TRANSITION, //Ganti Resolusi
		NONE,
		UNKNOWN_POST,
		POSSIBLE_RIGHT_POST,
		POSSIBLE_LEFT_POST,
		RIGHT_POST,
		LEFT_POST,
		BOTH_POST
        };

	enum{
		//Goback State
		CHECK_COMPASS,
		TURNING,
		CHECK_OWN_GOAL,
		SEARCHING,
		GOING_BACK
		};

			

	class Behavior
	{
		private:
			static Behavior* m_UniqueInstance;
			
			//constructor
			Behavior();
			
			//compass for turning ball
			int sp;
			int kompensator;
			int cmps_now;
			int cmps_error;
			double CompassError;
			double CompassPost;
			
			//used variable
			double PanLastSeenBall;
			int State;
			int GoBackState;
			int StepToDo;
			int PostFound;		
			double PanPostRight;
			double PanPostLeft;
			double PanPostBoth;
			int PostPosition;
	
			//Object for other class
			BallTracker Tracker;
      			BallFollower Follower;
			
			//varible for process state
			Point2D ObjectPos;
			int ObjectSize;
			Vector2<int> ObjectDistance;
			int CounterCariBola;
			bool flagCounterCariBola;
			bool IsOneCycle;
			//used for odometry in ProcessAproachingBall2
			double X_Moved;
			
			//FSM
			void ReadyState();
			void KickOff();
			void NormalState();
			void DefenseState();
			void GobackState();
      			void checkPost();
			void ProcessAproachingBall2();
      			void turningBall();
      			void dribbleBall();
			void StaticState();
			void KickState();
			void FirstPenaltySearching();
			void LocalizePenaltyMark();
			void LocalizeGoalPost();
			void PassBall();
			void PenaltyKick();

			//Cari Bola kalau robot bingung
			void CariBola();
			void SwitchingTransition();

		public:

			//state waiting
			int CountToReady;
      			bool WaitUntilReady;
			static const int MaxCountToReady = 3;
			int Count;
			
			static Behavior* GetInstance() { return m_UniqueInstance; }
			
			//setting GOBACK State
			void goback_check_compass(){ GoBackState = CHECK_COMPASS; }
			void goback_turning(){ GoBackState = TURNING; }
			void goback_check_own_goal(){ GoBackState = CHECK_OWN_GOAL; }
			void goback_searching(){ GoBackState = SEARCHING; }
			void goback_backward(){ GoBackState = BACKWARD;	}
			
			//Setting FSM
			void set_ready_state(){ State = READY_STATE; }
		        void set_kick_off(){ State = KICK_OFF; }
			void set_normal_state(){ State = NORMAL_STATE; }
		        void set_defense_state(){ State = DEFENSE_STATE; }
			void set_goback_state(){ State = GOBACK_STATE; }
			void set_check_post_position(){ State = CHECK_POST_STATE; }
			void set_aproaching_ball(){ State = APROACHING_BALL; }
		        void set_turn_around_ball(){ State = TURN_AROUND_BALL_STATE; }
		        void set_dribble_ball(){ State = DRIBBLE_STATE; }
			void set_static_state(){ State = STATIC_STATE; }
			void set_kick_state(){ State = KICK_STATE; }
			void set_first_penalty_searching(){ State = FIRST_PENALTY_SEARCHING; }
			void set_localize_penalty_mark(){ State = LOCALIZE_PENALTY_MARK; }
			void set_localize_goalpost(){ State = LOCALIZE_GOALPOST; }
			void set_pass_ball_state(){ State = PASS_BALL_STATE; }
			void set_penalty_kick_state(){ State = PENALTY_KICK_STATE;	}

			int get_state() { return State; };
			int get_goback_state() { return GoBackState; };

			void Process(Point2D center);
			void Process(Point2D center,int Objectsize, Vector2<int> Distance);
			void CheckStatus();

			void LoadINISettings(minIni* ini);
				void LoadINISettings(minIni* ini, const std::string &section);
			void SaveINISettings(minIni* ini);
			void SaveINISettings(minIni* ini, const std::string &section);


			//Lokalisasi kasar state
			float r1; //Jarak ke tiang kiri
			float r2; //Jarak ke tiang kanan
			float corner2leftpost; //jarak dari titik 0,0 atau pojok kiri gawang ke tiang kiri gawang
			float L, P; //lebar dan panjang lapangan dalam cm
			float x_loc, y_loc; //0,0 dari titik tengah lapangan, positif ke arah gawang lawan

			//Switch Resolution
			bool Bingung;
			bool flagSwitchResolution;
			bool flagLagiSwitch;
			int counterLagiSwitch;
			void set_switching_transition(){ State = SWITCHING_TRANSITION; }
			int CountTendang;
	};
}

#endif
