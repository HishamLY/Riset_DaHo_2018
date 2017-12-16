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

        };

		

	class Behavior
	{
		private:
			static Behavior* m_UniqueInstance;

			Behavior();

			int State;
			int PostStatus;
			int StepToDo;
			int PostFound;			
			double PanPostRight;
			double PanPostLeft;
			double PanPostBoth;
			static const int MaxCountToReady = 3;

			BallTracker Tracker;
      	                BallFollower Follower;
			
			Point2D ObjectPos;
			int ObjectSize;
			Vector2<int> ObjectDistance;

			double X_Moved;
			
			void (


		public:
			int CountToReady;
                        bool WaitUntilReady;

			static Behavior* GetInstance() { return m_UniqueInstance; }
			

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

			void Process(Point2D center);
			void Process(Point2D center,int Objectsize, Vector2<int> Distance);
			void CheckStatus();

			void LoadINISettings(minIni* ini);
        		void LoadINISettings(minIni* ini, const std::string &section);
        		void SaveINISettings(minIni* ini);
        		void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif
