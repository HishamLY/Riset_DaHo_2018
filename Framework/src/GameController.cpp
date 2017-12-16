
#include "GameController.h"
#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include "LinuxDARwIn.h"
#include "StatusCheck.h"
#include "Behavior.h"

using namespace Robot;

GameController::GameController()
{
	m_target_goalcolour = GOAL_YELLOW;
	m_target_compass = GOAL_YELLOW;
        m_old_gamestate = STATE_INITIAL;
        m_last_penalty = PENALTY_NONE;
}

int GameController::ByteConvert(int idxStart, char* ReceivePack, char* type)
{
	int idxCount;
	int i;
	int Value = 0;

	if(type == "integer") idxCount = 3;
	else if(type =="short") idxCount = 1;

	for(i = idxCount; i >= 0; i--)
	{
		Value = (Value << 8) + ReceivePack[idxStart+i];
	}

	return Value;
}

bool WaitKickOff = false;

timeval tim;
double tStartWaitKickOff, tEndWaitKickOff;

extern bool startKickOff;

void GameController::Process()
{
	//===========   CHECK RIVAL'S COLOR POST   ================
	m_target_goalcolour = Data.teams[0].goalColour;
        if(m_target_goalcolour == GOAL_BLUE)//gawang kita biru, berarti nyerang ke kuning
	{
		printf("GOAL_YELLOW\n");
        	m_target_goalcolour = GOAL_YELLOW;
        	m_target_compass = GOAL_YELLOW;//INI TAMBAHAN LUQMAN
	}
	else
	{
		printf("GOAL_BLUE\n");
        	m_target_goalcolour = GOAL_YELLOW;
		m_target_compass = GOAL_BLUE;// INI TAMBAHAN LUQMAN
	}
	//===========   CHECK STATUS FAULT   ======================
        if(Data.teams[0].players[0].penalty != m_last_penalty)
        {
        	m_last_penalty = Data.teams[0].players[0].penalty;

                if(m_last_penalty != PENALTY_NONE)
                {
                	if(StatusCheck::m_is_started == 1)
                        {
                        	Walking::GetInstance()->Stop();
                                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                                while(Action::GetInstance()->Start(15) == false) usleep(8000);
                                while(Action::GetInstance()->IsRunning() == true) usleep(8000);

                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        }
                }
		else
                {
                        StatusCheck::m_is_started = 1;

                        Action::GetInstance()->Start(9);
                        while(Action::GetInstance()->IsRunning() == true) usleep(8000);

                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                        Behavior::GetInstance()->set_first_penalty_searching();
                 }
	}

	//==================    CHECK STATUS PERTANDINGAN    ======================
                if(Data.state == STATE_PLAYING && WaitKickOff)
                {
                        //Stop Timer
                        gettimeofday(&tim, NULL);
                        tEndWaitKickOff=tim.tv_sec+(tim.tv_usec/1000000.0);
                        //			Behavior::GetInstance()->set_kick_off();//INI TAMBAHAN LUQMAN
                        Behavior::GetInstance()->set_static_state();//INI TAMBAHAN LUQMAN
                        Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
                        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                        /*Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
                        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;*/
                        if((tEndWaitKickOff - tStartWaitKickOff) > 10.0)
                        {
                                WaitKickOff = false;

                                StatusCheck::m_is_started = 1;
                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                MotionManager::GetInstance()->ResetGyroCalibration();
                                while(1)
				{
                                        if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                        {
                                                break;
                                        }
                                        else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                        {
                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                        }
                                        usleep(8000);
                                 }
                        }
                }

		if(Data.state != m_old_gamestate)
                {
                        if(Data.state == STATE_PLAYING)
                        {
			//	startKickOff = true;
				startKickOff = false;
/*                              if(m_old_gamestate == STATE_SET)
                                {
                                        StatusCheck::m_is_started = 1;
                                        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                        MotionManager::GetInstance()->ResetGyroCalibration();
                                        while(1)
					{
                                                if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                                {
                                                        break;
                                                }
                                                else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                                {
                                                        MotionManager::GetInstance()->ResetGyroCalibration();
                                                }
                                                usleep(8000);
                                        }
                                }
*/
				if(m_old_gamestate != STATE_SET)
                                {
                                        MotionManager::GetInstance()->Reinitialize();
                                        MotionManager::GetInstance()->SetEnable(true);
                                        StatusCheck::m_is_started = 1;

                                        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                                        Action::GetInstance()->Start(9);
                                        while(Action::GetInstance()->IsRunning() == true) usleep(8000);

                                        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                        MotionManager::GetInstance()->ResetGyroCalibration();
                                        while(1)
					{
                                        if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                        {
                                                break;
                                        }
                                        else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                        {
                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                        }
                                        usleep(8000);
                                        }
                                }

				if(Data.kickOffTeam == TEAM_BLUE)
                                {
                                        if(Data.teams[0].teamColour == TEAM_CYAN)
                                        {
                                                StatusCheck::m_is_started = 1;
                                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                                while(1)
                                                {
							if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                                        {
                                                                break;
                                                        }
                                                        else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                                        {
                                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                                        }
                                                        usleep(8000);
                                                }
                                        }

					else if(Data.teams[0].teamColour == TEAM_MAGENTA)
                                        {
                                                //Start KickOff Timer
                                                gettimeofday(&tim, NULL);
                                                tStartWaitKickOff=tim.tv_sec+(tim.tv_usec/1000000.0);
                                                WaitKickOff = true;
                                        }
                                }
                                else if(Data.kickOffTeam == TEAM_RED)
                                {
                                        if(Data.teams[0].teamColour == TEAM_CYAN)
                                        {
                                                //Start KickOff Timer
                                                gettimeofday(&tim, NULL);
                                                tStartWaitKickOff=tim.tv_sec+(tim.tv_usec/1000000.0);
                                                WaitKickOff = true;
                                        }
					else if(Data.teams[0].teamColour == TEAM_MAGENTA)
                                        {
                                                StatusCheck::m_is_started = 1;
                                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                                while(1)
                                                {
                                                        if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                                        {
                                                                break;
                                                        }
                                                        else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                                        {
                                                                MotionManager::GetInstance()->ResetGyroCalibration();
                                                        }
                                                        usleep(8000);
                                                }
                                        }
				}
				else    //if DROPBALL
                                {
                                        StatusCheck::m_is_started = 1;
                                        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                        MotionManager::GetInstance()->ResetGyroCalibration();
                                        while(1)
                                        {
                                                if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                                                {
                                                        break;
                                                }
                                                else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                                                {
                                                        MotionManager::GetInstance()->ResetGyroCalibration();
                                                }
                                                usleep(8000);
                                        }
                                }
			}
			else if(Data.state == STATE_SET)
                        {
                                StatusCheck::m_is_started = 1;

				startKickOff = true;//tambahn luqman

                                MotionManager::GetInstance()->Reinitialize();
                                MotionManager::GetInstance()->SetEnable(true);

                                Action::GetInstance()->m_Joint.SetEnableBody(true, true);
                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Walking::GetInstance()->Stop();
                                Action::GetInstance()->Start(9);
                        }
			else
                        {
                                if(StatusCheck::m_is_started == 1)
                                {
                                        StatusCheck::m_is_started    = 0;
                                        Walking::GetInstance()->Stop();
                                        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                                        while(Action::GetInstance()->Start(15) == false) usleep(8000);
                                        while(Action::GetInstance()->IsRunning() == true) usleep(8000);
                                }
                        }

			m_old_gamestate = Data.state;
                }

                if(m_old_gamestate == STATE_SET)
                {
//                        Behavior::GetInstance()->set_kick_off();//INI DIUBAH
//			startKickOff = true;
//                        Walking::GetInstance()->Stop();
			
                        Behavior::GetInstance()->set_static_state();//INI DIUBAH
                }

                if(m_last_penalty != PENALTY_NONE)
                {
                        Behavior::GetInstance()->set_static_state();
                }
				
}

void GameController::parseData(char* ReceivePack)
{
	int idx;

	strncpy(Data.header, ReceivePack, 4);

	Data.version = ByteConvert(4, ReceivePack, "integer");
	Data.playersPerTeam = ReceivePack[8];
	Data.state = ReceivePack[9];
	Data.firstHalf = ReceivePack[10];
	Data.kickOffTeam = ReceivePack[11];
	Data.secondaryState = ReceivePack[12];
	Data.dropInTeam = ReceivePack[13];
	Data.dropInTime = ByteConvert(14, ReceivePack, "short");
	Data.secsRemaining = ByteConvert(16, ReceivePack, "integer");

    Data.teams[0].teamNumber = TeamNumber;

	if(Data.teams[0].teamNumber == ReceivePack[20])
	{
		Data.teams[0].teamColour = ReceivePack[21];
		Data.teams[0].goalColour = ReceivePack[22];
		Data.teams[0].score = ReceivePack[23];
		Data.teams[1].score = ReceivePack[71];  //rival score
		idx = 24 + 4*(RobotNumber - 1);
		Data.teams[0].players[0].penalty = ByteConvert(idx, ReceivePack, "short");
		Data.teams[0].players[0].secsTillUnpenalised = ByteConvert(idx+2, ReceivePack, "short");
	}
	else if(Data.teams[0].teamNumber == ReceivePack[68])
	{
		Data.teams[0].teamColour = ReceivePack[69];
		Data.teams[0].goalColour = ReceivePack[70];
		Data.teams[0].score = ReceivePack[71];
		Data.teams[1].score = ReceivePack[23];  //rival score
		idx = 72 + 4*(RobotNumber - 1);
		Data.teams[0].players[0].penalty = ByteConvert(idx, ReceivePack, "short");
		Data.teams[0].players[0].secsTillUnpenalised = ByteConvert(idx+2, ReceivePack, "short");
	}
	else
	{
	    Data.teams[0].teamNumber = 100;
	}
}

void GameController::showInterface()
{
    printf("\n====== GAME CONTROLLER DATA ======\n");
	//Header
	printf("- Header		= '%s'\n",Data.header);
	//Version
	printf("- Version		= %d\n",Data.version);
	//playersPerTeam
	printf("- PlayersPerTeam	= %d\n",Data.playersPerTeam);
	//state
	if(Data.state == STATE_INITIAL)
		printf("- State			= Initial\n");
	else if(Data.state == STATE_READY)
		printf("- State			= Ready\n");
	else if(Data.state == STATE_SET)
		printf("- State			= Set\n");
	else if(Data.state == STATE_PLAYING)
		printf("- State			= Playing\n");
	else if(Data.state == STATE_FINISHED)
		printf("- State			= Finished\n");
	//kickOffTeam
	if(Data.kickOffTeam == TEAM_BLUE)
		printf("- KickOffTeam		= Blue\n");
	else if(Data.kickOffTeam == TEAM_RED)
		printf("- KickOffTeam		= Red\n");
	else if(Data.kickOffTeam == DROPBALL)
		printf("- KickOffTeam		= Drop ball\n");

	//Secondary State
	if(Data.secondaryState == STATE2_NORMAL)
	{
		if(Data.firstHalf == 1)
			printf("- Secondary State	= Halftime - First\n");
		else if(Data.firstHalf == 0)
			printf("- Secondary State	= Halftime - Second\n");
	}
	else if(Data.secondaryState == STATE2_OVERTIME)
	{
		if(Data.firstHalf == 1)
			printf("- Secondary State	= Overtime - First\n");
		else if(Data.firstHalf == 0)
			printf("- Secondary State	= Overtime - Second\n");
	}
	else if(Data.secondaryState == STATE2_PENALTYSHOOT)
	{
		if(Data.firstHalf == 0)
		{
			if(Data.teams[0].teamColour == TEAM_CYAN)
				printf("- Secondary State	= Penalty shoot - Our Turn\n");
			else if(Data.teams[0].teamColour == TEAM_MAGENTA)
                                printf("- Secondary State       = Penalty shoot - Rival Turn\n");
		}
		else if(Data.firstHalf == 1)
		{
			if(Data.teams[0].teamColour == TEAM_MAGENTA)
                                printf("- Secondary State       = Penalty shoot - Our Turn\n");
                        else if(Data.teams[0].teamColour == TEAM_CYAN)
                                printf("- Secondary State       = Penalty shoot - Rival Turn\n");
		}
	}
	//dropInTeam
	if(Data.dropInTeam == 0)
		printf("- DropInTeam		= Magenta\n");
	else if(Data.dropInTeam == 1)
		printf("- DropInTeam		= Cyan\n");
	//dropInTime
	printf("- DropInTime		= %d\n",Data.dropInTime);
	//secsRemaining
	printf("- SecondRemaining	= %d : %d\n", Data.secsRemaining/60, Data.secsRemaining%60);
	//teamNumber
	if(Data.teams[0].teamNumber == 100)
	{
	    printf("Your team is currently not playing\n");
	    return;
	}
	if(Data.teams[0].teamColour == TEAM_CYAN)
		 printf("- TeamNumber		= %d - cyan\n",Data.teams[0].teamNumber);
	else if(Data.teams[0].teamColour == TEAM_MAGENTA)
	     printf("- TeamNumber		= %d - magenta\n",Data.teams[0].teamNumber);
    //robotNumber
		printf("- robotNumber		= %d\n",RobotNumber);
	//goalColour
	if(Data.teams[0].goalColour == GOAL_BLUE)
		 printf("- Own GoalColour	= Blue\n");
	else if(Data.teams[0].goalColour == GOAL_YELLOW)
		 printf("- Own GoalColour	= Yellow\n");
	//score
		printf("- Score			= [Our Team] %d : %d [Rival Team]\n", Data.teams[0].score, Data.teams[1].score);
	//Penalty State
	if(Data.teams[0].players[0].penalty == PENALTY_NONE)
		printf("- Penalty		= No Penalty\n");
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_BALL_MANIPULATION)
		printf("- Penalty		= Ball manipulation - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_PHYSICAL_CONTACT)
		printf("- Penalty		= Physical Contact - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_ILLEGAL_ATTACK)
		printf("- Penalty		= Illegal Attack - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_ILLEGAL_DEFENSE)
		printf("- Penalty		= Illegal Defense - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_REQUEST_FOR_PICKUP)
		printf("- Penalty		= Request For PickUp - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_REQUEST_FOR_SERVICE)
		printf("- Penalty		= Request For Service - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE)
		printf("- Penalty		= Upgrade PickUp to Service - %d\n",Data.teams[0].players[0].secsTillUnpenalised);
	else if(Data.teams[0].players[0].penalty == PENALTY_MANUAL)
		printf("- Penalty		= Manual\n");
}

void GameController::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, DATA_SECTION);
}

void GameController::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    if((value = ini->geti(section, "RobotNumber", INVALID_VALUE)) != INVALID_VALUE)     RobotNumber = value;
    if((value = ini->geti(section, "TeamNumber", INVALID_VALUE)) != INVALID_VALUE)      TeamNumber = value;
}

void GameController::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, DATA_SECTION);
}

void GameController::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "RobotNumber",      RobotNumber);
    ini->put(section,   "TeamNumber",       TeamNumber);
}
