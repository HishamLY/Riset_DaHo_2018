/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include "StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"
#include "LinuxActionScript.h"
#include "Behavior.h"

using namespace Robot;

int StatusCheck::m_cur_mode     = READY;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;

int *pValue=new int();

bool ListenGameController = false;

void StatusCheck::Check(CM730 &cm730)
{
  if (cm730.ReadByte(200,50, pValue, NULL) == 0)
    {
/*
      if (*pValue < 112)//118
        {
          printf("Baterry Status : %d\n",*pValue);
          LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Cat_Meow_short.mp3");
          printf("baterry_low\n");
        }
*/
    }

  if(MotionStatus::FALLEN != STANDUP && m_is_started == 1)
    {
      //Buat servo off
      Head::GetInstance()->MoveByAngle(0,65);
      usleep(250000);
      MotionManager::GetInstance()->SetEnable(false);
      MotionManager::GetInstance()->SetDisableServo(); 
      usleep(2000000);
      MotionManager::GetInstance()->SetEnable(true);
      //////

      Action::GetInstance()->Stop();
      while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
      if(Walking::GetInstance()->IsRunning() == 1)
	      Walking::GetInstance()->Stop();
      while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);

      Action::GetInstance()->m_Joint.SetEnableBody(true, true);

    if(MotionStatus::FALLEN == FORWARD) {
      	Action::GetInstance()->Start(86);   // FORWARD GETUP, no grass 20
		//Walking::GetInstance()->X_OFFSET = 0;
	}
    else if(MotionStatus::FALLEN == BACKWARD && *pValue > 114) {
    	Action::GetInstance()->Start(88);//88 //99   // BACKWARD GETUP, no grass 11
		//Walking::GetInstance()->X_OFFSET = 0;
	}
    else if(MotionStatus::FALLEN == BACKWARD && *pValue <= 114) {
      Action::GetInstance()->Start(34);//88 //34   // BACKWARD GETUP, no grass 11
    //Walking::GetInstance()->X_OFFSET = 0;
  }/*
	else if(MotionStatus::FALLEN = BACKWARD && *pValue > 114) {
		Action::GetInstance()->Start(99);
	}*/

      while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			Walking::GetInstance()->Initialize();
		  Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
		  Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
		  Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;

      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

    }

  if(m_old_btn == MotionStatus::BUTTON)
    return;

  m_old_btn = MotionStatus::BUTTON;

  if(m_old_btn & BTN_MODE)
    {
      fprintf(stderr, "Mode button pressed.. \n");

      if(m_is_started == 1)
        {
          m_is_started    = 0;
          m_cur_mode      = READY;
          LinuxActionScript::m_stop = 1;

          Walking::GetInstance()->Stop();
          Action::GetInstance()->m_Joint.SetEnableBody(true, true);

          while(Action::GetInstance()->Start(9) == false) usleep(8000); //Kalau duduk 9
          while(Action::GetInstance()->IsRunning() == true) usleep(8000);
/*
	Action::GetInstance()->Start(9);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
*/
          ListenGameController = false;
        }
      else
        {
          m_cur_mode++;
          if(m_cur_mode >= MAX_MODE) m_cur_mode = READY;
        }

      MotionManager::GetInstance()->SetEnable(false);
      usleep(10000);

      if(m_cur_mode == READY)
        {
          cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
        }
      else if(m_cur_mode == SOCCER_YELLOW_POST)
        {
          cm730.WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Autonomous soccer mode.mp3");
        }
      else if(m_cur_mode == SOCCER_BLUE_POST)
        {
          cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Interactive motion mode.mp3");
        }
      else if(m_cur_mode == PENALTY_KICK)
        {
          cm730.WriteByte(CM730::P_LED_PANNEL, 0x04, NULL);
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Vision processing mode.mp3");
        }
      else if(m_cur_mode == GAME_CONTROLLER)
        {
          cm730.WriteByte(CM730::P_LED_PANNEL, 0x00, NULL);
        }
    }
     
  if(m_old_btn & BTN_START)
    {
      if(m_is_started == 0)
        {
          fprintf(stderr, "Start button pressed.. & started is false.. \n");

          if(m_cur_mode == SOCCER_YELLOW_POST || m_cur_mode == SOCCER_BLUE_POST)
            {
              MotionManager::GetInstance()->Reinitialize();
              MotionManager::GetInstance()->SetEnable(true);
              m_is_started = 1;
              LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

              Action::GetInstance()->m_Joint.SetEnableBody(true, true);

              //Action::GetInstance()->Start(12);
              while(Action::GetInstance()->Start(9) == false) usleep(8000);
              while(Action::GetInstance()->IsRunning() == true) usleep(8000);

              Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
              Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

              MotionManager::GetInstance()->ResetGyroCalibration();
              while(1)
                {
                  if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                    {
                      //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
                      break;
                    }
                  else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                    {
                      //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
                      MotionManager::GetInstance()->ResetGyroCalibration();
                    }
                  usleep(8000);
                }
            }
          else if(m_cur_mode == PENALTY_KICK)
            {
              MotionManager::GetInstance()->Reinitialize();
              MotionManager::GetInstance()->SetEnable(true);
              m_is_started = 1;
              //LinuxActionScript::PlayMP3("../../../Data/mp3/Start vision processing demonstration.mp3");

              // Joint Enable...
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
                      //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
                      break;
                    }
                  else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                    {
                      //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
                      MotionManager::GetInstance()->ResetGyroCalibration();
                    }
                  usleep(8000);
                }
            }
          else if(m_cur_mode == GAME_CONTROLLER)
            {
              ListenGameController = true;
            }
        }
      else
        {
          fprintf(stderr, "Start button pressed.. & started is true.. \n");
        }
    }
}
