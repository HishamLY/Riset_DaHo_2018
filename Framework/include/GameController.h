#ifndef _GAMECONTROLLER_H_
#define _GAMECONTROLLRE_H_

#include <iostream>

#include "RoboCupGameControlData.h"
#include "minIni.h"

#define DATA_SECTION   "Robot Data"
#define INVALID_VALUE   -1024.0

using namespace std;

namespace Robot{
	class GameController
	{
    	public :
		GameController();

        	int RobotNumber;
        	int TeamNumber;

		uint8 m_target_goalcolour;
		uint8 m_target_compass;
		uint8 m_old_gamestate;
		uint16 m_last_penalty;

        	RoboCupGameControlData Data;

        	void parseData(char* ReceivePack);   //Parsing and Extract the Package receive
        	void showInterface();    //showing parsing package
		void Process();

        	int ByteConvert(int idxStart, char* ReceivePack, char* type);    //Convert Array of Byte to Short or Int

        	void LoadINISettings(minIni* ini);
        	void LoadINISettings(minIni* ini, const std::string &section);
        	void SaveINISettings(minIni* ini);
        	void SaveINISettings(minIni* ini, const std::string &section);
	};
};

#endif
