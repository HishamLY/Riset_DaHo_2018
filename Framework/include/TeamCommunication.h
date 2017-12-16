#ifndef TEAMCOMMUNICATION_H_
#define TEAMCOMMUNICATION_H_

#include<stdio.h> //printf
#include<string> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>

#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include "minIni.h"

#define DATA_SECTION   "Team Communication"
#define DATA_SECTION_UDP_RECEIVER   "UDP Receiver"
#define INVALID_VALUE   -1024.0

using namespace std;

namespace Robot {
	class Sender {
    private:
      struct sockaddr_in si_other;
      int m_sock;
      int Port;
      void setSocket();
    public:
      Sender(int port);
      bool send ( std::string ) const;
    	bool send ( void* data, int length ) const;
  };

	class Receiver {
    private:
    	static const int MAXRECV = 500;
      struct sockaddr_in si_me, si_other;
      int m_sock;
      int Port;
			char referee_address[64];
			void setSocket();
    public:
      Receiver(int port);
      void set_non_blocking ( const bool );
      int recv ( std::string& ) const;
      int recv ( void* data, int length ) const;
			bool check_receiver();
			void LoadINISettings(minIni* ini);
      void LoadINISettings(minIni* ini, const std::string &section);
      void SaveINISettings(minIni* ini);
      void SaveINISettings(minIni* ini, const std::string &section);};

	enum StatusPlayer{
		PLAYMAKER1,
		PLAYMAKER2,
		DEFENDER,
		GOAL_KEEPER,
		MAX_STATUS
	};

	struct TeamData{
		double x;
		double y;
		double tilt;
	};

	class TeamCommunication{
		private:
		public:
			TeamCommunication();

			Sender* sender;
      Receiver* receiver;

			std::string DataHeader;
			int PlayerStatus;
			int Port;

			TeamData teamData[MAX_STATUS];

			void InitTeamStatus();
			void CreateConnection();
			void Process(char* Data);

			void LoadINISettings(minIni* ini);
      void LoadINISettings(minIni* ini, const std::string &section);
      void SaveINISettings(minIni* ini);
      void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif
