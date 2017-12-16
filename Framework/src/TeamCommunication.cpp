#include "TeamCommunication.h"

#include <iostream>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sstream>

using namespace Robot;

void die(char *s)
{
  perror(s);
  exit(1);
}

//Set up Array team status untuk komunikasi pada port 2013
TeamCommunication::TeamCommunication():Port(2103),PlayerStatus(1)
{
	InitTeamStatus();
}

//Melakukan assign port kepada sender dan receiver
void TeamCommunication::CreateConnection()
{
	sender = new Sender(Port);
  receiver = new Receiver(Port);
}

//Melakukan inisiasi team status
void TeamCommunication::InitTeamStatus()
{
	for(int i = 0; i < MAX_STATUS; i++)
    teamData[i] = {-1.0, -1.0, 0};
}

//Melakukan pengolahan data yang dikirim melalui socket agar data dapat diassign (merubah bentuk array of char ke tipe float)
void TeamCommunication::Process(char* Data)
{
	char *delimit;
  delimit = strtok(Data,"#");
	char *Header = delimit;
	delimit = strtok(NULL, "#");
  int PlayerStatus = atoi(delimit); //atoi merupakan fungsi untuk casting string ke integer
  delimit = strtok(NULL, "#");
  double Tilt = atof(delimit); //atof merupakan fungsi untuk casting string ke float
  delimit = strtok(NULL, "#");
  double X = atof(delimit);
  delimit = strtok(NULL, "#");
  double Y = atof(delimit);
  teamData[PlayerStatus-1] = {X, Y, Tilt};

// for(int i = 0; i < 4; i++)
//   printf("PlayerStatus = %d, Tilt = %lf, (%lf, %lf)\n", i+1, teamData[i].tilt, teamData[i].x, teamData[i].y);
}

void TeamCommunication::LoadINISettings(minIni* ini)
{
  LoadINISettings(ini, DATA_SECTION);
}

void TeamCommunication::LoadINISettings(minIni* ini, const std::string &section)
{
  int value = -2;
  if((value = ini->geti(section, "RobotNumber", INVALID_VALUE)) != INVALID_VALUE)
    PlayerStatus = value;
  if((value = ini->geti(section, "Port", INVALID_VALUE)) != INVALID_VALUE)
    Port = value;
  std::string svalue = "";
  if((svalue = ini->gets(section, "DataHeader", "")) != "")
    DataHeader = svalue;
}

void TeamCommunication::SaveINISettings(minIni* ini)
{
  SaveINISettings(ini, DATA_SECTION);
}

void TeamCommunication::SaveINISettings(minIni* ini, const std::string &section)
{
  ini->put(section,   "RobotNumber",      PlayerStatus);
  ini->put(section,   "Port",       Port);
}

//Konstruktor Sender
Sender::Sender(int port)
{
	Port = port;
	setSocket();
}

//Prosedur untuk melakukan pembuatan socket komunikasi sender
void Sender::setSocket()
{
  //Proses pembuatan socket UDP untuk komunikasi
	if ( (m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    die("socket");
  }

  int broadcastEnable = 1;
  //Melakukan pengaturan pada socket sehingga socket dapat melakukan broadcast
	if(setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)))
	{
		die("socket");
	}

	memset((char *) &si_other, 0, sizeof(si_other));

  //Inisiasi Socket Address
	si_other.sin_family = AF_INET;
  si_other.sin_port = htons(Port); //htons fungsi untuk casting dari integer ke dalam tipe address
  si_other.sin_addr.s_addr = INADDR_BROADCAST;
}

//Fungsi untuk melakukan pengiriman data kepada receiver dengan parameter string
bool Sender::send ( const std::string s ) const
{
  int status = ::sendto ( m_sock, s.c_str(), s.size(), 0, (struct sockaddr*) &si_other, sizeof(si_other) );
  if (status == -1)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//Fungsi untuk melakukan pengiriman data kepada receiver dengan parameter tipe bentukan dan panjang data yang dikirim
bool Sender::send ( void* data, int length ) const
{
  int status = ::sendto ( m_sock, data, length, 0, (struct sockaddr*) &si_other, sizeof(si_other) );
  if ( status == -1 )
  {
    return false;
  }
  else
  {
      return true;
  }
}

//Konstruktor receiver
Receiver::Receiver(int port)
{
  Port = port;
  setSocket();
}

//Prosedur untuk melakukan set socket pada receiver
//Sama dengan set socket pada sender
void Receiver::setSocket()
{
  //Set socket menjadi UDP
  if ( (m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    die("socket");
  }

  int broadcastEnable = 1;
  if(setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)))
  {
    die("socket");
  }

	memset((char *) &si_me, 0, sizeof(si_me));
	memset((char *) &si_other, 0, sizeof(si_other));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(Port);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if( bind(m_sock , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
  {
  	die("bind");
  }
}

//Fungsi untuk melakukan receive message, yang akan mengembalikan panjang message yang dikirimkan jika berhasil dan -1 jika gagal
int Receiver::recv ( std::string& s ) const
{
    char buf [ MAXRECV + 1 ];

    s = "";

    memset ( buf, 0, MAXRECV + 1 );

    // int status = ::recv ( m_sock, buf, MAXRECV, 0 );
    unsigned int slen = sizeof(si_other);
    // int status = ::recvfrom( m_sock, buf, MAXRECV, 0, (struct sockaddr *) &si_other, (socklen_t*)&sizeof(si_other));
    int status = ::recvfrom( m_sock, buf, MAXRECV, 0, (struct sockaddr *) &si_other, &slen); //Melakukan penerimaan message yang telah dikirim oleh sender

    //Return value dari recvfrom -1 jika gagal dan returnval > 0 jika berhasil
    //Return value berhasil merupakan panjang message yang diterima oleh receiver
    if ( status == -1 )
    {
        cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
        return 0;
    }
    else if ( status == 0 )
    {
        return 0;
    }
    else
    {
        s = buf;
        return status;
    }
}

//Fungsi untuk melakukan receive message, yang akan mengembalikan panjang message yang dikirimkan jika berhasil dan -1 jika gagal
int Receiver::recv ( void* data, int length ) const
{
    //int status = ::recv ( m_sock, data, length, 0 );
    int slen = sizeof(si_other);
    int status = ::recvfrom( m_sock, data, length, 0, (struct sockaddr *) &si_other, (socklen_t*)&slen) ;

    //printf("Received packet from %s:%hu\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

    if ( status == -1 )
    {
        //cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
        return 0;
    }
    else if ( status == 0 )
    {
        return 0;
    }

    return status;
}

//Prosedur yang digunakan agar proses receiver atau fungsi recvfrom pada receiver tidak melakukan blocking terhadap proses send and receive yang berjalan
//Receiver dalam hal ini robot juga dapat mengirimkan message pada robot lainnya
void Receiver::set_non_blocking ( const bool b )
{
    int opts;

    opts = fcntl ( m_sock, F_GETFL );

    if ( opts < 0 )
    {
        return;
    }

    if ( b )
        opts = ( opts | O_NONBLOCK );
    else
        opts = ( opts & ~O_NONBLOCK );

    fcntl ( m_sock,
            F_SETFL,opts );
}

//Fungsi untuk mengecek socket address dari receiver
bool Receiver::check_receiver()
{
	if(strcmp(inet_ntoa(si_other.sin_addr), referee_address) == 0)
    return true;
	else
		return false;
}

void Receiver::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, DATA_SECTION_UDP_RECEIVER);
}

void Receiver::LoadINISettings(minIni* ini, const std::string &section)
{
    std::string svalue = "";
    if((svalue = ini->gets(section, "RefereeAddress", "")) != "")	strcpy(referee_address, svalue.c_str());
}
