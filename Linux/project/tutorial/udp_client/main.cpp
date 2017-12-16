/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <vector>
#include <cstdlib>
#include <iostream>

#include "minIni.h"
#include "LinuxNetwork.h"

#define INI_FILE_PATH       "config.ini"

using namespace std;
using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== UDP Client Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxSocket* sock = new LinuxSocket();

    char receivedData[256];

    if(!sock->createUdp())
    {
        printf("Error creating socket!");
        return 1;
    }

    sock->setUdp(2129);

    sock->set_non_blocking(true);

    while(1)
    {
        if(sock->sendUdp("FROM CLIENT"))
		cout << "TRANSMITTING" << endl;

//	if(sock->recv(receivedData, 256))
//		cout<< receivedData << endl;

        usleep(100000);
        //system("clear");
    }

    return 0;
}





/*
    Simple udp client
    Silver Moon (m00n.silv3r@gmail.com)
*/
/* =========== TES KOMEN
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>

//#define SERVER "127.0.0.1"
//#define SERVER "192.168.123.1"
#define BUFLEN 512  //Max length of buffer
//#define PORT 8888   //The port on which to send data
#define PORT 3839   //The port on which to send data

void die(char *s)
{
    perror(s);
    exit(1);
}

int main(void)
{
    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
    char buf[BUFLEN];
    char message[BUFLEN];

    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
	 int broadcastEnable = 1;
   if(setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)))
     {
         die("socket");
     }
           
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.s_addr=INADDR_BROADCAST;

/*    if (inet_aton(SERVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
*/
/* =============== TES KOMEN
    while(1)
    {
        printf("Enter message : ");
        gets(message);

        //send the message
        if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }

        //receive a reply and print it
        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);
        //try to receive some data, this is a blocking call
/*        if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == -1)
        {
            die("recvfrom()");
        }

        puts(buf);
*//* =========== TES KOMEN		}

    return 0;
}
*/
