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

#include "GameController.h"
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
    printf( "\n===== GameController Receiver Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxSocket* sock = new LinuxSocket();

    GameController* GC = new GameController();

    GC->LoadINISettings(ini);

    char receivedData[256];

    if(!sock->createUdp())
    {
        printf("Error creating socket!");
        return 1;
    }

    if(!sock->bind(3838))
    {
        printf("Error binding!");
        return 1;
    }

    //sock->set_non_blocking(true);

    while(1)
    {
        if(sock->recv(receivedData, 256) == 116)
        {
            GC->parseGameControlPackage(receivedData);
            GC->showInterface();
        }

        usleep(100000);
        //system("clear");
    }

    return 0;
}
