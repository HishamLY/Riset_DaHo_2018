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
    printf( "\n===== TCP Server Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxSocket* sock = new LinuxSocket();
    LinuxSocket newSocket;
    vector<LinuxSocket> newSock;
    string receivedData;

    if(!sock->create())
    {
        printf("Error creating socket!");
        return 1;
    }

    if(!sock->bind(4200))
    {
        printf("Error binding!");
        return 1;
    }

    if(!sock->listen())
    {
        printf("Error listen!");
        return 1;
    }

    sock->set_non_blocking(true);

    while(1)
    {
        if(sock->accept(newSocket))
        {
            newSocket.set_non_blocking(true);
            newSock.push_back(newSocket);
        }
        for(vector<LinuxSocket>::iterator iter = newSock.begin(); iter != newSock.end(); iter++)
        {
            (*iter).recv(receivedData);
            printf("%s\n",receivedData.c_str());
        }

        printf("TOTAL CONNECTION HISTORY: %d\n",newSock.size());
        usleep(500000);
    }

    return 0;
}

