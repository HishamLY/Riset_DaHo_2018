/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <libgen.h>
#include <string.h>
//#include <iostream>
using namespace std;

#include "minIni.h"
#include "LinuxNetwork.h"

#define INI_FILE_PATH       "config.ini"

using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== TCP Client Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxSocket* sock = new LinuxSocket();

    if(!sock->create())
    {
        printf("Error creating socket!");
        return 1;
    }
    if(!sock->connect("127.0.0.1",4200))
    {
        printf("Error connect!");
        return 1;
    }

    char buffer[256] = "v";
    //string receivedData;

    while(sock->send(buffer))
    {
        //if(sock->recv(receivedData) != 0)
            //cout << receivedData << endl;
        printf("Input Data : ");
        gets(buffer);
    }

    return 0;
}

