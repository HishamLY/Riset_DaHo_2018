/*
 * main.cpp
 *
 *  Created on: 2013. 3. 22.
 *      Author: KRSBI ITB
 */

#include <stdio.h>
#include <libgen.h>
#include <sys/time.h>

#include "Compass.h"
//#include <GL/glut.h>
//#include  <qt4/QtGui/QApplication>

#define INI_FILE_PATH       "../../../../Data/config.ini"

#if defined (_WIN32) || defined( _WIN64)
#define         DEVICE_PORT             "COM5"                               // COM5 for windows
#endif

#ifdef __linux__
#define   DEVICE_PORT     "/dev/Compass"                         // ttyUSB0 for linux
#define   DEVICE_PORT2    "/dev/CM730"
#endif

using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void usage()
{
    printf("Usage : ./Compass <commmand>\n  command :\
           \n\t\t1 : Get Software Version\
           \n\t\t2 : Get Angle 8 Bit\
           \n\t\t3 : Get Angle 16 Bit\
           \n\t\t4 : Get Pitch\
           \n\t\t5 : Get Roll\
           \n\t\t6 : Get Raw Magnetic Data\
           \n\t\t7 : Get Raw Accelerometer Data\
           \n\t\t8 : Get All\
           \n\t\t9 : Process Data\
           \n\t\t10 : Iron Calibration\
           \n\t\t11 : Calibrate\n");
}

int main(int argc, char* argv[])
{
    int cmd;

    if(argc != 2)
    {
        usage(); return 1;
    }
    else
    {
        if(strcmp(argv[1],"1") == 0)        cmd = 1;
        else if(strcmp(argv[1],"2") == 0)   cmd = 2;
        else if(strcmp(argv[1],"3") == 0)   cmd = 3;
        else if(strcmp(argv[1],"4") == 0)   cmd = 4;
        else if(strcmp(argv[1],"5") == 0)   cmd = 5;
        else if(strcmp(argv[1],"6") == 0)   cmd = 6;
        else if(strcmp(argv[1],"7") == 0)   cmd = 7;
        else if(strcmp(argv[1],"8") == 0)   cmd = 8;
        else if(strcmp(argv[1],"9") == 0)   cmd = 9;
        else if(strcmp(argv[1],"10") == 0)   cmd = 10;
        else if(strcmp(argv[1],"11") == 0)   cmd = 11;
        else { usage(); return 1; }
    }

    printf( "\n===== Compass Tutorial for DARwIn =====\n\n");

    change_current_dir();


    minIni* ini = new minIni(INI_FILE_PATH);


    int Ret;                                                                // Used for return values
    Serial *serial = new Serial();

    // Open serial port
    Ret = serial->Open(DEVICE_PORT,9600);                              // Open serial link at 57600 bauds
    if (Ret!=1) {                                                           // If an error occured...
  Ret = serial->Open(DEVICE_PORT2,9600);
  if (Ret != 1)
  {
         printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
           return Ret;                                                         // ... quit the application
  }
    }
    printf ("Serial port opened successfully!\n");

    Compass::GetInstance()->LoadINISettings(ini);


  usleep(100000);

    while(1)
    {

        switch(cmd)
        {
                      case 1 : Compass::GetInstance()->getSoftwareVersion(serial); break;
            case 2 : Compass::GetInstance()->getAngle8Bit(serial);
                    printf("%i\n", Compass::GetInstance()->GetAngle8Bit());

                      break;
            case 3 : Compass::GetInstance()->getAngle16Bit(serial); break;
            case 4 : Compass::GetInstance()->getPitch(serial); break;
            case 5 : Compass::GetInstance()->getRoll(serial); break;
            case 6 : Compass::GetInstance()->getMagneticRawData(serial); break;
            case 7 : Compass::GetInstance()->getAccelerometerRawData(serial); break;
            case 8 : Compass::GetInstance()->getAllData(serial); break;
            case 9 : Compass::GetInstance()->processData(serial); break;
            case 10 : Compass::GetInstance()->IronCalibration(serial,ini); break;
            case 11 : Compass::GetInstance()->Calibration(serial); break;
          /*
            case 1 : Compass::GetInstance()->getSoftwareVersion(serial, serialRecv); break;
            case 2 : Compass::GetInstance()->getAngle8Bit(serial, serialRecv); break;
            case 3 : Compass::GetInstance()->getAngle16Bit(serial, serialRecv); break;
            case 4 : Compass::GetInstance()->getPitch(serial, serialRecv); break;
            case 5 : Compass::GetInstance()->getRoll(serial, serialRecv); break;
            case 6 : Compass::GetInstance()->getMagneticRawData(serial, serialRecv); break;
            case 7 : Compass::GetInstance()->getAccelerometerRawData(serial, serialRecv); break;
            case 8 : Compass::GetInstance()->getAllData(serial, serialRecv); break;
            case 9 : Compass::GetInstance()->Calibration(serial, serialRecv); break;
            */
        }
        usleep(50);

        if (cmd==11 || cmd==10)
          break;
    }
    // Close the connection with the device
    serial->Close();
    delete serial;

    return 0;
}



