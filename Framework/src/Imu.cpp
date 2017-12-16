/*
 * @file : Imu.cpp
 * @autohor: Imre (13210024)
 * This class is used to measuring the angleX and angleY of the robot using the IMU Sensors
 * email : imre.here@yahoo.co.id
 */

#include <stdio.h>
#include <math.h>
#include "Imu.h"
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>



using namespace Robot;

Imu* Imu::m_UniqueInstance = new Imu();

Imu::Imu() :
        m_CM730(0),
        DEBUG_PRINT(false)
{
        for (int i=0; i<6; i++)
            zeroValue[i] = 0;
        

        /* All the angles start at 180 degrees */
        gyroXangle = 180;
        gyroYangle = 180;
        gyroZangle = 180;    

        // Complimentary filter
        compAngleX = 180;
        compAngleY = 180;

        kalmanX.setAngle(180); // Set starting angle
        kalmanY.setAngle(180);
}

Imu::~Imu()
{
}

void Imu::Initialize(CM730 *cm730)
{
    m_CM730 = cm730;
/*
    if(m_CM730->Connect() == false)
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, "Fail to connect CM-730\n");
        return false;
    }
*/
    m_CalibrationStatus = 0;
}

void Imu::CalibrateIMU()
{
    printf("Start IMU Calibration\n");
/*    
    usleep(100000);//wait for the sensor to get ready  unsigned long timer;

    for (int i = 0; i < 100; i++)
    {
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &value, 0) == CM730::SUCCESS)
            zeroValue[0] += (double)value;
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &value, 0) == CM730::SUCCESS)
            zeroValue[1] += (double)value;
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_GYRO_Z_L, &value, 0) == CM730::SUCCESS)
            zeroValue[2] += (double)value;
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &value, 0) == CM730::SUCCESS)
            zeroValue[3] += (double)value;
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &value, 0) == CM730::SUCCESS)
            zeroValue[4] += (double)value;
        if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &value, 0) == CM730::SUCCESS)
            zeroValue[5] += (double)value;
        usleep(10000);//wait for the sensor to get ready
    }

    zeroValue[5] -= 99.2; // Z value is -1g when facing upwards - Sensitivity = 0.33/3.3*1023=102.3
    m_CalibrationStatus = 1;
//    for (int i=0; i<6; i++)
//	printf("%.1f ", zeroValue[i]);
*/
    zeroValue[0] = 512;
    zeroValue[1] =512;
    zeroValue[2] =512;
    zeroValue[3] =512;
    zeroValue[4] =512;
    zeroValue[5] =512;

    printf("\nIMU Calibration Complete \n");
    
    timer = clock();
}

void Imu::ReadIMU()
{
    if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &value, 0) == CM730::SUCCESS){}
      double gyroXrate = -(((double)value - zeroValue[0]) / 0.992); // (gyroXadc-gryoZeroX)/Sensitivity - in quids - Sensitivity = 0.00333/3.3*1023=1.0323
      gyroXangle += gyroXrate * (double)(clock() - timer) / CLOCKS_PER_SEC; // Without any filter

      if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &value, 0) == CM730::SUCCESS){}
      double gyroYrate = -(((double)value - zeroValue[1]) / 0.992);
      gyroYangle += gyroYrate * (double)(clock() - timer) / CLOCKS_PER_SEC;

      /*
      double gyroZrate = -((analogRead(gZ)-zeroValue[2])/1.0323);
      gyroZangle += gyroZrate*((double)(micros()-timer)/1000000);
      Serial.println(gyroZangle); // This is the yaw
      */

      if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &value, 0) == CM730::SUCCESS){}
      double accXval = (double)value - zeroValue[3];
      if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &value, 0) == CM730::SUCCESS){}
      double accYval = (double)value - zeroValue[4];
      if(m_CM730->ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &value, 0) == CM730::SUCCESS){}
      double accZval = (double)value - zeroValue[5];

      // Convert to 360 degrees resolution
      // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
      // We are then convert it to 0 to 2π and then from radians to degrees
      double accXangle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
      double accYangle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;

      if(accZval < 0)//360 degrees
      {
        if(accXangle < 0)
            accXangle = -180-accXangle;
        else
            accXangle = 180-accXangle;
        
        if(accYangle < 0)
            accYangle = -180-accYangle;
        else
            accYangle = 180-accYangle;
      }

      /* You might have to tune the filters to get the best values */
      compAngleX = (0.98 * (compAngleX + (gyroXrate * (double)(clock() - timer) / CLOCKS_PER_SEC))) + (0.02 * (accXangle));
      compAngleY = (0.98 * (compAngleY + (gyroYrate * (double)(clock() - timer) / CLOCKS_PER_SEC))) + (0.02 * (accYangle));
      kalmanAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(clock() - timer) / CLOCKS_PER_SEC);
      kalmanAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(clock() - timer) / CLOCKS_PER_SEC);

      timer = clock(); // reset timing
}

double Imu::GetAngleX()
{
 return 180 - GetKalmanAngleX();
}

double Imu::GetAngleY()
{
 return 180 - GetKalmanAngleY();
}

