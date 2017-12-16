/*
 *   Compass.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _COMPASS_H_
#define _COMPASS_H_

#include <string.h>
#include "minIni.h"
#include "Serial.h"
#include <stdio.h>
#include <vector>

#define COMPASS_SECTION    "Compass"
#define INVALID_VALUE   -1024.0

#define SOFTWARE_VERSION 	0x11
#define ANGLE_8_BIT 		0x12
#define ANGLE_16_BIT		0x13
#define PITCH			0x16
#define ROLL 			0x17
#define MAGNETIC_DATA 		0x19
#define ACCELEROMETER_DATA	0x20
#define ALL_DATA		0x23
#define CALIBRATE_EN1		0x31
#define CALIBRATE_EN2		0x45
#define CALIBRATE_EN3		0x5A
#define CALIBRATE		0x5E
#define RESTORE1		0x6A
#define RESTORE2		0x7C
#define RESTORE3		0x81
#define BAUD19200		0xA0
#define BAUD38400		0xA1

#define X_var 0
#define Y_var 1

using namespace std;

namespace Robot
{
	class Compass
	{
	private:
		static Compass* m_UniqueInstance;
		Compass();

		int get8bit;
		double Angle16Bit;
		char *serialRecv;
		Serial* seral;

		float accX,accY, accZ;
		float magX, magY, magZ;
		float pitch, roll;

		int delX, delY;
		float r,q,sigma,angle;

		float sum, av0, av1;
		int averageCounter,counter;
		bool atas;

		float heading;
		unsigned char heading8bit;
		

	public:


		static Compass* GetInstance() { return m_UniqueInstance; }

		~Compass();

		void getSoftwareVersion(Serial *serial);
		void Calibration(Serial *serial);
		void getAngle8Bit(Serial *serial);
		void getAngle16Bit(Serial *serial);
		void getPitch(Serial *serial);
		void getRoll(Serial *serial);
		void getMagneticRawData(Serial *serial);
		void getAccelerometerRawData(Serial *serial);
		void getAllData(Serial *serial);
		void processData(Serial *serial);
		void IronCalibration(Serial *serial,minIni* ini);

		int GetAngle8Bit(){return get8bit;}
		double GetAngle16Bit() { return Angle16Bit;}
		double GetCompassOrientation() { return heading;}
		unsigned char GetCompassOrientation8Bit() { return heading8bit; }

		int getAccX(){return accX;}
		int getAccY(){return accY;}
		int getAccZ(){return accZ;}
		int getMgX(){return magX;}
		int getMgY(){return magY;}
		int getMgZ(){return magZ;}

 		void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif

