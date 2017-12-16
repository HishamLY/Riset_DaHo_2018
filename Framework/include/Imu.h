/*
 * @file : Imu.h
 * @autohor: Imre (13210024)
 * This class is used to measuring the angleX and angleY of the robot using the IMU Sensors
 * email : imre.here@yahoo.co.id
 */

#ifndef _IMU_H_
#define _IMU_H_

#include <list>
#include <fstream>
#include <iostream>
#include "MotionStatus.h"
#include "MotionModule.h"
#include "CM730.h"
#include "Kalman.h"
#include <ctime>
#include <math.h>

#define PI 3.14159265
#define RAD_TO_DEG 57.29577951

namespace Robot
{
	class Imu
	{
	private:
		static Imu* m_UniqueInstance;
		CM730 *m_CM730;
		int m_CalibrationStatus;

		/*
		GFB : Gyro sensor Y axis value   sumbu y robot
		GRL : Gyro sensor X axis value
		AFB : Acceleration sensor Y axis value sumbu y robot
		ARL : Acceleration sensor X axis value
		*/
		double zeroValue[6]; // gyroX, gyroY, gyroZ accX, accY ,accZ

		/* Kalman object contruction */
		Kalman kalmanX;
		Kalman kalmanY;

		/* All the angles start at 180 degrees */
		double gyroXangle ;
		double gyroYangle ;
		double gyroZangle ;	

		// Complimentary filter
		double compAngleX ;
		double compAngleY ;

		// Kalman filter
		double kalmanAngleX;
		double kalmanAngleY;

		int value;

		//time for counting time differences
		clock_t timer;

        Imu();

	protected:

	public:
		bool DEBUG_PRINT;

		~Imu();

		static Imu* GetInstance() { return m_UniqueInstance; }

		void Initialize(CM730 *cm730);

		int GetCalibrationStatus() { return m_CalibrationStatus; }

		//Return measured angle using kalman or complimentary filter.
		double GetKalmanAngleX() {return kalmanAngleX;}
		double GetKalmanAngleY() {return kalmanAngleY;}
		double GetCompAngleX() {return compAngleX;}
		double GetCompAngleY() {return compAngleY;}
		
		//return angle that will be used by Torso Matrix
		double GetAngleX();
		double GetAngleY();

		void CalibrateIMU();
		void ReadIMU();
	};
}

#endif

