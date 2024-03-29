/**
* @file OdometryData.h
* Contains the OdometryData class.
* @author Max Risler
*/

#ifndef _ODOMETRYDATA_H_
#define _ODOMETRYDATA_H_

#include "Pose2D.h"

namespace Robot{
/**
* @class OdometryData
* OdometryData contains an approximation of overall movement the robot has done.
* @attention Only use differences of OdometryData at different times.
* Position in mm
*/
class OdometryData : public Pose2D {};

/**
* @class GroundTruthOdometryData
* Contains an observed overall movement the robot has done.
*/
class GroundTruthOdometryData : public OdometryData {};
}
#endif // OdometryData_H
