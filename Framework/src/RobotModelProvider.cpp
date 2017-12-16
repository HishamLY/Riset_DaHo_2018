/**
* @file RobotModelProvider.cpp
*
* This file implements a module that provides information about the current state of the robot's limbs.
*
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#include "RobotModelProvider.h"
#include "DebugDrawings.h"
#include <stdio.h>
//#include "Tools/Debugging/DebugDrawings3D.h"

using namespace Robot;

void RobotModelProvider::update(RobotModel &robotModel)
{
  robotModel.setJointData(theRobotDimensions, theMassCalibration);
}

