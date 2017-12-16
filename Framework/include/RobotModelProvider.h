/**
* @file RobotModelProvider.h
*
* This file declares a module that provides information about the current state of the robot's limbs.
*
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef __RobotModelProvider_h_
#define __RobotModelProvider_h_

//#include "Tools/Module/Module.h"
#include "RobotModel.h"
#include "RobotDimensions.h"
#include "MassCalibration.h"
//#include "Representations/Infrastructure/JointData.h"
#include "JointData.h"

namespace Robot
{
/**
* @class RobotModelProvider
*
* A module for computing the current state of the robot's limbs
*/
//class RobotModelProvider: public RobotModelProviderBase
class RobotModelProvider
{
private:
  /** Executes this module
  * @param robotModel The data structure that is filled by this module
  */

public :
    void update(RobotModel& robotModel);

    RobotDimensions theRobotDimensions;
    MassCalibration theMassCalibration;
};
}
#endif// __RobotModelProvider_h_
