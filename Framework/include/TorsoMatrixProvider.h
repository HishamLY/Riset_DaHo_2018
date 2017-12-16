/**
* @file TorsoMatrixProvider.h
* Declaration of module TorsoMatrixProvider.
* @author Colin Graf
*/

#ifndef _TORSOMATRIXPROVIDER_H_
#define _TORSOMATRIXPROVIDER_H_

//#include "Tools/Module/Module.h"
//#include "SensorData.h" //BELUM DISET SENSOR2NYA UTK DARWIN
#include "RobotModel.h"
#include "OdometryData.h"
#include "TorsoMatrix.h"
#include "GroundContactState.h"
#include "RobotDimensions.h"

namespace Robot{
/**
* @class TorsoMatrixProvider
* A module that provides the (estimated) position and velocity of the inertia board.
*/
//class TorsoMatrixProvider : public TorsoMatrixProviderBase
class TorsoMatrixProvider
{
public:
 //-----------------additional---------------//
  RobotDimensions theRobotDimensions; //oke
  TorsoMatrix theTorsoMatrix;

    /** Updates the TorsoMatrixPrev representation.
  * @param torsoMatrixPrev The inertia matrix as it was computed in the previous execution of this module.
  */
  void update(TorsoMatrixPrev& torsoMatrixPrev, RobotModel& theRobotModel);
  void update(TorsoMatrix& torsoMatrix, RobotModel& theRobotModel, double angleX, double angleY);


  /** Updates the TorsoMatrix representation.
  * @param torsoMatrix The inertia matrix representation which is updated by this module.
  */
  void update(TorsoMatrix& torsoMatrix, RobotModel& theRobotModel);

  /** Updates the OdometryData representation.
  * @param odometryData The odometry data representation which is updated by this module.
  */
  void update(OdometryData& odometryData);

private:
  float lastLeftFootZRotation; /**< The last z-rotation of the left foot. */
  float lastRightFootZRotation; /**< The last z-rotation of the right foot. */

  Vector3<> lastFootSpan; /**< The last span between both feet. */
  Pose3D lastTorsoMatrix; /**< The last torso matrix for calculating the odometry offset. */
};
}
#endif
