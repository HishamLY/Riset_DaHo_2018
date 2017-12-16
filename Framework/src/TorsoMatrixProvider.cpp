/**
* @file TorsoMatrixProvider.cpp
* Implementation of module TorsoMatrixProvider. (Edited for implementation in Darwin-OP)
* @author Colin Graf
* @author Imre Nagi
*/

#include "TorsoMatrixProvider.h"
#include "DebugDrawings.h"
#include <stdio.h>

using namespace Robot;

void TorsoMatrixProvider::update(TorsoMatrixPrev& torsoMatrixPrev, RobotModel& theRobotModel)
{
  (TorsoMatrix&) torsoMatrixPrev = theTorsoMatrix;
}

void TorsoMatrixProvider::update(TorsoMatrix& torsoMatrix, RobotModel& theRobotModel, double angleX, double angleY)
{
  //generate rotation matrix from measured angleX and angleY
  //const Vector3<> axis((float) theFilteredSensorData.data[SensorData::angleX], (float) theFilteredSensorData.data[SensorData::angleY], 0);
  const Vector3<> axis((float) MATH::DegreesToRadians(angleX), (float) MATH::DegreesToRadians(angleY) , 0);


  RotationMatrix torsoRotation(axis, axis.abs()); //torsoRotation variable declaration. see RotationMatrix.

  // calculate "center of hip" position from left foot
  Pose3D fromLeftFoot(torsoRotation);
  fromLeftFoot.conc(theRobotModel.limbs[RobotModel::mp_foot_l]);
  fromLeftFoot.translate(0, 0, (float) -theRobotDimensions.heightLeg5Joint);
  fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoRotation;

  // calculate "center of hip" position from right foot
  Pose3D fromRightFoot(torsoRotation);
  fromRightFoot.conc(theRobotModel.limbs[RobotModel::mp_foot_r]);
  fromRightFoot.translate(0, 0, (float) -theRobotDimensions.heightLeg5Joint);
  fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoRotation;

  // get foot z-rotations
  const Pose3D& leftFootInverse(theRobotModel.limbs[RobotModel::mp_foot_l].invert());
  const Pose3D& rightFootInverse(theRobotModel.limbs[RobotModel::mp_foot_r].invert());
  const float leftFootZRotation = leftFootInverse.rotation.getZAngle();
  const float rightFootZRotation = rightFootInverse.rotation.getZAngle();

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z > fromRightFoot.translation.z;

  // calculate foot span
  const Vector3<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3D newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  // calculate torso offset
  if(torsoMatrix.translation.z != 0) // the last torso matrix should be valid
  {
    Pose3D& torsoOffset(torsoMatrix.offset);
    torsoOffset = torsoMatrix.invert();
    torsoOffset.translate(lastFootSpan.x / (useLeft ? 2.f : -2.f), lastFootSpan.y / (useLeft ? 2.f : -2.f), 0);
    torsoOffset.rotateZ(useLeft ? float(leftFootZRotation - lastLeftFootZRotation) : float(rightFootZRotation - lastRightFootZRotation));
    torsoOffset.translate(newFootSpan.x / (useLeft ? -2.f : 2.f), newFootSpan.y / (useLeft ? -2.f : 2.f), 0);
    torsoOffset.conc(newTorsoMatrix);
  }

  // adopt new matrix and footSpan
  (Pose3D&)torsoMatrix = newTorsoMatrix;
  lastLeftFootZRotation = leftFootZRotation;
  lastRightFootZRotation = rightFootZRotation;
  lastFootSpan = newFootSpan;

  // valid?
  //torsoMatrix.isValid = theGroundContactState.contact;
  torsoMatrix.isValid = true;

}

void TorsoMatrixProvider::update(OdometryData& odometryData)
{
  Pose2D odometryOffset;

  if(lastTorsoMatrix.translation.z != 0.)
  {
    Pose3D odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.invert());

    odometryOffset.translation.x = odometryOffset3D.translation.x;
    odometryOffset.translation.y = odometryOffset3D.translation.y;
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

  odometryData += odometryOffset;

  (Pose3D&)lastTorsoMatrix = theTorsoMatrix;
}

