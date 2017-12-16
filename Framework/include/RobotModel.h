/**
* @file RobotModel.h
* Declaration of class RobotModel
*/

#ifndef _ROBOTMODEL_H_
#define _ROBOTMODEL_H_

#include "Pose3D.h"
#include "JointData.h"  //Jointdata ini masih punya  nao
#include "RobotDimensions.h"
#include "MassCalibration.h"
#include "Common.h"

namespace Robot
{
/**
 * @class RobotModel
 * Contains information about extremities.
 */
class RobotModel
{
public:
  enum Limb
  {
    mp_shoulder_r       = 1,
    mp_shoulder_l       = 2,
    mp_arm_upper_r      = 3,
    mp_arm_upper_l      = 4,
    mp_arm_lower_r      = 5,
    mp_arm_lower_l      = 6,
    mp_pelv_y_r         = 7,
    mp_pelv_y_l         = 8,
    mp_pelv_r           = 9,
    mp_pelv_l           = 10,
    mp_thigh_r          = 11,
    mp_thigh_l          = 12,
    mp_tibia_r          = 13,
    mp_tibia_l          = 14,
    mp_ankle_r          = 15,
    mp_ankle_l          = 16,
    mp_foot_r           = 17,
    mp_foot_l           = 18,
    mp_neck             = 19,
    mp_head             = 20,
    numOfLimbs
  };

  JointData m_Joint;

  Pose3D limbs[numOfLimbs]; /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
  Vector3<> centerOfMass; /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
  float totalMass; /**< The mass of the robot. */

  /** Constructor */
  RobotModel() : totalMass(0) {}

  /**
  * Constructs the RobotModel from given joint data.
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
  //RobotModel(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);
  RobotModel(const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /**
  * Recalculates the RobotModel from given joint data.
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
  void setJointData(const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /** Creates a 3-D drawing of the robot model. */
  void draw();

  void draw_coordinates(int a);
};
}
#endif //RobotModel_H
