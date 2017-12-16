/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
*/

#ifndef __MASSCALIBRATION_H__
#define __MASSCALIBRATION_H__

#include "Vector3.h"

namespace Robot
{
class MassCalibration
{
public:
  enum Limb
  {
    mp_neck, //0
    mp_head,
    mp_shoulder_l,
    mp_arm_upper_l,
    mp_arm_lower_l,
    mp_shoulder_r,
    mp_arm_upper_r,
    mp_arm_lower_r,
    mp_pelv_l,
    mp_pelv_y_l,
    mp_thight_l,
    mp_tibia_l,
    mp_ankle_l,
    mp_foot_l,
    mp_pelv_r,
    mp_pelv_y_r,
    mp_thight_r,
    mp_tibia_r,
    mp_ankle_r,
    mp_foot_r,
    mp_body,
    numOfLimbs
  };

  /** Information on the mass distribution of a limb of the robot.*/
  class MassInfo
  {
  private:

  public:
    float mass; /**< The mass of this limb. */
    Vector3<> offset; /**< The offset of the center of mass of this limb relative to its hinge. */

    /** Default constructor. */
    MassInfo() : mass(0), offset(0,0,0) {}
  };

  MassInfo masses[numOfLimbs]; /**< Information on the mass distribution of all joints. */

     /**
  * Default constructor.
  */
  MassCalibration()
  {
    masses[0].mass = 24.357;    masses[0].offset = Vector3<>(-71.281, 1.424     , -16.567); //x y z
    masses[1].mass = 158.0419;  masses[1].offset = Vector3<>(7.66   , 0.00639   , 18.564);
    masses[2].mass = 25.913;    masses[2].offset = Vector3<>(1.393  , -13.522   , 10.264);
    masses[3].mass = 168.377;   masses[3].offset = Vector3<>(0.734  , 0.659     , -36.23);
    masses[4].mass = 59.288;    masses[4].offset = Vector3<>(-13.49 , 6.66      , -45.83);
    masses[5].mass = 25.913;    masses[5].offset = Vector3<>(1.393  , 13.522    , 10.264);
    masses[6].mass = 59.288;    masses[6].offset = Vector3<>(0.734  , -0.659    , -36.23);
    masses[7].mass = 168.377;   masses[7].offset = Vector3<>(-13.49 , -6.66     , -45.83 );
    masses[8].mass = 167.107;   masses[8].offset = Vector3<>(-18.242, 0.079     , -13.873);
    masses[9].mass = 27.069;    masses[9].offset = Vector3<>(0.48   , 0         , 18.437);
    masses[10].mass = 119.0433; masses[10].offset = Vector3<>(0.6919, -0.3226   , -62.9655);
    masses[11].mass = 70.309;   masses[11].offset = Vector3<>(6.5476, -0.592    , 53.954);
    masses[12].mass = 167.107;  masses[12].offset = Vector3<>(-18.53, -0.213    , 13.87);
    masses[13].mass = 79.44;    masses[13].offset = Vector3<>(-0.5028   ,9.505  , -25.995);
    masses[14].mass = 167.107;  masses[14].offset = Vector3<>(-18.242   ,-0.079 , -13.873);
    masses[15].mass = 27.069;   masses[15].offset = Vector3<>(0.480     ,0      , 18.437);
    masses[16].mass = 119.0433; masses[16].offset = Vector3<>(0.6919    ,0.3226 , -62.9655);
    masses[17].mass = 70.309;   masses[17].offset = Vector3<>(6.54      ,0.592  , 53.954);
    masses[18].mass = 167.107;  masses[18].offset = Vector3<>(-18.53    ,0.213  , 13.87);
    masses[19].mass = 79.44;    masses[19].offset = Vector3<>(-0.5028   ,-9.505 , -25.995);
    masses[20].mass = 975.599;  masses[20].offset = Vector3<>(-19.66    ,-3.115 , -39.44);
  }

};
}
#endif
