/**
* @file RobotModel.cpp
* Implementation of class RobotModel.
* @author Alexander Härtl
* @author Imre
*/

#include "RobotModel.h"
#include "DebugDrawings.h"
#include <stdio.h>
#include "MotionStatus.h"

using namespace Robot;

RobotModel::RobotModel(const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  setJointData(robotDimensions, massCalibration);
}

void RobotModel::draw_coordinates(int a)
{
  Pose3D& p = limbs[a];
  Vector3<>& v = p.translation;
  Vector3<> v1 = p * Vector3<>(200, 0, 0);
  Vector3<> v2 = p * Vector3<>(0, 200, 0);
  Vector3<> v3 = p * Vector3<>(0, 0, 200);
  printf("(%f,%f,%f) - (%f,%f,%f)\n", v.x, v.y, v.z, v1.x, v1.y, v1.z);
  printf("(%f,%f,%f) - (%f,%f,%f)\n", v.x, v.y, v.z, v2.x, v2.y, v2.z);
  printf("(%f,%f,%f) - (%f,%f,%f)\n", v.x, v.y, v.z, v3.x, v3.y, v3.z);
}

void RobotModel::setJointData(const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
/*
  float angle[21];
  angle[1]=-0.8484628889;
  angle[2]=0.7206823333;
  angle[3]=-0.3117845556;
  angle[4]=0.3066733333;
  angle[5]=0.506011;
  angle[6]=-0.5162334444;
  angle[7]=0;
  angle[8]=0;
  angle[9]=0.0051112222;
  angle[10]=-0.0102224444;
  angle[11]=-0.6337915556;
  angle[12]=0.6286803333;
  angle[13]=0.9251312222;
  angle[14]=-0.9302424444;
  angle[15]=0.5213446667;
  angle[16]=-0.5264558889;
  angle[17]=0.0102224444;
  angle[18]=-0.0153336667;
  angle[19]=0;
  angle[20]=0.1686703333;
*/

// compute unilateral limbs
  limbs[mp_neck] = Pose3D(0, 0, robotDimensions.zLegJoint1ToHeadPan)
                 .rotateZ(MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_PAN));  //checked
  //.rotateZ(angle[JointData::ID_HEAD_PAN]);

  limbs[mp_head] = Pose3D(limbs[mp_neck])
                   .translate(0, 0, 30.5)
                   .rotateY(-MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_TILT)); //checked
  //.rotateY(-angle[JointData::ID_HEAD_TILT]); //checked

  // compute bilateral limbs
  for(int side = 0; side < 2; side++)
    {
      bool left = side == 0;
      int sign = left ? -1 : 1;
      Limb pelvis = left ? mp_pelv_y_l : mp_pelv_y_r;
      int leg0 = left ? JointData::ID_L_HIP_YAW : JointData::ID_R_HIP_YAW;
      Limb shoulder = left ? mp_shoulder_l : mp_shoulder_r;
      int arm0 = left ? JointData::ID_L_SHOULDER_PITCH : JointData::ID_R_SHOULDER_PITCH;

      limbs[shoulder + 0] = Pose3D(robotDimensions.armOffset.x, robotDimensions.armOffset.y * -sign, robotDimensions.armOffset.z)
                            //.rotateY(-MotionStatus::m_CurrentJoints.GetRadian(arm0 + 0));  //unchecked
                            .rotateY(0);  //shoulder pitch rotasi di sumbu Y
      limbs[shoulder + 2] = Pose3D(limbs[shoulder + 0])
                            //.rotateX((MotionStatus::m_CurrentJoints.GetRadian(arm0 + 2) + pi_4) * -sign); //shoulder roll . perhatikan posisi robot saat axis definition //Z
                            .rotateX((0 + pi_4) * -sign); //shoulder roll . perhatikan posisi robot saat axis definition //Z
      limbs[shoulder + 4] = Pose3D(limbs[shoulder + 2])
                            .translate(robotDimensions.upperArmLength, 0, 0)
                            //.rotateY(MotionStatus::m_CurrentJoints.GetRadian(arm0 + 4) * -sign); //elbow rotasi disumbu x //X
                            .rotateY(0 * -sign); //elbow rotasi disumbu x //X

      //hip yaw
      limbs[pelvis + 0] =  Pose3D(0, robotDimensions.lengthBetweenLegs / 2.0f * -sign, 0)
                           .rotateZ(-MotionStatus::m_CurrentJoints.GetRadian(leg0 + 0)); //checked
      //.rotateZ(-angle[leg0 + 0]);

      //hip roll
      limbs[pelvis + 2] = Pose3D(limbs[pelvis + 0])
                          .rotateX(MotionStatus::m_CurrentJoints.GetRadian(leg0 + 2)); //checked
      //.rotateX(angle[leg0 + 2]);

      //hip pitch
      limbs[pelvis + 4] = Pose3D(limbs[pelvis + 2])
                          .rotateY(MotionStatus::m_CurrentJoints.GetRadian(leg0 + 4) * sign);//checked
      //.rotateY(angle[leg0 + 4] * sign);

      //lutut
      limbs[pelvis + 6] = Pose3D(limbs[pelvis + 4])
                          .translate(0, 0, -robotDimensions.upperLegLength)
                          .rotateY(sign*MotionStatus::m_CurrentJoints.GetRadian(leg0 + 6));  //checked
      //.rotateY(sign*angle[leg0 + 6]);

      //ankle pitch
      limbs[pelvis + 8] = Pose3D(limbs[pelvis + 6])
                          .translate(0, 0, -robotDimensions.lowerLegLength)
                          .rotateY(sign*MotionStatus::m_CurrentJoints.GetRadian(leg0 + 8));  //checked
      //.rotateY(sign*angle[leg0 + 8]);

      //ankle roll
      limbs[pelvis + 10] = Pose3D(limbs[pelvis + 8])
                           .rotateX(MotionStatus::m_CurrentJoints.GetRadian(leg0 + 10));
      //.rotateX(angle[leg0 + 10]);
    }
  /*
    printf("mp_neck\n");
    draw_coordinates(mp_neck);
    printf("mp_head\n");
    draw_coordinates(mp_head);
    printf("\n");
    printf("mp_pelv_y_l\n");
    draw_coordinates(mp_pelv_y_l);
    printf("mp_pelv_y_r\n");
    draw_coordinates(mp_pelv_y_r);

    printf("\n");
    printf("m_pelv_l\n");
    draw_coordinates(mp_pelv_l);
    printf("m_pelv_r\n");
    draw_coordinates(mp_pelv_r);

    printf("\n");
    printf("mp_thigh_l\n");
    draw_coordinates(mp_thigh_l);
    printf("mp_thigh_r\n");
    draw_coordinates(mp_thigh_r);

    printf("\n");
    printf("mp_tibia_l\n");
    draw_coordinates(mp_tibia_l);
    printf("mp_tibia_r\n");
    draw_coordinates(mp_tibia_r);

    printf("\n");
    printf("mp_ankle_l\n");
    draw_coordinates(mp_ankle_l);
    printf("mp_ankle_r\n");
    draw_coordinates(mp_ankle_r);

    printf("\n");
    printf("mp_foot_l\n");
    draw_coordinates(mp_foot_l);
    printf("mp_foot_r\n");
    draw_coordinates(mp_foot_r);
  */

  // calculate center of mass
  //const MassCalibration::MassInfo& torso(MassCalibration.masses[MassCalibration::mp_body]);
  const MassCalibration::MassInfo& torso(massCalibration.masses[massCalibration.mp_body]);

  // initialize accumulators
  centerOfMass = torso.offset * torso.mass;
  totalMass = torso.mass;
  for(int i = 1; i < numOfLimbs; i++)
    {
      const MassCalibration::MassInfo& limb(massCalibration.masses[i]);
      totalMass += limb.mass;
      centerOfMass += (limbs[i] * limb.offset) * limb.mass;
    }
  centerOfMass /= totalMass;
}

void RobotModel::draw()
{}

