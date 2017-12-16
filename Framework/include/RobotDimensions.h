    /**
 * @file RobotDimensions.h
 * Description of the Dimensions of the DarwinOP
 * @author Imre Nagi
 */

#ifndef __ROBOTDIMENSIONS_H__
#define __ROBOTDIMENSIONS_H__

#include "Vector3.h"

namespace Robot
{
class RobotDimensions
{
public:
  RobotDimensions() :
    lengthBetweenLegs(74), //OKE 50+50 darwin 74mm (coklat = darwin)
    upperLegLength(93),    //OKE 93.0 (coklat = darwin)
    lowerLegLength(93.0f), //OKE 93 (coklat = darwin)
    heightLeg5Joint(33.5f),  //OKE coklat 32 mm + 2 mm akrilik darwin 33.5 mm
    zLegJoint1ToHeadPan(143.5f), //OKE 85mm+126.5 143.5 mm

    xHeadTiltToCamera(51.77f), //33.2
    zHeadTiltToCamera(7.75f),
    headTiltToCameraTilt(0.8722f),
    //batas
    xHeadTiltToUpperCamera(33.2f),   //darwin 33.2mm
    zHeadTiltToUpperCamera(34.4f),  //darwin 34.4mm
    headTiltToUpperCameraTilt(0.0f), //darwin 0mm

    armOffset(5,82,62), //didimensi robot (0,98,100)  darwin (0,82,62)
    upperArmLength(60), //OKE  coklat 57.5 mm darwin 60mm
    lowerArmLength(129.0f), //OKE coklat 117 mm darwin 129mm
    motionCycleTime(0.010f),
    imageRecordingTime(0.034f),
    imageRecordingDelay(-0.003f)
  {}

  float lengthBetweenLegs;         //!<length between leg joints LL1 and LR1
  float upperLegLength;            //!<length between leg joints LL2 and LL3 in z-direction
  float lowerLegLength;            //!<length between leg joints LL3 and LL4 in z-direction
  float heightLeg5Joint;           //!<height of leg joints LL4 and LR4 of the ground

  float zLegJoint1ToHeadPan;       //!<height offset between LL1 and head pan joint

  float xHeadTiltToCamera;         //!<forward offset between head tilt joint and lower camera
  float zHeadTiltToCamera;         //!<height offset between head tilt joint lower and
  float headTiltToCameraTilt;      //!<tilt of lower camera against head tilt

  float xHeadTiltToUpperCamera;    //!<forward offset between head tilt joint and upper camera
  float zHeadTiltToUpperCamera;    //!<height offset between head tilt joint upper and
  float headTiltToUpperCameraTilt; //!<tilt of upper camera against head tilt

  Vector3<> armOffset;             //! The offset of the first left arm joint relative to the middle between the hip joints
  float upperArmLength;            //! The length between the shoulder and the elbow in y-direction
  float lowerArmLength;            //!< height off lower arm starting at arm2/arm3

  float motionCycleTime;           //! Length of one motion cycle in seconds.
  float imageRecordingTime;        //! Time the camera requires to take an image (in s, for motion compensation, may depend on exposure).
  float imageRecordingDelay;       //! Delay after the camera took an image (in s, for motion compensation).
};
}
#endif
