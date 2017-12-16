/**
* @file RobotCameraMatrixProvider.cpp
* This file implements a class to calculate the position of the camera relative to the body for the Darwin.
* @author Colin Graf
*/

#include "RobotCameraMatrixProvider.h"

using namespace Robot;

void RobotCameraMatrixProvider::update(RobotCameraMatrixPrev& robotCameraMatrixPrev,  CameraCalibration& theCameraCalibration)
{

  robotCameraMatrixPrev.computeRobotCameraMatrix(theRobotDimensions,
                                                 MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_PAN),
                                                 MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_TILT),
                                                 theCameraCalibration, false);
}

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix,   CameraCalibration& theCameraCalibration)
{
 robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions,
                                                 MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_PAN),
                                                 MotionStatus::m_CurrentJoints.GetRadian(JointData::ID_HEAD_TILT),
                                                theCameraCalibration, false);
}
