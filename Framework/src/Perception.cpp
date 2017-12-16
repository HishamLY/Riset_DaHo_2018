
/*******************************************************************************
*
* File : Perception.cpp
* @author : Imre Nagi (imre.here@yahoo.co.id)
*           Institut Teknologi Bandung
*
*
*
********************************************************************************/

#include "Perception.h"
#include <stdlib.h>
#include <stdio.h>

using namespace Robot;
//using namespace std;

vector<Sensor> Perception::SensorsResult;
Perception* Perception::m_UniqueInstance = new Perception();

Perception::Perception()
{
  ReadCameraOrMap = 1;
}

Perception::~Perception()
{
}

void Perception::pushLandmark(Vector2<> pf, int typeLandmark)
{
    Sensor temp;
    temp.jarak = sqrt(pf.x/10*pf.x/10 + pf.y/10*pf.y/10);
    temp.arah = MATH::getAngleToPoint(pf.x, pf.y) ; //sudut antara robot dan landmark
    if (temp.arah > 180)
        temp.arah -= 360;
    temp.n = typeLandmark;
    //printf("push %.1f,%.1f,%i \n", temp.jarak, temp.arah, temp.n);
    SensorsResult.push_back(temp);
}

void Perception::ResetPercept()
{
    SensorsResult.clear();
}

void Perception::update(CameraMatrix& thecameramatrix)
{
  Vector2<> tempPoint(0,0);
  ResetPercept();
  if (GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->BOTH_POST)
  {
      if (GoalPercept::GetInstance()->Owner == GoalPercept::OWN_GOAL)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->LeftFoot .X, (int)GoalPercept::GetInstance()->LeftFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, LeftOwnPost);
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->RightFoot .X, (int)GoalPercept::GetInstance()->RightFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, RightOwnPost);
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->CenterFoot .X, (int)GoalPercept::GetInstance()->CenterFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, CenterOwnPost);
      }  
      else if (GoalPercept::GetInstance()->Owner == GoalPercept::OPPONENT_GOAL || GoalPercept::GetInstance()->Owner == GoalPercept::UNCLEAR)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->LeftFoot .X, (int)GoalPercept::GetInstance()->LeftFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, LeftOppPost);
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->RightFoot .X, (int)GoalPercept::GetInstance()->RightFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, RightOppPost);
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->CenterFoot .X, (int)GoalPercept::GetInstance()->CenterFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, CenterOppPost);
      }
  }
  else if (GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->RIGHT_POST || GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->POSSIBLE_RIGHT_POST)
  {
      if (GoalPercept::GetInstance()->Owner == GoalPercept::OWN_GOAL)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->RightFoot .X, (int)GoalPercept::GetInstance()->RightFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, RightOwnPost);
      }  
      else if (GoalPercept::GetInstance()->Owner == GoalPercept::OPPONENT_GOAL || GoalPercept::GetInstance()->Owner == GoalPercept::UNCLEAR)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->RightFoot .X, (int)GoalPercept::GetInstance()->RightFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, RightOppPost);
      }
  }
  else if (GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->LEFT_POST || GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->POSSIBLE_LEFT_POST)
  {
      if (GoalPercept::GetInstance()->Owner == GoalPercept::OWN_GOAL)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->LeftFoot .X, (int)GoalPercept::GetInstance()->LeftFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, LeftOwnPost);
      }  
      else if (GoalPercept::GetInstance()->Owner == GoalPercept::OPPONENT_GOAL || GoalPercept::GetInstance()->Owner == GoalPercept::UNCLEAR)
      {
        Geometry::calculatePointOnField((int)GoalPercept::GetInstance()->LeftFoot .X, (int)GoalPercept::GetInstance()->LeftFoot.Y, thecameramatrix, tempPoint);   
        pushLandmark(tempPoint, LeftOppPost);
      }
  }
  else if (GoalPercept::GetInstance()->Status == GoalPercept::GetInstance()->UNKNOWN_POST)
  {
      ResetPercept();
  }
}

bool Perception::isOppGoal(unsigned int presentval, unsigned int setpoint, double headPan)
{
int offset;
    double angle= 0, bodyOrientation=0;
  
    if (presentval<=setpoint)
       {
 	   if (presentval-setpoint<-128)
                offset = 256 - setpoint + presentval;
           else
                offset = presentval-setpoint;
       }
    else
       {
         if (presentval-setpoint<128)
              offset = presentval-setpoint;
          else
              offset = presentval - 256 - setpoint;
       }
    
    bodyOrientation = (double)offset/128.0*180.0;
    bodyOrientation *= -1;
    if (bodyOrientation <0 && bodyOrientation >= -180)
        bodyOrientation += 360;
 
    angle = bodyOrientation + headPan;
    if (angle < 0)
      angle += 360;
    else if (angle > 360)
      angle -= 360;

    if (angle <=90 || angle >= 270)
      fprintf(stderr, "OPP - presentval = %i sp = %i angle = %.2f, bodyOrientation = %.2f , headPan = %.2f \r",presentval, setpoint, angle, bodyOrientation, headPan);
    else if (angle > 90 && angle < 270)
      fprintf(stderr, "OWN - presentval = %i sp = %i angle = %.2f, bodyOrientation = %.2f , headPan = %.2f \r",presentval, setpoint, angle, bodyOrientation, headPan);

    if (angle <=90 || angle >= 270)
      return true; //gawang lawan
    else if (angle > 90 && angle < 270)
      return false; //gawang sendiri
    
}
