/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Head.h"
#include "Camera.h"
#include "ImgProcess.h"
#include "BallTracker.h"
#include "Behavior.h"

using namespace Robot;

BallTracker::BallTracker() :
        ball_position(Point2D(-1.0, -1.0))
{
	NoBallCount = 0;

	check = false;
}

BallTracker::~BallTracker()
{
}

void BallTracker::Process(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->GetLastPanAngle();
			Head::GetInstance()->MoveTrackingFar();
			NoBallCount++;
			check = true;
		}
		else
		{
			if(check)
			{
				if(Head::GetInstance()->CheckLastPanAngle() == RIGHT )
				{
					Head::GetInstance()->initSearching(false, true, false, false);
				}
				else if(Head::GetInstance()->CheckLastPanAngle() == LEFT )
				{
					Head::GetInstance()->initSearching(false, false, false, true);
				}
				check = false;
			}
			else
			{
				if(abs((int)Walking::GetInstance()->A_MOVE_AMPLITUDE) < 20)
					Head::GetInstance()->MoveSearching();
				else if(abs((int)Walking::GetInstance()->A_MOVE_AMPLITUDE) >= 20)
					Head::GetInstance()->MoveSearchingTurn();
			}
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

extern double panLastSeenPost;

void BallTracker::ProcessHighResolution(Point2D pos, bool *IsOneCycle)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(*IsOneCycle == false)
		{
			if(NoBallCount < 20)
			{
				NoBallCount++;
				Head::GetInstance()->MoveByAngle(50, 30);
			}
			else if(NoBallCount < 40)
			{
				NoBallCount++;
				Head::GetInstance()->MoveByAngle(0, 30);
			}
			else if(NoBallCount < 60)
			{
				NoBallCount++;
				Head::GetInstance()->MoveByAngle(-50, 30);
			}
			else
			{
				NoBallCount = 0;
				*IsOneCycle = true;
			}
		}
		else
		{
			NoBallCount = 0;
			Head::GetInstance()->MoveByAngle(0, 10);
		}
	}
	else 
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2); //Center resolusi gede
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // angle per pixel //dikonversi ke sudut karena sudut bola resolusi kecil dan besar sama
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // angle per pixel //dikonversi ke sudut karena sudut bola resolusi kecil dan besar sama
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

void BallTracker::ProcessUpHead(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
                ball_position.Y = -1;
		Head::GetInstance()->MoveUp(5);
		NoBallCount = 0;
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}

}

void BallTracker::ProcessCheckPostLeft(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
                ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount/3)
		{
			NoBallCount++;
		}
		else
		{
			Head::GetInstance()->MoveSearchingPostLeft();
			NoBallCount = 0;
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

void BallTracker::ProcessCheckPostRight(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
                ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount/3)
		{
			NoBallCount++;
		}
		else
		{
			Head::GetInstance()->MoveSearchingPostRight();
			NoBallCount = 0;
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

void BallTracker::ProcessCheckPostAhead(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
                ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount/3)
		{
			NoBallCount++;
		}
		else
		{
			Head::GetInstance()->MoveSearchingPost();
			NoBallCount = 0;
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

void BallTracker::ProcessCheckPost(Point2D pos, int CompassError)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		Head::GetInstance()->SetLimit(CompassError);
		ball_position.X = -1;
                ball_position.Y = -1;
		Head::GetInstance()->MoveSearchingPost();
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}

void BallTracker::ProcessCheckPost(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
                ball_position.Y = -1;
		Head::GetInstance()->MoveSearchingPost();
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
}


void BallTracker::ProcessToKick(Point2D pos)
{
        if(pos.X < 0 || pos.Y < 0)
        {
		ball_position.X = -1;
                ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
                {
                        NoBallCount++;
                }
                else
                {
			Head::GetInstance()->MoveSearching();
		}
	}
        else
        {
		NoBallCount = 0;
                Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
                Point2D offset = pos - center;
                offset *= -1; // Inverse X-axis, Y-axis
                offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
                offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
                ball_position = offset;
                Head::GetInstance()->MoveTracking(ball_position);
        }
}

void BallTracker::ProcessKick(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
		{
			NoBallCount++;
		}
		else
		{
			Head::GetInstance()->MoveSearchingPenaltyKick();
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTracking(ball_position);
	}
}

void BallTracker::ProcessLocalizePenaltyMark(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->GetLastPanAngle();
			Head::GetInstance()->MoveTrackingFar();
			NoBallCount++;
			check = true;
		}
		else
		{
			if(check)
			{
				if(Head::GetInstance()->CheckLastPanAngle() == RIGHT )
				{
                                		Head::GetInstance()->initSearching(false, true, false, false);
				}
                        	else if(Head::GetInstance()->CheckLastPanAngle() == LEFT )
				{
                                		Head::GetInstance()->initSearching(false, false, false, true);
				}
				check = false;
			}
			else
			{
				Head::GetInstance()->MoveSearching();
			}
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTrackingFar(ball_position);
	}
	Head::GetInstance()->MoveSearchingPenaltyMark();
}
