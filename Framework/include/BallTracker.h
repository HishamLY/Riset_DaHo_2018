/*
 *   BallTracker.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _BALL_TRACKER_H_
#define _BALL_TRACKER_H_

#include <string.h>

#include "Point.h"
#include "minIni.h"

namespace Robot
{
	class BallTracker
	{
	private:
		int NoBallCount;
		static const int NoBallMaxCount = 15;

	public:
		Point2D     ball_position;
		bool check;

		BallTracker();
		~BallTracker();

		void initBallCount() { NoBallCount = 0; }
		void Process(Point2D pos);
		void ProcessHighResolution(Point2D pos, bool *IsOneCycle);
		void ProcessUpHead(Point2D pos);
		void ProcessCheckPostLeft(Point2D pos);
		void ProcessCheckPostAhead(Point2D pos);
		void ProcessCheckPostRight(Point2D pos);
		void ProcessCheckPost(Point2D pos, int CompassError);
		void ProcessCheckPost(Point2D pos);
		void ProcessToKick(Point2D pos);
		void ProcessKick(Point2D pos);
		void ProcessLocalizePenaltyMark(Point2D pos);
	};
}

#endif
