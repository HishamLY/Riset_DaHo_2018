/**
 *   @file Perception.h
 *   @author Imre Nagi
 *
 */

#ifndef _PERCEPTION_H_
#define _PERCEPTION_H_

#include <string.h>
#include "Localization.h"
#include "Vector2.h"
#include "Point.h"
#include "MathFunction.h"
#include "CameraMatrix.h"
#include "ColorFinder.h"
 #include "Geometry.h"

namespace Robot
{
	enum Mark{
        CenterOppPost = 0,
        RightOppPost, 
        LeftOppPost,
        CenterOwnPost,
        RightOwnPost,
        LeftOwnPost, 
        CenterCircle 
	};

	class Perception
	{
	private:

		static Perception* m_UniqueInstance;
		Perception();
		
	public:
        /**
        * It gives you chances to access this Perception class directly using
        * Perception::GetInstance()-><MEMBER'S NAME>
        */

		static Perception* GetInstance() { return m_UniqueInstance; }
		~Perception();
		int ReadCameraOrMap;

		//static vector<Sensor> SensorsResult;
		static vector<Sensor> SensorsResult;

		/**
		* this function is used for pushing back a new element to Sensors's Vector
		* this function must be called everytime the camera detect landmarks (goal, centercircle )
       * @param pointonfield of landmark detected
       * @param Landmark code.
		*/
        void pushLandmark(Vector2<> pf, int typeLandmark);

        /**
        * gunanya untuk nyimpan seluruh hasil bacaan sensor kedalam PerceptResult
        */
		void CreatePercept();

		/**
		* clear the perception. Reset semua hasil bacaan
		*/
		void ResetPercept();

		/**
		* bikin testing untuk baca sensor dari kamera dengan kompas
		*/
		void ReadSensor();

		bool isOppGoal(unsigned int presentval, unsigned int setpoint, double headPan);
		//bool isOwnGoal(double bodyOrientation, double headPan);
		void update(CameraMatrix& thecameramatrix);
	};
}

#endif
