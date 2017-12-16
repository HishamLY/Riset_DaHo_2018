#ifndef _SELFLOCALIZE_H_
#define _SELFLOCALIZE_H_

#include "MathFunction.h"
#include "minIni.h"

#define SECTION   "Localization"
#define INVALID_VALUE   -1024.0

#define NO_MOVEMENT 0
#define TRANSLATE_FORWARD 1
#define TRANSLATE_BACKWARD 2
#define TRANSLATE_RIGHT 3
#define TRANSLATE_LEFT 4
#define ROTATE_RIGHT 5
#define ROTATE_LEFT 6

namespace Robot{
    struct coord{
        double x;
        double y;
        double angle;
    };


    enum PostSeen{
       UNFOUND,
       UNIDENTIFIED,
       LEFT_PART,
       RIGHT_PART,
       MIDDLE_PART,
       BOTH_PART
    };

    struct Sample{
                coord PositionAndOrientation;
                double prob;
            };

    enum Goal{
        NO_GOAL = 0,
        YELLOW_GOAL = 1,
        BLUE_GOAL = 2,
    };
    class Sensor {
	public:
		double jarak;
		double arah;
		double n;
	public:
		Sensor();
		friend ostream& operator<< (ostream&, const Sensor&);
		Sensor(double, double, double);
};

    class SelfLocalize
    {
        public :
            int total_samples;
            int window_width;
            int window_height;
            int field_width;
            int field_height;
            int border_strip_width;
            int y_goal_top;
            int y_goal_bottom;

            double Jarak; //jarak gawang
            double Jarak1; //jarak titik penalty
            double bobotmax;

            int SampleTypeOfGoal;
			int SamplePartOfGoalPostSeen;
			double SampleDistance;
			double SampleDis;


            int delta_x_clusters;
            int delta_y_clusters;
            int delta_angle_clusters;

            static vector<Sample> Samples;
            static Sample Solution;
            coord Hipotesis;
            coord Robot;

            static int line_angle_from_heading;

            static coord lline, rline, hline, tline;

            void Init();
            void generateRandomSamples( double , int );
            
            MATH::Scalar check_landmark( int, int, int );
            MATH::Scalar check_landmark();

            void checkSensorModel(int TypeOfGoal, int PartOfGoalPostSeen, double Distance, double Dis);
            void resampling();
            void addSamples();
            void paintModel();

            void Odometri(double xmove, double ymove, double amove);
            void BacaSensor();
            
            void getSolution();

            void LoadINISettings(minIni* ini);
            void LoadINISettings(minIni* ini, const std::string &section);
            void SaveINISettings(minIni* ini);
            void SaveINISettings(minIni* ini, const std::string &section);
    };
}

#endif
