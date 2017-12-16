// krn 2015 modified by luqman
/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include "Vector.h"
#include "Matrix.h"
#include "MX28.h"
#include "MotionStatus.h"
#include "Kinematics.h"
#include "Walking.h"
using namespace Robot;


#define PI (3.14159265)
#define KICK_STRENGTH   35.0 //55.0     //EDIT FOR DYNAMIC KICK

//initialize filtering  Accel Gyro
double fbAccelFilter = MotionStatus::FB_ACCEL;
double rlAccelFilter = MotionStatus::RL_ACCEL;
double fbAccelFilter2 = MotionStatus::FB_ACCEL;
double rlAccelFilter2 = MotionStatus::RL_ACCEL;
double fbGyroFilter = MotionStatus::FB_GYRO;
double rlGyroFilter = MotionStatus::RL_GYRO;

/*daritem 2016*/
//Deklarasi variabel gerakan kompas tangan 
int count_uparm = 0;
int count_downarm = 0;

timeval ti;
double tStart = ti.tv_sec+(ti.tv_usec/1000000.0), tEnd;
double RobotAngle ;

double RobotAngle2 =0;
double RobotSpeed =0;

double TimeElapsed;
int counter = 0;
int counterStage =0;
bool m_TimeMinus = false;
double TempRobotAngleFromSurfaceFB = 0;
double X_MOVE_AMPLITUDE_TEMP = 0;
/***************************/
Walking* Walking::m_UniqueInstance = new Walking();

Walking::Walking()
{
	X_OFFSET = -10;
	Y_OFFSET = 10;
	Z_OFFSET = 20;
    R_OFFSET = 0;
	P_OFFSET = 0;
    A_OFFSET = 0;
    HIP_PITCH_OFFSET = 15.0;
	PERIOD_TIME = 600;
	DSP_RATIO = 0.1;
	STEP_FB_RATIO = 0.28;
	Z_MOVE_AMPLITUDE = 40;
    Y_SWAP_AMPLITUDE = 20.0;
    Y_SWAP_AMPLITUDE_TEMP = 20.0; //daritem 2016
    Z_SWAP_AMPLITUDE = 5;        
    Z_SWAP_AMPLITUDE_TEMP = 5;    //daritem 2016
    PELVIS_OFFSET = 3.0;
    ARM_SWING_GAIN = 1.5;
	BALANCE_KNEE_GAIN = 0.3;
	BALANCE_ANKLE_PITCH_GAIN = 0.9;
	BALANCE_HIP_ROLL_GAIN = 0.5;
	BALANCE_HIP_PITCH_GAIN = 0.5; //daritem 2016
	BALANCE_ANKLE_ROLL_GAIN = 1.0;

	P_GAIN = JointData::P_GAIN_DEFAULT;
    I_GAIN = JointData::I_GAIN_DEFAULT;
    D_GAIN = JointData::D_GAIN_DEFAULT;

	X_MOVE_SCALE = 2.4f;
    Y_MOVE_SCALE = 1.1f;
    A_MOVE_SCALE = 1.4f;

    /*marathon*/
    X_MOVE_COUNTER = 0;
    Y_MOVE_COUNTER = 0;
    A_MOVE_COUNTER = 0;
    /*************/

	X_MOVE_AMPLITUDE = 0;
	X_MOVE_AMPLITUDE_TEMP = 0; //daritem  2016
	Y_MOVE_AMPLITUDE = 0;
	A_MOVE_AMPLITUDE = 0;	
	A_MOVE_AIM_ON = false;
	BALANCE_ENABLE = true;

    X_KICK_AMPLITUDE = 0;       // EDIT FOR DYNAMIC KICKING

	//angle initialization
	m_Joint.SetAngle(JointData::ID_R_SHOULDER_PITCH, -48.345); //daritem2016 dikomen
	m_Joint.SetAngle(JointData::ID_L_SHOULDER_PITCH, 41.313);  //daritem2016 dikomen
 	m_Joint.SetAngle(JointData::ID_R_SHOULDER_ROLL, -17.873);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_ROLL, 17.580);
	//m_Joint.SetAngle(JointData::ID_R_ELBOW, 29.300);
	//m_Joint.SetAngle(JointData::ID_L_ELBOW, -29.593);
	m_Joint.SetValue(JointData::ID_R_ELBOW, 1650); //daritem2016 dikomen
    m_Joint.SetValue(JointData::ID_L_ELBOW, 2550); //daritem2016 dikomen

	m_Joint.SetAngle(JointData::ID_HEAD_TILT, Kinematics::EYE_TILT_OFFSET_ANGLE);

	//slope initialization
	m_Joint.SetSlope(JointData::ID_R_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	m_Joint.SetSlope(JointData::ID_L_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_R_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_L_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    /*marathon*/
    //m_Joint.SetSlope(JointData::ID_R_SHOULDER_PITCH, JointData::SLOPE_UNITY, JointData::SLOPE_UNITY);
    //m_Joint.SetSlope(JointData::ID_L_SHOULDER_PITCH, JointData::SLOPE_UNITY, JointData::SLOPE_UNITY);
    //m_Joint.SetSlope(JointData::ID_R_SHOULDER_ROLL, JointData::SLOPE_UNITY, JointData::SLOPE_UNITY);
    //m_Joint.SetSlope(JointData::ID_L_SHOULDER_ROLL, JointData::SLOPE_UNITY, JointData::SLOPE_UNITY);
    /**/
    m_Joint.SetSlope(JointData::ID_R_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_L_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	m_Joint.SetSlope(JointData::ID_HEAD_PAN, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);

	//PID setting
	//kalo yang lain ga di set, berarti masuk default (P_DEFAULT) (ada di jointdata) nilainya 32.
    m_Joint.SetPGain(JointData::ID_R_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_R_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_R_ELBOW, 8);
    m_Joint.SetPGain(JointData::ID_L_ELBOW, 8);

    StepCount = 0;      //EDIT FOR STEP COUNTING
    StartCount = false; //EDIT FOR STEP COUNTING
}

//destuctor
Walking::~Walking()
{
}

void Walking::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, WALKING_SECTION);
}
void Walking::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "x_offset", INVALID_VALUE)) != INVALID_VALUE)                X_OFFSET = value;
    if((value = ini->getd(section, "y_offset", INVALID_VALUE)) != INVALID_VALUE)                Y_OFFSET = value;
    if((value = ini->getd(section, "z_offset", INVALID_VALUE)) != INVALID_VALUE)                Z_OFFSET = value;
    if((value = ini->getd(section, "roll_offset", INVALID_VALUE)) != INVALID_VALUE)             R_OFFSET = value;
    if((value = ini->getd(section, "pitch_offset", INVALID_VALUE)) != INVALID_VALUE)            P_OFFSET = value;
    if((value = ini->getd(section, "yaw_offset", INVALID_VALUE)) != INVALID_VALUE)              A_OFFSET = value;
    if((value = ini->getd(section, "hip_pitch_offset", INVALID_VALUE)) != INVALID_VALUE)        HIP_PITCH_OFFSET = value;
    if((value = ini->getd(section, "period_time", INVALID_VALUE)) != INVALID_VALUE)             PERIOD_TIME = value;
    if((value = ini->getd(section, "dsp_ratio", INVALID_VALUE)) != INVALID_VALUE)               DSP_RATIO = value;
    if((value = ini->getd(section, "step_forward_back_ratio", INVALID_VALUE)) != INVALID_VALUE) STEP_FB_RATIO = value;
    if((value = ini->getd(section, "foot_height", INVALID_VALUE)) != INVALID_VALUE)             Z_MOVE_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_right_left", INVALID_VALUE)) != INVALID_VALUE)        Y_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_top_down", INVALID_VALUE)) != INVALID_VALUE)          Z_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "pelvis_offset", INVALID_VALUE)) != INVALID_VALUE)           PELVIS_OFFSET = value;
    if((value = ini->getd(section, "arm_swing_gain", INVALID_VALUE)) != INVALID_VALUE)          ARM_SWING_GAIN = value;
    if((value = ini->getd(section, "balance_knee_gain", INVALID_VALUE)) != INVALID_VALUE)       BALANCE_KNEE_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_pitch_gain", INVALID_VALUE)) != INVALID_VALUE)BALANCE_ANKLE_PITCH_GAIN = value;
    if((value = ini->getd(section, "balance_hip_roll_gain", INVALID_VALUE)) != INVALID_VALUE)   BALANCE_HIP_ROLL_GAIN = value; //marathon
    if((value = ini->getd(section, "balance_hip_pitch_gain", INVALID_VALUE)) != INVALID_VALUE)  BALANCE_HIP_PITCH_GAIN = value; //ditambahin biar sama kayak saveINISettings
    if((value = ini->getd(section, "balance_ankle_roll_gain", INVALID_VALUE)) != INVALID_VALUE) BALANCE_ANKLE_ROLL_GAIN = value; //marathon

    //edit for odometri tuning
    if((value = ini->getd(section, "x_move_scale", INVALID_VALUE)) != INVALID_VALUE)            X_MOVE_SCALE = value;
    if((value = ini->getd(section, "y_move_scale", INVALID_VALUE)) != INVALID_VALUE)            Y_MOVE_SCALE = value;
    if((value = ini->getd(section, "a_move_scale", INVALID_VALUE)) != INVALID_VALUE)            A_MOVE_SCALE = value;


    if((value = ini->getd(section, "balance_hip_roll_gain", INVALID_VALUE)) != INVALID_VALUE)   BALANCE_HIP_ROLL_GAIN = value;
    if((value = ini->getd(section, "balance_hip_pitch_gain", INVALID_VALUE)) != INVALID_VALUE)   BALANCE_HIP_PITCH_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_roll_gain", INVALID_VALUE)) != INVALID_VALUE) BALANCE_ANKLE_ROLL_GAIN = value;

    int ivalue = INVALID_VALUE;

    if((ivalue = ini->geti(section, "p_gain", INVALID_VALUE)) != INVALID_VALUE)                 P_GAIN = ivalue;
    if((ivalue = ini->geti(section, "i_gain", INVALID_VALUE)) != INVALID_VALUE)                 I_GAIN = ivalue;
    if((ivalue = ini->geti(section, "d_gain", INVALID_VALUE)) != INVALID_VALUE)                 D_GAIN = ivalue;
}
void Walking::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, WALKING_SECTION);
}
void Walking::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "x_offset",                 X_OFFSET);
    ini->put(section,   "y_offset",                 Y_OFFSET);
    ini->put(section,   "z_offset",                 Z_OFFSET);
    ini->put(section,   "roll_offset",              R_OFFSET);
    ini->put(section,   "pitch_offset",             P_OFFSET);
    ini->put(section,   "yaw_offset",               A_OFFSET);
    ini->put(section,   "hip_pitch_offset",         HIP_PITCH_OFFSET);
    ini->put(section,   "period_time",              PERIOD_TIME);
    ini->put(section,   "dsp_ratio",                DSP_RATIO);
    ini->put(section,   "step_forward_back_ratio",  STEP_FB_RATIO);
    ini->put(section,   "foot_height",              Z_MOVE_AMPLITUDE);
    ini->put(section,   "swing_right_left",         Y_SWAP_AMPLITUDE);
    ini->put(section,   "swing_top_down",           Z_SWAP_AMPLITUDE);
    ini->put(section,   "pelvis_offset",            PELVIS_OFFSET);
    ini->put(section,   "arm_swing_gain",           ARM_SWING_GAIN);
    ini->put(section,   "balance_knee_gain",        BALANCE_KNEE_GAIN);
    ini->put(section,   "balance_ankle_pitch_gain", BALANCE_ANKLE_PITCH_GAIN);
    ini->put(section,   "balance_hip_roll_gain",    BALANCE_HIP_ROLL_GAIN);
    ini->put(section,   "balance_hip_pitch_gain",   BALANCE_HIP_PITCH_GAIN); //daritem 2016
    ini->put(section,   "balance_ankle_roll_gain",  BALANCE_ANKLE_ROLL_GAIN);

    //edit for odometri tuning
    ini->put(section,   "x_move_scale",             X_MOVE_SCALE);
    ini->put(section,   "y_move_scale",             Y_MOVE_SCALE);
    ini->put(section,   "a_move_scale",             A_MOVE_SCALE);


    ini->put(section,   "p_gain",                   P_GAIN);
    ini->put(section,   "i_gain",                   I_GAIN);
    ini->put(section,   "d_gain",                   D_GAIN);
}
double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
	/*ini komentar dari daritem 2016*/
    //mag = amplituda
	//sin (2*pi*f*t - theta)
	//1/period = f
	//rumus umum: A sin(2*pi*f - theta) + shifting
}

bool Walking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
    Matrix3D Tad, Tda, Tcd, Tdc, Tac;
	double Out5Temp; //daritem 2016, dibawah dipake
	Vector3D vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
	double LEG_LENGTH = Kinematics::LEG_LENGTH;//panjang kaki total
	double THIGH_LENGTH = Kinematics::THIGH_LENGTH;//panjang paha
	double CALF_LENGTH = Kinematics::CALF_LENGTH;//panjang betis
	double ANKLE_LENGTH = Kinematics::ANKLE_LENGTH;//panjang ankle

	Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));

	//hasilnya 
	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
	_Rac = vec.Length();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if(isnan(_Acos) == 1)//isnan = is not a number
		return false;
    *(out + 3) = _Acos;//angle 3 dan 9

    // Get Ankle Roll

    Tda = Tad;
	if(Tda.Inverse() == false)
       	return false;
    _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);

    if(_m > 1.0)
        _m = 1.0;
    else if(_m < -1.0)
        _m = -1.0;

    _Acos = acos(_m);

    if(isnan(_Acos) == 1)
        return false;

    if(Tda.m[7] < 0.0)
    {
        *(out + 5) = _Acos;//isi dari adress out, kebalikan dari marathon
	Out5Temp = -_Acos; //daritem 2016
    }
    else
    {
        *(out + 5) = -_Acos;//angle 5 dan 11, kebalikan dari marathon
	Out5Temp = _Acos; //daritem 2016
    }

    // Get Hip Yaw
	Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(Out5Temp * 180.0 / PI, 0, 0));
	Tdc = Tcd;
	if(Tdc.Inverse() == false)
        return false;
	Tac = Tad * Tdc;
    _Atan = atan2(-Tac.m[1] , Tac.m[5]);

    if(isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;//angle[0] dan 6

    // Get Hip Roll
    _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;//angle 1 dan 7

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
	_n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if(isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan; //angle 2 dan 8
    *(out + 4) = _theta - *(out + 3) - *(out + 2);//angle 4 dan 10

    return true;
    //printf("%lf;%lf;%lf;%lf;%lf;%lf;", *(out), *(out+1), *(out+2), *(out+3), *(out+4), *(out+5));
}

void Walking::update_param_time()
{
    if(!Balance) //daritem 2016
    	m_PeriodTime = PERIOD_TIME;
    m_DSP_Ratio = DSP_RATIO;//0.1
    m_SSP_Ratio = 1 - DSP_RATIO;//0.9

    m_X_Swap_PeriodTime = m_PeriodTime / 2;//karena dalam 1 period, kedua kaki maju secara bergantian
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;//waktu yang dibutuhkan untuk kaki dalam keadaan salah satu mengijak tanah
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;//waktu kaki dalam keadaan salah satu menginjak tanah
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;//waktu awal saat akan mengangkat kaki kiri, 600/4 = 150 * 0.1 = 15 ms, jadi di awal 
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;//600/4 = 150 * 1.9 =  285ms (300-15)
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;//150 * 2.1 = 315ms
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;//150 + 3.9 = 585 ms

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2; //fasa 1 = (285 + 15) /2 = 150 
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2; //fasa 2 = (315 + 285) / 2 = 300
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2; //fasa 3 = (585 + 315) /2 = 450

    m_Pelvis_Offset = PELVIS_OFFSET*MX28::RATIO_ANGLE2VALUE;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void Walking::update_param_move()
{
    double compensator; //marathon
	// Forward/Back
    //m_X_Move_Amplitude = X_MOVE_AMPLITUDE; //marathon
    //m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO; //marathon
    
    /*daritem 2016*/
    if(!Balance) //daritem 2016
    {
		if(m_Z_Move_Amplitude < Z_MOVE_AMPLITUDE /2) 
			m_X_Move_Amplitude = 0;                  
		else
    		m_X_Move_Amplitude = X_MOVE_AMPLITUDE_TEMP; 
    	m_X_Swap_Amplitude = X_MOVE_AMPLITUDE_TEMP * STEP_FB_RATIO; //swap= pertukaran, kaki kiri dan kanan
    }
    else
    {
    	m_X_Swap_Amplitude = m_X_Move_Amplitude * STEP_FB_RATIO;//swap= pertukaran, kaki kiri dan kanan
    }
    /***********************/

    // Right/Left
    /*daritem 2016*/
    if(Y_MOVE_AMPLITUDE > 0)
    {
        m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;//Y-move/2
    }
    else
    {
        m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    }
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE_TEMP + m_Y_Move_Amplitude_Shift * 0.04;
    /********************/

    /*marathon
    m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
    if(m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;
    ************/

    if(!Balance) //daritem 2016
    {
		if(Walking::GetInstance()->IsRunning() == false)
			m_Z_Move_Amplitude = 0;
		else
		{
			if(m_Z_Move_Amplitude < Z_MOVE_AMPLITUDE/2)
				m_Z_Move_Amplitude += 2;
            if(m_Z_Move_Amplitude >= Z_MOVE_AMPLITUDE/2)
				m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE/2;
		}
		//m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    	m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    	m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE_TEMP;
    	m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;
    }
    else
    {
        //m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    	m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    	m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
    	m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    }
    /*marathon
    //tambahan gerakan
    if (X_MOVE_AMPLITUDE > 30.0)
    {
        compensator = (X_MOVE_AMPLITUDE / 30);
        m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE * compensator/ 2;
        m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
        m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE * compensator;
        m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;
    }
    else
        m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
        m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
        m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
        m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;
    ************/
    // Direction -> kalau aim true, arahnya putar kebalik
    if(A_MOVE_AIM_ON == false)
    {
        m_A_Move_Amplitude = A_MOVE_AMPLITUDE * PI / 180.0 / 2;//angle -> rad = angle * PI/180
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    }
    else
    {
        m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}

void Walking::update_param_balance()
{
	m_X_Offset = X_OFFSET;
    m_Y_Offset = Y_OFFSET;
    m_Z_Offset = Z_OFFSET;
    m_R_Offset = R_OFFSET * PI / 180.0;
    m_P_Offset = P_OFFSET * PI / 180.0;
    m_A_Offset = A_OFFSET * PI / 180.0;
    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*MX28::RATIO_ANGLE2VALUE;
}

void Walking::Initialize()
{
	X_MOVE_AMPLITUDE   = 0;
    Y_MOVE_AMPLITUDE   = 0;
    A_MOVE_AMPLITUDE   = 0;

	m_Body_Swing_Y = 0;
    m_Body_Swing_Z = 0;

    m_X_Swap_Phase_Shift = PI;
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = PI / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = PI / 2;
    m_Z_Swap_Phase_Shift = PI * 3 / 2;
    m_Z_Move_Phase_Shift = PI / 2;
    m_A_Move_Phase_Shift = PI / 2;

	m_Z_Move_Amplitude = 0;
    m_Z_Move_Amplitude_L = 15;
    m_Z_Move_Amplitude_R = 15;
	
    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;
    update_param_time();
    update_param_move();
    
    /*daritem 2016*/
    RobotSpeed = 0;
    RobotAngle2 = 0;
    TimeElapsed = 0;
    RobotAngle = 0;	//tambahan TA luqman
    counter = 0;
    Balance = false;
    BalanceRL = false;
	flagBalance = 0;
    m_BalanceCounter = 0;
	stage = 2;
	PrevStage = stage;
	MirrorValue = false;
    /********************/

    /*marathon*/
    //m_Right_Kick = false;   //EDITED FOR DYNAMIC KICKING
    //m_Left_Kick = false;    //EDITTED FOR DYNAMIC KICKING
    //Compass = false;
    /*******************/
    Process();
}

void Walking::Start()
{
	m_Ctrl_Running = true;
    m_Real_Running = true;
}

void Walking::Stop()
{
	m_Ctrl_Running = false;
}
		
bool Walking::IsRunning()
{
	return m_Real_Running;
}

//EDITED FOR DYNAMIC KICKING
void Walking::RightKick()
{
	m_Right_Kick = true;
}

void Walking::LeftKick()
{
	m_Left_Kick = true;
}

void Walking::NoKick()
{
	m_Left_Kick = false;
	m_Right_Kick = false;
}
//========================================

void Walking::Process()
{
    double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    double pelvis_offset_r, pelvis_offset_l;
    double angle[16], ep[12];
    double offset;
    double TIME_UNIT = MotionModule::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    int dir[16]          = {   -1,        -1,          1,         1,         -1,            1,          -1,        -1,         -1,         -1,         1,            1,           1,           -1,     1,	1 };//arah putarnya
    double initAngle[16] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,       0.0,        0.0,        0.0,       0.0,          0.0,       -48.345,       41.313, 	0.0,		0.0    };
    int outValue[16];
    double RobotAngleFromSurfaceFB; //daritem 2016
    double RobotAngleFromSurfaceRL; //daritem 2016
    //double arm_raise; //marathon
    
    //EDITTED FOR DYNAMIC KICKING
    //static bool m_RK_Flag = false; //marathon
    //static bool m_LK_Flag = false; //marathon
    //========================================

    /*daritem 2016*/
    //---tambahan TA LUQMAN
	int AngleOffset[16];
    gettimeofday(&ti, NULL);
    tEnd=ti.tv_sec+(ti.tv_usec/1000000.0);
	TimeElapsed = tEnd - tStart;
    //printf("time elapsed = %lf\n",tEnd - tStart);
    tStart = tEnd;
	//-----------
    /*********************/

    //initialisasi saat mulai berjalan
    if(m_Time == 0)
    {
        update_param_time();
        m_Phase = PHASE0;
        if(m_Ctrl_Running == false)
        {
    	   if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
    //m_time >= 150 -4 dan 150 + 4 (time unit = 8)
    {
        /*marathon
        //EDITTED FOR DYNAMIC KICKING
        if (m_Right_Kick && !m_RK_Flag)
        {
            X_KICK_AMPLITUDE = KICK_STRENGTH;   //langkah tendangan
            m_RK_Flag = true;
        }
        update_param_move();    //NOT EDITTED

        if (m_RK_Flag || m_LK_Flag)
        {
            //update moving parameter manually
            X_MOVE_AMPLITUDE = X_KICK_AMPLITUDE;    
            m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
            m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO; 
            if (X_KICK_AMPLITUDE == KICK_STRENGTH)
            {
                m_Z_Move_Amplitude = (Z_MOVE_AMPLITUDE + 15) / 2;
                m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
            }
            if (X_KICK_AMPLITUDE <= 5.0)
            {   
                m_Right_Kick = false;
                m_RK_Flag = false;
                m_Left_Kick = false;
                m_LK_Flag = false;
            }
            else
                X_KICK_AMPLITUDE = X_KICK_AMPLITUDE - 10.0;
        }
        //======================================
        ****************/
    	update_param_move();
        m_Phase = PHASE1;
		m_X_Moved = m_X_Moved + m_X_Move_Amplitude;
        m_Y_Moved = m_Y_Moved + m_Y_Move_Amplitude;
        m_A_Moved = m_A_Moved + m_A_Move_Amplitude;
    }
    else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
	{
       	update_param_time();
       	m_Time = m_Phase_Time2;
       	m_Phase = PHASE2;
       	if(m_Ctrl_Running == false)
       	{
       		if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
       		{
           		m_Real_Running = false;
       		}
       		else
       		{
        		X_MOVE_AMPLITUDE = 0;
        		Y_MOVE_AMPLITUDE = 0;
        		A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
    {
        /*marathon
        //EDITTED FOR STEP COUNTING
        if(StartCount)
        {
                StepCount++;
        }
        else
        {
                StepCount = 0;
        }
        //========================================
        
        //EDITTE FOR DYNAMIC KICKING
        if (m_Left_Kick && !m_LK_Flag)
        {
            X_KICK_AMPLITUDE = KICK_STRENGTH;   //langkah tendangan
            m_LK_Flag = true;
            
        }

            update_param_move();        //NOT EDITTED

        if (m_RK_Flag || m_LK_Flag)
        {
            //update moving parameter manually
            X_MOVE_AMPLITUDE = X_KICK_AMPLITUDE;    
            m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
                m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO; 
            if (X_KICK_AMPLITUDE == KICK_STRENGTH)
            {
                m_Z_Move_Amplitude = (Z_MOVE_AMPLITUDE + 15) / 2;
                m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
            }
            if (X_KICK_AMPLITUDE <= 5.0)
            {   
                m_Right_Kick = false;
                m_RK_Flag = false;
                m_Left_Kick = false;
                m_LK_Flag = false;
            }
            else
            X_KICK_AMPLITUDE = X_KICK_AMPLITUDE - 10.0;
        }
    //=============================================
        *************/
       	update_param_move();
       	m_Phase = PHASE3;
        // ODOMETRY VALUE UPDATE
        m_X_Moved = m_X_Moved + m_X_Move_Amplitude;
        m_Y_Moved = m_Y_Moved + m_Y_Move_Amplitude;
        m_A_Moved = m_A_Moved + m_A_Move_Amplitude;
    }
    update_param_balance();

	// Compute endpoints
	x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
	y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
	z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
	a_swap = 0;
	b_swap = 0;
	c_swap = 0;

    /*marathon
    if(m_Time <= m_SSP_Time_Start_L)
    {
        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_L)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
    }
    else if(m_Time <= m_SSP_Time_Start_R)
    {
        x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_R)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    }
    else
    {
        x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    *****************************************/

    //daritem 2016//
	if(Balance && m_BalanceCounter == 0)
	{
		double SteppingGain = 40;
		double SteppingHigh = 40;
		double SteppingHigh2 = 25;
		if(stage == 4 && stage == 5)
			m_PeriodTime = 3000;
		else
			m_PeriodTime = 1000;
		DSP_RATIO = 0.1;
		update_param_time();

		if(flagBalance == 0)
		{
			m_Time = (m_SSP_Time_End_L + m_SSP_Time_Start_L)/ 2;
			stage = 2;
		}

		flagBalance = 1;
		PrevStage = stage;

		if(stage == 2)//kaki kiri maju
		{
			m_Z_Move_Amplitude = SteppingHigh/2;
			m_Z_Move_Amplitude_R = SteppingHigh/2;
			m_Z_Move_Amplitude_L = SteppingHigh/2;
			m_X_Move_Amplitude = SteppingGain;
			m_Z_Offset = 40;
			update_param_move();
			stage = 2;
			counterStage = 0;
			if(m_Time >= m_SSP_Time_End_L)
			{
				stage = 3;
				m_Time = m_SSP_Time_End_L;
			}
		}
		else if(stage == 3)//kaki kiri harusnya tetep..kaki kanan juga diem
		{
			m_Z_Offset = 40;
			stage = 3;
			if(m_Time >= m_SSP_Time_Start_R)
			{
				stage = 5;
				m_Time = (m_SSP_Time_Start_R);
			}
		}
		else if(stage == 4)//kaki kiri diem, kaki kanan turun dari atas
		{
			m_Z_Move_Amplitude = SteppingHigh2/2;
			m_Z_Move_Amplitude_R = SteppingHigh2/2;
			m_Z_Move_Amplitude_L = SteppingHigh2/2;
			m_X_Move_Amplitude = 0;
			m_Z_Offset = 40;
			update_param_move();
			stage = 4;
			if(m_Time >= m_SSP_Time_End_R)
			{
				stage = 7;
			}
		}
		else if(stage == 5)//narik kaki kanan dari belakang ke tengah
		{
			m_Z_Move_Amplitude = SteppingHigh2/2;
			m_Z_Move_Amplitude_R = SteppingHigh2/2;
			m_Z_Move_Amplitude_L = SteppingHigh2/2;
			m_X_Move_Amplitude = SteppingGain;
			m_Z_Offset = 40;
			update_param_move();
			stage = 5;
			if(m_Time >= (m_SSP_Time_End_R + m_SSP_Time_Start_R)/2)
			{
				stage = 4;
				m_Time = (m_SSP_Time_Start_R + m_SSP_Time_End_R)/2;
			}
		}
		else
		{
			stage = 7;
			m_Z_Move_Amplitude = SteppingHigh/2;
			m_Z_Move_Amplitude_R = SteppingHigh/2;
			m_Z_Move_Amplitude_L = SteppingHigh/2;
			m_X_Move_Amplitude = 0;
			m_Z_Offset = 40;
			update_param_move();
			if(m_Time >= m_PeriodTime - TIME_UNIT)
			{
				stage = 2;
				m_Time = (m_SSP_Time_End_L + m_SSP_Time_Start_L)/2;
			}
		}

    	// Compute endpoints
    	x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    	y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    	z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
    	a_swap = 0;
    	b_swap = 0;
    	c_swap = 0;
		if(stage == 1)
		{
	        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
	        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
	        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
	        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
	        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
	        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
	        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
	        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
	        pelvis_offset_l = 0;
	        pelvis_offset_r = 0;
		}
		else if(stage == 2)
		{
	        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
		}
		else if(stage == 5)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
   		 	z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
		}
		else if(stage == 3)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_r = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
		}
		else if(stage == 4)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
        }
		else
		{
    		x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = 0;
    		pelvis_offset_r = 0;
		}
	}
	else if(BalanceRL)
	{
		if(stage == 4 && stage == 5)
			m_PeriodTime = 3000;
		else
			m_PeriodTime = 1000;
		m_X_Move_Amplitude = 0;
		DSP_RATIO = 0.1;
		update_param_time();

		if(flagBalance == 0)
		{
			m_Time = (m_SSP_Time_End_L + m_SSP_Time_Start_L)/ 2;
			stage = 2;
		}

		flagBalance = 1;
		PrevStage = stage;

		if(stage == 2)//kaki kiri maju
		{
			m_Z_Move_Amplitude = 70/2;
			m_Z_Move_Amplitude_R = 70/2;
			m_Z_Move_Amplitude_L = 70/2;
			m_Y_Move_Amplitude = 70;
			m_Z_Offset = 40;
			update_param_move();
			stage = 2;
			counterStage = 0;
			if(m_Time >= m_SSP_Time_End_L)
			{
				stage = 3;
				m_Time = m_SSP_Time_End_L;
			}
		}
		else if(stage == 3)//kaki kiri harusnya tetep..kaki kanan juga diem
		{
			m_Z_Offset = 40;
			stage = 3;
			if(m_Time >= m_SSP_Time_Start_R)
			{
				stage = 5;
				m_Time = (m_SSP_Time_Start_R);
			}
		}
		else if(stage == 4)//kaki kiri diem, kaki kanan turun dari atas
		{
			m_Z_Move_Amplitude = 30/2;
			m_Z_Move_Amplitude_R = 30/2;
			m_Z_Move_Amplitude_L = 30/2;
			m_Y_Move_Amplitude = 0;
			m_Z_Offset = 40;
			update_param_move();
			stage = 4;
			if(m_Time >= m_SSP_Time_End_R)
			{
				stage = 7;
			}
		}
		else if(stage == 5)//narik kaki kanan dari belakang ke tengah
		{
			m_Z_Move_Amplitude = 30/2;
			m_Z_Move_Amplitude_R = 30/2;
			m_Z_Move_Amplitude_L = 30/2;
			m_Y_Move_Amplitude = 70;
			m_Z_Offset = 40;
			update_param_move();
			stage = 5;
			if(m_Time >= (m_SSP_Time_End_R + m_SSP_Time_Start_R)/2)
			{
				stage = 4;
				m_Time = (m_SSP_Time_Start_R + m_SSP_Time_End_R)/2;
			}
		}
		else
		{
			stage = 7;
			m_Z_Move_Amplitude = 70/2;
			m_Z_Move_Amplitude_R = 70/2;
			m_Z_Move_Amplitude_L = 70/2;
			m_Y_Move_Amplitude = 0;
			m_Z_Offset = 40;
			update_param_move();
			if(m_Time >= m_PeriodTime - TIME_UNIT)
			{
				stage = 2;
				m_Time = (m_SSP_Time_End_L + m_SSP_Time_Start_L)/2;
			}
		}
	    	// Compute endpoints
	    	x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
	    	y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
	    	z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
	    	a_swap = 0;
	    	b_swap = 0;
	    	c_swap = 0;
		if(stage == 1)
		{
	        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
	        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
	        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
	        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
	        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
	        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
	        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
	        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
	        pelvis_offset_l = 0;
	        pelvis_offset_r = 0;
		}
		else if(stage == 2)
		{
	        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
        }
		else if(stage == 5)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
   		 	z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
        }
		else if(stage == 3)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_r = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        }
		else if(stage == 4)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
        }
		else
		{
    		x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude_L, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude_R, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = 0;
    		pelvis_offset_r = 0;
		}
	}
	else if(!Balance)
	{
		flagBalance = 0;
		DSP_RATIO = 0.1;
		update_param_time();

    	if(m_Time <= m_SSP_Time_Start_L)
		{
    		x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);//y minnya 
    		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = 0;
    		pelvis_offset_r = 0;
		}
		else if(m_Time <= m_SSP_Time_End_L)
		{
    		x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
		}
		else if(m_Time <= m_SSP_Time_Start_R)
		{
    		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = 0;
    		pelvis_offset_r = 0;
		}
		else if(m_Time <= m_SSP_Time_End_R)
		{
    		x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
    		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
		}
		else
		{
    		x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    		y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    		x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    		y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    		z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    		c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    		pelvis_offset_l = 0;
    		pelvis_offset_r = 0;
		}
	}
    //////////end daritem 2016///////////////////////////
	a_move_l = 0;
	b_move_l = 0;
	a_move_r = 0;
	b_move_r = 0;

	ep[0] = x_swap + x_move_r + m_X_Offset;
	ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
	ep[2] = z_swap + z_move_r + m_Z_Offset;
	ep[3] = a_swap + a_move_r - m_R_Offset / 2;
	ep[4] = b_swap + b_move_r + m_P_Offset;
	ep[5] = c_swap + c_move_r - m_A_Offset / 2;
	ep[6] = x_swap + x_move_l + m_X_Offset;
	ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
	ep[8] = z_swap + z_move_l + m_Z_Offset;
	ep[9] = a_swap + a_move_l + m_R_Offset / 2;
	ep[10] = b_swap + b_move_l + m_P_Offset;
	ep[11] = c_swap + c_move_l + m_A_Offset / 2;

	// Compute body swing
	if(m_Time <= m_SSP_Time_End_L)
	{
		m_Body_Swing_Y = -ep[7];
	    m_Body_Swing_Z = ep[8];
	}
	else
	{
    	m_Body_Swing_Y = -ep[1];
    	m_Body_Swing_Z = ep[2];
	}
    m_Body_Swing_Z -= Kinematics::LEG_LENGTH;
	/*
    //marathon
	// Compute arm swing
	if(m_X_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    }
    else
    {
        angle[12] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, m_PeriodTime, PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if(m_Y_Move_Amplitude == 0){
        angle[14] =  -10; //Right
        angle[15] = 10; //Left
    }
    else{
        m_Joint.SetAngle(JointData::ID_R_ELBOW, 5.300);     //  2940);          //Angle 29.300);
        m_Joint.SetAngle(JointData::ID_L_ELBOW, -5.300);        //  2940);          //Angle 29.300);
        angle[12] = 10;
        angle[13] = 10;
        angle[14] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_Y_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[15] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_Y_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }
	*/
	
    //increment m_Time by TIME_UNIT...
	if(m_Real_Running == true)
	{
    	m_Time += TIME_UNIT;
	   	if(m_Time >= m_PeriodTime)
            m_Time = 0;
	}

	// Compute angles
	if((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
   	    && (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1))
	{
    	for(int i=0; i<12; i++)
            angle[i] *= 180.0 / PI;//rad -> angle
	}
	else
	{
        return; // Do not use angle;
	}

	// Compute motor value
    //memberikan nilai value ke servodari angle yang sudah didapatkan saat compute angles
	for(int i=0; i<16; i++)
	{
        offset = (double)dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
    	if(i == 1) // R_HIP_ROLL
        	offset += (double)dir[i] * pelvis_offset_r;
    	else if(i == 7) // L_HIP_ROLL
        	offset += (double)dir[i] * pelvis_offset_l;
    	else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
        	offset -= (double)dir[i] * HIP_PITCH_OFFSET * MX28::RATIO_ANGLE2VALUE;
    	outValue[i] = MX28::Angle2Value(initAngle[i]) + (int)offset;
	}

    
	// adjust balance offset
    /*
    if(BALANCE_ENABLE == true)
    {
        double rlGyroErr = MotionStatus::RL_GYRO;
        double fbGyroErr = MotionStatus::FB_GYRO;

        //for filtering gyro
        double RAWfbGyro = MotionStatus::FB_GYRO;
        double RAWrlGyro = MotionStatus::RL_GYRO;
        double GyroFilterConst = 0.1;

        //for filtering accelerometer
        double RAWfbAccel = MotionStatus::FB_ACCEL;
        double RAWrlAccel = MotionStatus::RL_ACCEL;
        double AccelFilter1Const = 0.1;
        double AccelFilter2Const = 0.1;

        //filtering gyro
        fbGyroFilter = fbGyroFilter + (RAWfbGyro - fbGyroFilter) * GyroFilterConst;
        rlGyroFilter = rlGyroFilter + (RAWrlGyro - rlGyroFilter) * GyroFilterConst;

        //filtering accelerometer
        //2 step filtering
        //step 1
        fbAccelFilter = (fbAccelFilter + (RAWfbAccel - fbAccelFilter) * AccelFilter1Const) - (500 * AccelFilter1Const);
        rlAccelFilter = (rlAccelFilter + (RAWrlAccel - rlAccelFilter) * AccelFilter1Const) - (500 * AccelFilter1Const);
        //                                                                                  untuk transl$
        //step 2
        fbAccelFilter2 = (fbAccelFilter2 + (fbAccelFilter - fbAccelFilter2) * AccelFilter2Const);
        rlAccelFilter2 = (rlAccelFilter2 + (rlAccelFilter - rlAccelFilter2) * AccelFilter2Const);

        //printf("\t\%f;%f\t\t%f;%f\n", fbGyroFilter, rlGyroFilter, fbAccelFilter2, rlAccelFilter2);


    #ifdef MX28_1024
        outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // L_HIP_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN); // L_KNEE
        
        outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // L_ANKLE_PITCH        
        
        outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // L_ANKLE_ROLL
    #else
        outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*4); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*4); // L_HIP_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN*4); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN*4); // L_KNEE

        outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*4); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*4); // L_ANKLE_PITCH

        outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*4); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*4); // L_ANKLE_ROLL


        //compute arm pitch feedback
        outValue[13] = (-10 * fbGyroFilter) - (fbAccelFilter2 * 8) + 2050;
        outValue[12] = (10 * fbGyroFilter) + (fbAccelFilter2 * 8) + 2000;

        //compute arm roll feedback
        outValue[14] = (10 * rlGyroFilter) + ((rlAccelFilter2) * 8) + 1840;
        outValue[15] = (10 * rlGyroFilter) + ((rlAccelFilter2) * 8) + 2240;
        if (outValue[14] < 1735)
            outValue[15] = 1735;
        if (outValue[15] > 2400)
            outValue[14] = 2400;

        m_Joint.SetValue(JointData::ID_R_ELBOW, 1450);
        m_Joint.SetValue(JointData::ID_L_ELBOW, 2550);

        m_Joint.SetValue(JointData::ID_R_ELBOW, 1650);
        m_Joint.SetValue(JointData::ID_L_ELBOW, 2548);
    #endif
    }
    *****************/

    /*daritem 2016*/
	//diberikan kompensasi untuk balancing, tapi cuma ditambahin, gak dikali
	//sensor filtering
	//===== ACCELEROMETER & GYROSCOPE FEEDBACK ===============================
	double RAWfbGyro = MotionStatus::FB_GYRO;
    double RAWrlGyro = MotionStatus::RL_GYRO;
    double GyroFilterConst = 0.1;
    double RAWfbAccel = MotionStatus::FB_ACCEL;
    double RAWrlAccel = MotionStatus::RL_ACCEL;
    double AccelFilter1Const = 0.1;
    //filtering gyro
    fbGyroFilter = fbGyroFilter + (RAWfbGyro - fbGyroFilter) * GyroFilterConst;
    rlGyroFilter = rlGyroFilter + (RAWrlGyro - rlGyroFilter) * GyroFilterConst;
    fbAccelFilter = (fbAccelFilter + (RAWfbAccel - fbAccelFilter) * AccelFilter1Const);
    rlAccelFilter = (rlAccelFilter + (RAWrlAccel - rlAccelFilter) * AccelFilter1Const);

	double fbGyro = 0;
	double rlGyro = 0;

	double fbAccel = (fbAccelFilter - 512) * 4 * 980 / 512; //satuan cm per second kuadrat
	double rlAccel = (rlAccelFilter - 512) * 4 * 980 / 512; //satuan cm per second kuadrat
	double PercepatanSudut = fbAccel / 30;//tinggi kaki ke cm 30 cm
	//RobotSpeed = RobotSpeed + (PercepatanSudut * TimeElapsed);
	//RobotAngle2 = RobotAngle2 + (RobotSpeed * TimeElapsed);

	//dikomnen sementara
	//printf("Accelfb = %lf \t", fbAccel);
	//printf("Accelrl = %lf \t", rlAccel);
	//======================
	//printf("Percepatan Sudut = %lf\n", PercepatanSudut);
	//printf(" Robot Speed = %lf\n", RobotSpeed);
	//printf(" Robot Angle 2 = %lf\n", RobotAngle2);

	fbGyro = (fbGyroFilter) * 1600 / 512; //satuannya degree per second
	rlGyro = (rlGyroFilter) * 1600 / 512; //satuannya degree per second
	RobotAngle = RobotAngle + (fbGyro * TimeElapsed);
	RobotAngleFromSurfaceFB = 0;
	RobotAngleFromSurfaceRL = 0;

	RobotAngleFromSurfaceFB = fbAccel * 90 / 980;//satuan derajat, 90 adalah sudut maksimum, 980 = g
	RobotAngleFromSurfaceFB = RobotAngleFromSurfaceFB + (fbGyro * TimeElapsed);
	RobotAngleFromSurfaceRL = rlAccel * 90 / 980;//satuan derajat, 90 adalah sudut maksimum, 980 = g
	RobotAngleFromSurfaceRL = RobotAngleFromSurfaceRL + (rlGyro * TimeElapsed);

	RobotAngleFromSurfaceFB *= -1;//normalisasi, kalo ke depan positif
	RobotAngleFromSurfaceRL *= -1;//normalisasi, kalo ke kanan positif
	RobotAngleFromSurfaceRL -= 0;
    	RobotAngleFromSurfaceFB -= 19.0;

	counter++;
	if(counter < 5)
	{
		RobotAngle2 =0;
		RobotSpeed = 0;
		RobotAngleFromSurfaceFB = 0;
		TempRobotAngleFromSurfaceFB = 0;
		RobotAngleFromSurfaceRL = 0;
	}
	//dikomen sementara
	//printf("Gyro = %lf \t", RAWfbGyro);
	//printf("Gyrorl = %lf \t", RAWrlGyro);
	//printf("Robot Angle FB = %lf\n", RobotAngleFromSurfaceFB);
	//printf("Robot Angle RL = %lf\n", RobotAngleFromSurfaceRL);
	//===============
	//========================================================
	if(BALANCE_ENABLE == true)
	{

		//untuk mirror kaki kiri dan kanan
		if((Balance && RobotAngleFromSurfaceRL > 5) && m_Time <= (m_SSP_Time_End_L + m_SSP_Time_Start_L)/2 + TIME_UNIT)
		MirrorValue = false;
		else if((Balance && RobotAngleFromSurfaceRL < -5) && m_Time <= (m_SSP_Time_End_L + m_SSP_Time_Start_L)/2 + TIME_UNIT)
		MirrorValue = true;

	
	X_MOVE_AMPLITUDE_TEMP = X_MOVE_AMPLITUDE;
	if(RobotAngleFromSurfaceFB < -10)
	{
		X_MOVE_AMPLITUDE_TEMP = -5;
	}
	else
	{
		X_MOVE_AMPLITUDE_TEMP += 1;
		if(X_MOVE_AMPLITUDE_TEMP > X_MOVE_AMPLITUDE)
			X_MOVE_AMPLITUDE_TEMP = X_MOVE_AMPLITUDE;;
	}
		double fbGyroErr = MotionStatus::FB_GYRO;
    	double rlGyroErr = MotionStatus::RL_GYRO;
		double fbAccelErr = MotionStatus::FB_ACCEL;
    	double rlAccelErr = MotionStatus::RL_ACCEL;

	#ifdef MX28_1024
    	outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // R_HIP_ROLL
    	outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // L_HIP_ROLL

    	outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN); // R_KNEE
    	outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN); // L_KNEE

        outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // L_ANKLE_PITCH        

        outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // L_ANKLE_ROLL
	#else

    	if(!Balance)
    	{
        	outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4); // R_HIP_ROLL
        	outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4); // L_HIP_ROLL

        	outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN * 4); // R_KNEE
        	outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN * 4); // L_KNEE

            outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4); // R_ANKLE_PITCH
            outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4); // L_ANKLE_PITCH        

            outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4); // R_ANKLE_ROLL
            outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4); // L_ANKLE_ROLL
    	}
    	else if(Balance)
    	{
    		/*
    		if(!MirrorValue)
    			outValue[3] -= (int)(dir[3] * RobotAngleFromSurfaceFB  * 4095 /360 * BALANCE_KNEE_GAIN);// + (RobotAngleFromSurfaceFB +TempRobotAngleFromSurfaceFB) * 4095 /360 * BALANCE_KNEE_GAIN * 16 * TimeElapsed); // R_ANKLE_PITCH//-1
    		else
    			outValue[9] -= (int)(dir[9] * RobotAngleFromSurfaceFB  * 4095 /360 * BALANCE_KNEE_GAIN);// + (RobotAngleFromSurfaceFB +TempRobotAngleFromSurfaceFB) * 4095 /360 * BALANCE_KNEE_GAIN *16  * TimeElapsed); // R_ANKLE_PITCH//-1
    		*/
    		if(stage == 2)
    		{
    			if(!MirrorValue)
    				outValue[10] -= (int)(dir[10] * RobotAngleFromSurfaceFB * 4095 /360 + (RobotAngleFromSurfaceFB +TempRobotAngleFromSurfaceFB) * 4095 /360 * BALANCE_ANKLE_PITCH_GAIN * 16 * TimeElapsed); // R_ANKLE_PITCH//-1
    			else
    				outValue[4] -= (int)(dir[4] * RobotAngleFromSurfaceFB * 4095 /360  + (RobotAngleFromSurfaceFB +TempRobotAngleFromSurfaceFB) * 4095 /360 * BALANCE_ANKLE_PITCH_GAIN * 16 * TimeElapsed); // R_ANKLE_PITCH//-1
    		}
    		if(stage == 4 && stage == 5)
    		{
    			if(!MirrorValue)
    			{
    				outValue[5] += 300;
    			}
    			else
    			{
    				outValue[11] -= 300;
    			}
    		}
    	}
    #endif
    	TempRobotAngleFromSurfaceFB = RobotAngleFromSurfaceFB;
    }

	//arm's feedback
	if(counter > 5)
	{
		if(RobotAngleFromSurfaceFB > 0)
		{
		outValue[12] = outValue[12] - RobotAngleFromSurfaceFB * 2 * 4095 / 360;
		outValue[13] = outValue[13] + RobotAngleFromSurfaceFB * 2 * 4095 / 360;
		}
		else if(RobotAngleFromSurfaceFB < 0)
		{
		outValue[12] = outValue[12] - RobotAngleFromSurfaceFB * 2 * 4095 / 360;
		outValue[13] = outValue[13] + RobotAngleFromSurfaceFB * 2 * 4095 / 360;
		}
		outValue[14] = outValue[14] - RobotAngleFromSurfaceRL * 1 * 4095 / 360;
		outValue[15] = outValue[15] - RobotAngleFromSurfaceRL * 1 * 4095 / 360;
	}

	//servo's value limitation
	if(outValue[1] < 1971)
		outValue[1] = 1971;
	else if(outValue[1] > 2240)
		outValue[1] = 2240;
	if(outValue[7] > 2044)
		outValue[7] = 2044;
	else if(outValue[7] < 1794)
		outValue[7] = 1794;
	if(outValue[14] < 1845)
		outValue[14] =1845;
	if(outValue[15] > 2248)
		outValue[15] = 2248;
/******************************/
	//printf("%d\n", m_Joint.GetValue(JointData::ID_R_SHOULDER_PITCH));
	//dikomen sementara
	//printf("offset L_HIP_PITCH = %lf\n", MX28::Value2Angle(outValue[8]) - MX28::Value2Angle(2345));
	//printf("offset R_HIP_PITCH = %lf\n", MX28::Value2Angle(outValue[2]) - MX28::Value2Angle(1751));
	//================
	m_Joint.SetValue(JointData::ID_R_HIP_YAW,           outValue[0]);
	m_Joint.SetValue(JointData::ID_R_HIP_ROLL,          outValue[1]);//ID 9
	m_Joint.SetValue(JointData::ID_R_HIP_PITCH,         outValue[2]);
	m_Joint.SetValue(JointData::ID_R_KNEE,              outValue[3]);
	m_Joint.SetValue(JointData::ID_R_ANKLE_PITCH,       outValue[4]);
	m_Joint.SetValue(JointData::ID_R_ANKLE_ROLL,        outValue[5]);
	m_Joint.SetValue(JointData::ID_L_HIP_YAW,           outValue[6]);
	m_Joint.SetValue(JointData::ID_L_HIP_ROLL,          outValue[7]);
	m_Joint.SetValue(JointData::ID_L_HIP_PITCH,         outValue[8]);
	m_Joint.SetValue(JointData::ID_L_KNEE,              outValue[9]);
	m_Joint.SetValue(JointData::ID_L_ANKLE_PITCH,       outValue[10]);
	m_Joint.SetValue(JointData::ID_L_ANKLE_ROLL,        outValue[11]);
	m_Joint.SetValue(JointData::ID_R_SHOULDER_PITCH,    outValue[12]);
	m_Joint.SetValue(JointData::ID_L_SHOULDER_PITCH,    outValue[13]);
	m_Joint.SetValue(JointData::ID_R_SHOULDER_ROLL,    outValue[14]);
    m_Joint.SetValue(JointData::ID_L_SHOULDER_ROLL,    outValue[15]);
	m_Joint.SetAngle(JointData::ID_HEAD_PAN, A_MOVE_AMPLITUDE);

	for(int id = JointData::ID_R_HIP_YAW; id <= JointData::ID_L_ANKLE_ROLL; id++)
	{
	    m_Joint.SetPGain(id, P_GAIN);
        m_Joint.SetIGain(id, I_GAIN);
        m_Joint.SetDGain(id, D_GAIN);
	}
    /*daritem 2016*/
	//--- tambahan TA LUQMAN
	for(int i = 0; i <= 13; i++)
	{
		AngleOffset[i] = (outValue[i] - 2047) * 360 / 4095;
	}
	//----
    /**********/
}
/*
double Walking::Max(double input1, double input2)
{
	if(input1 >= input2)
		return(input1);
	else
		return(input2);
}

bool Walking::FSRcalculation(double fsr1, double fsr2, double *ReturnValue)//fsr1 positif
{
	int MAX_VALUE = 127;
	if(Max(fsr1, fsr2) == 0)
		return (0);
	else
	{
		*ReturnValue = (((fsr1 - fsr2) / (Max(fsr1, fsr2))) * MAX_VALUE);
		return (1);
	}
}
*/

double Walking::Get_X_Moved()  { double temp_X_Moved = m_X_Moved * X_MOVE_SCALE; m_X_Moved = 0; return temp_X_Moved; } //dalam mm
double Walking::Get_Y_Moved()  { double temp_Y_Moved = m_Y_Moved * Y_MOVE_SCALE; m_Y_Moved = 0; return temp_Y_Moved; } // dalam mm
double Walking::Get_A_Moved()  { double temp_A_Moved = m_A_Moved / A_MOVE_SCALE; m_A_Moved = 0; return temp_A_Moved; } //dalam rad

