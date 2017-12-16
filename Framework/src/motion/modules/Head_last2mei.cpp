/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <math.h>
#include "MX28.h"
#include "Kinematics.h"
#include "MotionStatus.h"
#include "Head.h"
#include "Camera.h"

using namespace Robot;

int countHeadSearchingCycle = 0;

Head* Head::m_UniqueInstance = new Head();

Head::Head()
{
	inc = 7;
	inc_pan = 0;
	inc_tilt = 10;//tadinya 10
	m_LastPanAngle = 0;

	m_Pan_p_gain = 0.1;
	m_Pan_d_gain = 0.22;

    m_Tilt_p_gain = 0.1;
	m_Tilt_d_gain = 0.22;



	m_LeftLimit = 30;

	m_RightLimit = -30;
	//m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE;
	m_TopLimit  = Kinematics::EYE_TILT_OFFSET_ANGLE + 20;
	m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65;

	m_Pan_Home = 0.0;
	m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0;

	m_Joint.SetEnableHeadOnly(true);
	//countHeadSearchingCycle = 0;

//	step1 = false;
//	step2 = false;
//	step3 = false;
//	step4 = false;
}

Head::~Head()
{
}

void Head::LookUp()
{
	MoveByAngle(0, m_TopLimit);
}

void Head::LookDown()
{
	MoveByAngle(0, m_BottomLimit);
}

double Head::GetHorizon()
{
	double m_PanAngles;
	if(m_PanAngle < 0)
		m_PanAngles = -m_PanAngle;
	else
		m_PanAngles = m_PanAngle;
	double yHorizon = ((double)Camera::HEIGHT/2.0) +  (m_TiltAngle - 57) * ((double)Camera::HEIGHT / Camera::VIEW_V_ANGLE) + (m_PanAngles * 0.09) *  ((double)Camera::HEIGHT / Camera::VIEW_V_ANGLE) ;
	if(yHorizon < 0)
		yHorizon = 2.0;
	else if(yHorizon > Camera::HEIGHT)
		yHorizon = (double)Camera::HEIGHT - 2.0;
	return yHorizon;
}



void Head::CheckLimit()
{
	if(m_PanAngle > m_LeftLimit)
		m_PanAngle = m_LeftLimit;
	else if(m_PanAngle < m_RightLimit)
		m_PanAngle = m_RightLimit;

	//tambahan untuk daritem
	if((m_PanAngle > 70) || (m_PanAngle < -70))
		m_BottomLimit = 0;
	else
		m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65;

	if(m_TiltAngle > m_TopLimit)
		m_TiltAngle = m_TopLimit;
	else if(m_TiltAngle < m_BottomLimit)
		m_TiltAngle = m_BottomLimit;	
}

void Head::Initialize()
{
	m_PanAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	m_TiltAngle = -MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	CheckLimit();

	InitTracking();
	MoveToHome();
}

void Head::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, HEAD_SECTION);
}

void Head::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "pan_p_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_p_gain = value;
    if((value = ini->getd(section, "pan_d_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_d_gain = value;
    if((value = ini->getd(section, "tilt_p_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_p_gain = value;
    if((value = ini->getd(section, "tilt_d_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_d_gain = value;
    if((value = ini->getd(section, "left_limit", INVALID_VALUE)) != INVALID_VALUE)  m_LeftLimit = value;
    if((value = ini->getd(section, "right_limit", INVALID_VALUE)) != INVALID_VALUE) m_RightLimit = value;
    if((value = ini->getd(section, "top_limit", INVALID_VALUE)) != INVALID_VALUE)   m_TopLimit = value;
    if((value = ini->getd(section, "bottom_limit", INVALID_VALUE)) != INVALID_VALUE)m_BottomLimit = value;
    if((value = ini->getd(section, "pan_home", INVALID_VALUE)) != INVALID_VALUE)    m_Pan_Home = value;
    if((value = ini->getd(section, "tilt_home", INVALID_VALUE)) != INVALID_VALUE)   m_Tilt_Home = value;
}

void Head::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, HEAD_SECTION);
}

void Head::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "pan_p_gain",   m_Pan_p_gain);
    ini->put(section,   "pan_d_gain",   m_Pan_d_gain);
    ini->put(section,   "tilt_p_gain",  m_Tilt_p_gain);
    ini->put(section,   "tilt_d_gain",  m_Tilt_d_gain);
    ini->put(section,   "left_limit",   m_LeftLimit);
    ini->put(section,   "right_limit",  m_RightLimit);
    ini->put(section,   "top_limit",    m_TopLimit);
    ini->put(section,   "bottom_limit", m_BottomLimit);
    ini->put(section,   "pan_home",     m_Pan_Home);
    ini->put(section,   "tilt_home",    m_Tilt_Home);
}

void Head::MoveToHome()
{
	MoveByAngle(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveByAngle(double pan, double tilt)
{
	m_PanAngle = pan;
	m_TiltAngle = tilt;

	CheckLimit();
}

void Head::MoveByAngleOffset(double pan, double tilt)
{	
	MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}

void Head::InitTracking()
{
	m_Pan_err = 0;
	m_Pan_err_diff = 0;
	m_Tilt_err = 0;
	m_Tilt_err_diff = 0;
}

void Head::MoveRight(int increment)
{
	m_PanAngle -= increment;
	CheckLimit();
}

void Head::MoveLeft(int increment)
{
	m_PanAngle += increment;
	CheckLimit();
}

void Head::MoveUp(int increment)
{
	m_TiltAngle += increment;
}

void Head::MoveDown(int increment)
{
	m_TiltAngle -= increment;
}

double pan, tilt;

void Head::initSearching(bool phase1, bool phase2, bool phase3, bool phase4)
{
	pan = 5;//tadinya 6
        tilt = (m_TopLimit-m_BottomLimit) * 4.0 * pan / (m_LeftLimit - m_RightLimit);//10

	step1 = phase1;
	step2 = phase2;
	step3 = phase3;
	step4 = phase4;

	if(phase2)
	{
		MoveByAngle(m_PanAngle,m_TiltAngle);
	//	MoveByAngle(0, m_TopLimit);
		inc_pan = -pan;
		inc_tilt = -tilt;
	}
	else if(phase4)
	{
		MoveByAngle(m_PanAngle,m_TiltAngle);
//		MoveByAngle(0, m_BottomLimit);
		inc_pan = pan;
		inc_tilt = tilt;
	}
	
	countHeadSearchingCycle = 0;
}

void Head::MoveSearchingPenaltyMark()
{
	m_PanAngle = 0;
	m_TiltAngle += inc_tilt;

	if(m_TiltAngle >= m_TopLimit)  inc_tilt = -5;
        else if(m_TiltAngle <= m_BottomLimit)  inc_tilt = 5;
}

void Head::MoveSearching()
{
	m_PanAngle += inc_pan;
        m_TiltAngle += inc_tilt;

	CheckLimit();

	if(step1)//saat kepala berada di paling kiri..kalo tiltnya max, 
	{
		m_PanAngle = m_LeftLimit;
		if(m_TiltAngle >= m_TopLimit) {step1 = false; step2 = true;}
		else {inc_pan = 0; inc_tilt = tilt;}
	}
	else if(step2)
        {
                inc_pan = -pan;
                if(m_TiltAngle >= m_TopLimit)  inc_tilt = -tilt;
                else if(m_TiltAngle <= m_BottomLimit)  inc_tilt = tilt;

		if(m_PanAngle <= m_RightLimit && m_TiltAngle >= m_TopLimit) {step2=false; step3=true;}
        }
	else if(step3)
	{
		m_PanAngle = m_RightLimit;
		if(m_TiltAngle <= m_BottomLimit) {step3 = false; step4 = true;}
                else {inc_pan = 0; inc_tilt = -tilt;}
	}
	else if(step4)
        {
                inc_pan = pan;

                if(m_TiltAngle >= m_TopLimit)  inc_tilt = -tilt;
                else if(m_TiltAngle <= m_BottomLimit)  inc_tilt = tilt;

		if(m_PanAngle >= m_LeftLimit && m_TiltAngle <= m_BottomLimit) {step1=true; step4=false;}
        }

	if(m_PanAngle == m_LeftLimit && m_TiltAngle >= m_TopLimit)
		countHeadSearchingCycle++;

	if(m_PanAngle == m_RightLimit && m_TiltAngle <= m_BottomLimit)
		countHeadSearchingCycle++;
}

void Head::MoveSearchingTurn()
{
	CheckLimit();

	m_TiltAngle += inc_tilt/2;
	
	if((int)m_PanAngle == 0)
	{
		if(m_TiltAngle >= m_TopLimit)
			inc_tilt = -tilt;
		else if(m_TiltAngle <=m_BottomLimit)
			inc_tilt = tilt;
	}
	else
	{
		if((m_PanAngle > 6) || (m_PanAngle < -6))
		{
			MoveSearching();
		}
		else
		{
			m_PanAngle = 0.0f;
		}
	}	
	
	if(m_TiltAngle >= m_TopLimit)
		countHeadSearchingCycle++;

	if(m_TiltAngle <= m_BottomLimit)
		countHeadSearchingCycle++;
	
}

void Head::initSearchingPost(int direction)
{
	dir = direction;
}

void Head::SetLimit(int CompassError)
{
    double Error = CompassError * 360 /255;
    if(Error < 0)
    {
        m_LeftLimit = 40 - Error;
        m_RightLimit = -40 - Error; 
    }
    else
    {
        m_LeftLimit = 40 - Error;
        m_RightLimit  = -40- Error;
    }

	if(m_LeftLimit > 40)
		m_LeftLimit = 40;
	if(m_RightLimit < -40)
		m_RightLimit = -40;
}
void Head::MoveSearchingPostLeft()
{
	if(m_PanAngle >= m_LeftLimit) countHeadSearchingCycle++;
        if(m_PanAngle <= 0) countHeadSearchingCycle++;

	m_TiltAngle = 70;//semula 40

	//inc_pan = m_LeftLimitAngle/2;
	inc_pan = 24;//semula 24
	
	if(dir == LEFT)
		m_PanAngle += inc_pan;
	else
		m_PanAngle += -inc_pan;

	 if(m_PanAngle >= m_LeftLimit) dir = RIGHT;
         if(m_PanAngle <= 0) dir = LEFT;

	CheckLimit();
}

void Head::MoveSearchingPostRight()
{
	if(m_PanAngle >= 0) countHeadSearchingCycle++;
        if(m_PanAngle <= m_RightLimit) countHeadSearchingCycle++;

	m_TiltAngle = 70;//semula 40

	//inc_pan = m_LeftLimitAngle/2;
	inc_pan = 24;//semula 24

	if(dir == LEFT)
		m_PanAngle += inc_pan;
	else
		m_PanAngle += -inc_pan;

	 if(m_PanAngle >= 0) dir = RIGHT;
         if(m_PanAngle <= m_RightLimit) dir = LEFT;

	CheckLimit();
}

void Head::MoveSearchingPost(int CompassError)
{
	CompassError = CompassError * 360 / 255;
	if(m_PanAngle >= m_LeftLimit) countHeadSearchingCycle++;
        if(m_PanAngle <= m_RightLimit) countHeadSearchingCycle++;

	m_TiltAngle = 80;//semula 40

	//inc_pan = m_LeftLimitAngle/2;
	inc_pan = 24;//semula 24

	if(dir == LEFT)
		m_PanAngle += inc_pan;
	else
		m_PanAngle += -inc_pan;

	 if(m_PanAngle >= m_LeftLimit) dir = RIGHT;
         if(m_PanAngle <= m_RightLimit) dir = LEFT;

	CheckLimit();
}
void Head::MoveSearchingPost()
{
	if(m_PanAngle >= m_LeftLimit) countHeadSearchingCycle++;
        if(m_PanAngle <= m_RightLimit) countHeadSearchingCycle++;

	m_TiltAngle = 80;//semula 40

	//inc_pan = m_LeftLimitAngle/2;
	inc_pan = 5;//semula 24
	
	if(dir == LEFT)
		m_PanAngle += inc_pan;
	else
		m_PanAngle += -inc_pan;

	 if(m_PanAngle >= m_LeftLimit) dir = RIGHT;
         if(m_PanAngle <= m_RightLimit) dir = LEFT;

	CheckLimit();
}

void Head::MoveSearchingPenaltyKick()
{
	m_TiltAngle = -25;
	m_PanAngle += inc;

	if(m_PanAngle > m_LeftLimit) inc = -10;
	else if(m_PanAngle < m_RightLimit) inc = 10;
}

void Head::MoveTracking(Point2D err)
{	
	m_Pan_err_diff = err.X - m_Pan_err;
	m_Pan_err = err.X;

	m_Tilt_err_diff = err.Y - m_Tilt_err;
	m_Tilt_err = err.Y;

	MoveTracking();
}

void Head::MoveTracking()
{
	double pOffset, dOffset;

	pOffset = m_Pan_err * m_Pan_p_gain;
	pOffset *= pOffset;
	if(m_Pan_err < 0)
		pOffset = -pOffset;
	dOffset = m_Pan_err_diff * m_Pan_d_gain;
	dOffset *= dOffset;
	if(m_Pan_err_diff < 0)
		dOffset = -dOffset;
	m_PanAngle += (pOffset + dOffset);	//DIKOMEN
	m_PanAngle += pOffset;

	pOffset = m_Tilt_err * m_Tilt_p_gain;
	pOffset *= pOffset;
	if(m_Tilt_err < 0)
		pOffset = -pOffset;
	dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
	dOffset *= dOffset;
	if(m_Tilt_err_diff < 0)
		dOffset = -dOffset;
	m_TiltAngle += (pOffset + dOffset);	//DIKOMEN
	m_TiltAngle += pOffset;

	CheckLimit();
}

void Head::MoveTrackingFar(Point2D err)
{
        m_Pan_err_diff = err.X - m_Pan_err;
        m_Pan_err = err.X;

        m_Tilt_err_diff = err.Y - m_Tilt_err;
        m_Tilt_err = err.Y;

        MoveTrackingFar();
}

void Head::MoveTrackingFar()
{
        double pOffset, dOffset;

        pOffset = m_Pan_err * m_Pan_p_gain;
        pOffset *= pOffset;
        if(m_Pan_err < 0)
                pOffset = -pOffset;
        dOffset = m_Pan_err_diff * m_Pan_d_gain;
        dOffset *= dOffset;
        if(m_Pan_err_diff < 0)
                dOffset = -dOffset;
//        m_PanAngle += (pOffset + dOffset);      //DIKOMEN
        m_PanAngle += pOffset;

        pOffset = m_Tilt_err * m_Tilt_p_gain;
        pOffset *= pOffset;
        if(m_Tilt_err < 0)
                pOffset = -pOffset;
        dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
        dOffset *= dOffset;
        if(m_Tilt_err_diff < 0)
                dOffset = -dOffset;
//        m_TiltAngle += (pOffset + dOffset);     //DIKOMEN
        m_TiltAngle += pOffset;

        CheckLimit();
}

void Head::Process()
{
	if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);

	if(m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);
}


