#include "CameraCalibration.h"

using namespace Robot;

CameraCalibration::CameraCalibration()
{
  cameraTiltCorrection = 0.0f;
  cameraRollCorrection = 0.0f;
  cameraPanCorrection = 0.0f;
  bodyTiltCorrection = 0.0f;
  bodyRollCorrection = 0.0f;
  bodyTranslationCorrection = Vector3<>(1.0f, 1.0f, 1.0f);
}

CameraCalibration::~CameraCalibration()
{
}

void CameraCalibration::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, SECTION);
}

void CameraCalibration::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    double dvalue = -2.0;
    if((dvalue = ini->getd(section, "cameraTiltCorrection", INVALID_VALUE)) != INVALID_VALUE)    cameraTiltCorrection = dvalue;
    if((dvalue = ini->getd(section, "cameraRollCorrection", INVALID_VALUE)) != INVALID_VALUE)    cameraRollCorrection = dvalue;
    if((dvalue = ini->getd(section, "cameraPanCorrection", INVALID_VALUE)) != INVALID_VALUE)     cameraPanCorrection  = dvalue;
    if((dvalue = ini->getd(section, "bodyTiltCorrection", INVALID_VALUE)) != INVALID_VALUE)     bodyTiltCorrection    = dvalue;
    if((dvalue = ini->getd(section, "bodyRollCorrection", INVALID_VALUE)) != INVALID_VALUE)     bodyRollCorrection    = dvalue;
    if((dvalue = ini->getd(section, "bodyTranslationCorrectionX", INVALID_VALUE)) != INVALID_VALUE)     bodyTranslationCorrection.x = dvalue;
    if((dvalue = ini->getd(section, "bodyTranslationCorrectionY", INVALID_VALUE)) != INVALID_VALUE)     bodyTranslationCorrection.y = dvalue;
    if((dvalue = ini->getd(section, "bodyTranslationCorrectionZ", INVALID_VALUE)) != INVALID_VALUE)     bodyTranslationCorrection.z = dvalue;
}

void CameraCalibration::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, SECTION);
}

void CameraCalibration::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "cameraTiltCorrection",              cameraTiltCorrection);
    ini->put(section,   "cameraRollCorrection",    cameraRollCorrection);
    ini->put(section,   "cameraPanCorrection",   cameraPanCorrection);
    ini->put(section,   "bodyTiltCorrection",        bodyTiltCorrection);
    ini->put(section,   "bodyRollCorrection",        bodyRollCorrection);
    ini->put(section,   "bodyTranslationCorrectionX",        bodyTranslationCorrection.x);
    ini->put(section,   "bodyTranslationCorrectionY",      bodyTranslationCorrection.y);
    ini->put(section,   "bodyTranslationCorrectionZ",      bodyTranslationCorrection.z);
}
