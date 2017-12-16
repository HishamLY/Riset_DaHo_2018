/**
* @file CameraCalibration.h
* Declaration of a class for representing the calibration values of the camera.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#ifndef __CAMERACALIBRATION_H__
#define __CAMERACALIBRATION_H__

#include "Vector3.h"
#include "minIni.h"

#define SECTION   "CameraCalibration"
#define INVALID_VALUE   -1024.0

namespace Robot
{
class CameraCalibration
{
private:

public:
  float cameraTiltCorrection, /**< The correction of the camera tilt angle in radians. */
  cameraRollCorrection, /**< The correction of the camera roll angle in radians. */
  cameraPanCorrection, /**< The correction of the camera pan angle in radians. */
  bodyTiltCorrection, /**< The correction of the body tilt angle in radians. */
  bodyRollCorrection; /**< The correction of the body roll angle in radians. */
  Vector3<> bodyTranslationCorrection; /**< The correction of the body translation in mm. */

    CameraCalibration();
    ~CameraCalibration();

  void LoadINISettings(minIni* ini);
  void LoadINISettings(minIni* ini, const std::string &section);
  void SaveINISettings(minIni* ini);
  void SaveINISettings(minIni* ini, const std::string &section);



  /**
  * Default constructor.

  CameraCalibration() :
    cameraTiltCorrection(0.0f),
    cameraRollCorrection(0.0f),
    cameraPanCorrection(-0.174f),
    bodyTiltCorrection(0.0f),
    bodyRollCorrection(0.0f),
    bodyTranslationCorrection(1.0f,1.0f,1.0f) {}
 */
};
}
#endif
