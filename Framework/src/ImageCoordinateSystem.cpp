/**
* @file ImageCoordinateSystem.cpp
* Implementation of a class that provides transformations on image coordinates.
* Parts of this class were copied from class ImageInfo.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
*/

#include "ImageCoordinateSystem.h"

using namespace Robot;

int ImageCoordinateSystem::xTable[640],
    ImageCoordinateSystem::yTable[480],
    ImageCoordinateSystem::table[6144];
/*int ImageCoordinateSystem::xTable[Camera::WIDTH],
    ImageCoordinateSystem::yTable[Camera::HEIGHT],
    ImageCoordinateSystem::table[6144];
*/
