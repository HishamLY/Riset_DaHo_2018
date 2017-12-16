###############################################################
#
# Purpose: Makefile for "DARwIn Linux Framework"
# Author.: robotis
# Version: 0.1
# License: GPL
# Modified: Imre
# This makefile only compiles NOBJS object, but linking NOBJS+OBJS for the execution file
#
###############################################################

CC = g++
AR = ar
ARFLAGS = cr

TARGET = darwin.a

INCLUDE_DIRS = -I../include -I../../Framework/include

CXXFLAGS += -O2 -DLINUX -Wall -shared -g $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall -shared $(INCLUDE_DIRS)
LFLAGS += -lpthread -ldl 
#-std=c++0x  -std=gnu++0x

NOBJS =        streamer/httpd.o \
 	../../Framework/src/vision/ColorFinder.o \
	../../Framework/src/Imu.o \
	../../Framework/src/Behavior.o          \
        ../../Framework/src/vision/BallFollower.o   \
        ../../Framework/src/vision/BallTracker.o    \
	../../Framework/src/GameController.o	\
        ../../Framework/src/motion/modules/Head.o   \
        ../../Framework/src/motion/modules/Walking.o\
	../../Framework/src/CameraCalibration.o \
 	 ../../Framework/src/Compass.o	\
        ../../Framework/src/vision/ImgProcess.o 	\
       ../../Framework/src/Serial.o	\
	../../Framework/src/CameraMatrix.o \
	../../Framework/src/LinePerceptor.o \
	../../Framework/src/LinePercept.o \
	../../Framework/src/Geometry.o \
        ../../Framework/src/DebugDrawings.o     \
        ../../Framework/src/CameraMatrixProvider.o \
        ../../Framework/src/FieldDimensions.o   \
        ../../Framework/src/ImageCoordinateSystem.o \
        ../../Framework/src/TeamCommunication.o \
        ../../Framework/src/RegionAnalyzer.o    \
        ../../Framework/src/Regionizer.o        \
        ../../Framework/src/RegionPercept.o     \
        ../../Framework/src/RobotModelProvider.o\
        ../../Framework/src/RobotModel.o        \
        ../../Framework/src/RotationMatrix.o    \
        ../../Framework/src/TorsoMatrixProvider.o \
        ../../Framework/src/RobotCameraMatrixProvider.o \
        ../../Framework/src/MVTools.o           \
        ../../Framework/src/StatusCheck.o       \
        ../../Framework/src/PointExplorer.o     \
	../../Framework/src/CM730.o     	\
	../../Framework/src/Perception.o \
	../../Framework/src/math/Matrix.o   \
        ../../Framework/src/math/Plane.o    \
        ../../Framework/src/math/Point.o    \
        ../../Framework/src/math/Vector.o   \
        ../../Framework/src/math/MathFunction.o   \
        ../../Framework/src/motion/JointData.o  	\
        ../../Framework/src/motion/Kinematics.o\
        ../../Framework/src/motion/MotionManager.o  \
        ../../Framework/src/motion/MotionStatus.o   \
        ../../Framework/src/motion/modules/Action.o \
        ../../Framework/src/vision/Image.o  		\
	../../Framework/src/vision/Camera.o	\
        ../../Framework/src/minIni/minIni.o	\
        streamer/jpeg_utils.o      \
        streamer/mjpg_streamer.o   \
        LinuxCamera.o \
        LinuxActionScript.o   \
        LinuxCM730.o    \
        LinuxMotionTimer.o    \
        LinuxNetwork.o	\
	ReadersWritersLock.o	\

OBJS =  ../../Framework/src/Localization.o \



$(TARGET): $(OBJS) $(NOBJS)
	$(AR) $(ARFLAGS) ../lib/$(TARGET) $(OBJS) $(NOBJS)
#	$(CC) -o $(TARGET) $(NOBJS) $(OBJS) $(LFLAGS) 

all: $(TARGET)

clean:
	rm -f $(NOBJS) ../lib/$(TARGET)
