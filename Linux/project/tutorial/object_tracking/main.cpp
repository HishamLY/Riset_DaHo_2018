/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"
#include "DebugDrawings.h"

#define INI_FILE_PATH       "config.ini"

using namespace std;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Object Tracking Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* orangeFinder = new ColorFinder();
    ColorFinder* greenFinder = new ColorFinder();
    ColorFinder* whiteFinder = new ColorFinder();

    orangeFinder->LoadINISettings(ini, "Orange");
    greenFinder->LoadINISettings(ini, "Green");
    whiteFinder->LoadINISettings(ini, "White");

    httpd::ball_finder = orangeFinder;
    httpd::red_finder = greenFinder;

    int PixelNumber;
    Point2D pt, pt0, pt1;

    ColorRGB yellow(ColorClasses::yellow);
    ColorRGB red(ColorClasses::red);
    ColorRGB white(ColorClasses::white);

    while(1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(frame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        greenFinder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        orangeFinder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        whiteFinder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

        vector<Point2D> fieldBorderPoints = greenFinder->getConvexFieldBorders(10, 20);
        vector<Point2D> listBlob = orangeFinder->getBlobCenter(frame, fieldBorderPoints);
        vector<Point2D> listLine = whiteFinder->linePercept(fieldBorderPoints, 10);

        for(int i = 0; i < frame->m_NumberOfPixels; i++)
        {
            if(greenFinder->m_result->m_ImageData[i] == 1)
            {
                frame->m_ImageData[i*frame->m_PixelSize + 0] = 0;
                frame->m_ImageData[i*frame->m_PixelSize + 1] = 255;
                frame->m_ImageData[i*frame->m_PixelSize + 2] = 0;
            }
        }

        for(vector<Point2D >::iterator iter = fieldBorderPoints.begin(); iter != fieldBorderPoints.end(); iter++)
        {
            PixelNumber = ((*iter).Y)*frame->m_Width+((*iter).X);
            Draw::SetPixel(frame, PixelNumber, red);
        }

        for(vector<Point2D >::iterator iter = listBlob.begin(); iter != listBlob.end(); iter++)
        {
            if((*iter).X!=-1 && (*iter).Y!=-1)
            {
                pt.X = (*iter).X; pt.Y = (*iter).Y;
                Draw::Circle(frame, pt, 3, red);
            }
        }

        for(vector<Point2D >::iterator iter = listLine.begin(); iter != listLine.end(); iter++)
        {
            if((*iter).X!=-1 && (*iter).Y!=-1)
            {
                pt0.X = (*iter).X; pt0.Y = (*iter).Y;
                pt1.X = (*(++iter)).X; pt1.Y = (*iter).Y;
                Draw::Line(frame, pt0, pt1, white);
            }
        }

        streamer->send_image(frame);
    }

    return 0;
}
