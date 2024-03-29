/*
 *   Compass.cpp
 *
 *   Author: KRSBI ITB
 *
 */

#include <math.h>
#include "Compass.h"


using namespace Robot;


Compass* Compass::m_UniqueInstance = new Compass();

Compass::Compass()
{
    Angle16Bit = 0;
    averageCounter = 0;
    sum = 0;
    av0 = 0;
    av1 = 0;
    atas = true;
    counter = 0;
    serialRecv = (char*)malloc(sizeof(char));

}

Compass::~Compass()
{
    delete serialRecv;
}

void Compass::getSoftwareVersion(Serial *serial)
//void Compass::getSoftwareVersion(Serial *serial, char* serialRecv)
{
    unsigned char value;
    if(serial->WriteChar(SOFTWARE_VERSION) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

    fprintf(stderr, " Software version = %5u\r", value);
}

void Compass::Calibration(Serial *serial)
//void Compass::Calibration(Serial *serial, char* serialRecv)
{
    unsigned char value;
    if(serial->WriteChar(CALIBRATE_EN1) != 1)
        exit(1);
    usleep(100e3);

    if(serial->WriteChar(CALIBRATE_EN2) != 1)
        exit(1);
    usleep(100e3);

    if(serial->WriteChar(CALIBRATE_EN3) != 1)
        exit(1);
    usleep(100e3);

    for (int i=0; i<4; i++)
    {
        int input;
        printf("put compass on the %i direction ! Ok ? (1/0)", i);
        scanf("%i",&input);
        if (input!=1)
            exit(1);

        if(serial->WriteChar(CALIBRATE) != 1)
            exit(1);
    }

    printf("CALIBRATION COMPLETE!!");
}

void Compass::getAngle8Bit(Serial *serial)
//void Compass::getAngle8Bit(Serial *serial, char* serialRecv
{
    unsigned char value;
    if(serial->WriteChar(ANGLE_8_BIT) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

//    fprintf(stderr, " Compass Angle 8 Bit = %5u\r", value);
    get8bit = value*360/255;
}

void Compass::getAngle16Bit(Serial *serial)
//void Compass::getAngle16Bit(Serial *serial, char* serialRecv)
{
    unsigned char highByte, lowByte;
    if(serial->WriteChar(ANGLE_16_BIT) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;

    highByte = serialRecv[0];
    if(serial->ReadChar(serialRecv, 10) != 1) return;

    lowByte = serialRecv[0];
    unsigned short value = (highByte << 8) | lowByte;

    //fprintf(stderr, " Compass Angle 16 Bit = %5u\r", value);
    Angle16Bit = (double)value;
}

void Compass::getPitch(Serial *serial)
//void Compass::getPitch(Serial *serial, char* serialRecv)
{
    signed char value;
    if(serial->WriteChar(PITCH) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

//    fprintf(stderr, " Pitch Angle +/- 0-85° = %5d\r", value);
    pitch = value;
}

void Compass::getRoll(Serial *serial)
//void Compass::getRoll(Serial *serial, char* serialRecv)
{
    signed char value;
    if(serial->WriteChar(ROLL) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

  //  fprintf(stderr, " Roll Angle +/- 0-85° = %5d\r", value);
    roll = value;
}

void Compass::getMagneticRawData(Serial *serial)
//void Compass::getMagneticRawData(Serial *serial, char* serialRecv)
{
    unsigned char highByte, lowByte;
    if(serial->WriteChar(MAGNETIC_DATA) != 1)
        exit(1);

    signed short value[] = {0, 0, 0};
    int i = 0;
    do
    {
        if(serial->ReadChar(serialRecv) != 1) return;

        highByte = serialRecv[0];
        if(serial->ReadChar(serialRecv) != 1) return;

        lowByte = serialRecv[0];
        value[i] = (highByte << 8) | lowByte;
    }
    while(i++ < 2);
    magX = value[0];
    magY = value[1];
    magZ = value[2];
//    fprintf(stderr, " Raw Magnetic Data = X(%5d)  Y(%5d)  Z(%5d)\r", value[0], value[1], value[2]);
//    fprintf(stderr, "%d;%d;%d\n", value[0], value[1], value[2]);
}

void Compass::getAccelerometerRawData(Serial *serial)
//void Compass::getAccelerometerRawData(Serial *serial, char* serialRecv)
{
    unsigned char highByte, lowByte;
    if(serial->WriteChar(ACCELEROMETER_DATA) != 1)
        exit(1);

    signed short value[] = {0, 0, 0};
    int i = 0;
    do
    {
        if(serial->ReadChar(serialRecv) != 1) return;

        highByte = serialRecv[0];
        if(serial->ReadChar(serialRecv) != 1) return;

        lowByte = serialRecv[0];
        value[i] = (highByte << 8) | lowByte;
    }
    while(i++ < 2);
    accX = value[0];
    accY = value[1];
    accZ = value[2];
//    fprintf(stderr, " Raw Accelerometer Data = X(%.3f)\tY(%.3f)\tZ(%.3f)\r", value[0]/pow(2,15), value[1]/pow(2,15), value[2]/pow(2,15));
}

void Compass::getAllData(Serial *serial)
{
    unsigned char highByte, lowByte;
    if(serial->WriteChar(ALL_DATA) != 1)
        exit(1);

    if(serial->ReadChar(serialRecv) != 1) return;

    highByte = serialRecv[0];
    if(serial->ReadChar(serialRecv, 10) != 1) return;

    lowByte = serialRecv[0];
    signed short value = (highByte << 8) | lowByte;

    fprintf(stderr, " Compass 16 Bit = %5d ", value);


    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

    fprintf(stderr, " Pitch +/- 0-85° = %5d ", value);

    if(serial->ReadChar(serialRecv) != 1) return;
    else    value = serialRecv[0];

    fprintf(stderr, " Roll +/- 0-85° = %5d\r", value);
}

void Compass::processData(Serial *serial)
{
    float temp,Value;
    getAccelerometerRawData(serial);
    getMagneticRawData(serial);
    getPitch(serial);
    getRoll(serial);
    
    float offsetx =  (float)delX;
    float offsety = (float)delY;
    float angle = this->angle;
    float sigma = this->sigma;

    float xhTemp, yhTemp;

    pitch *= -3.14/180;
    roll *= -3.14/180;

//    float heading1 = 180 * atan2(magY, magX)/3.14;

    float xh = magX * cos(pitch) + magZ * sin(pitch);
    float yh = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

//    float heading3 = 180 * atan2(yh, xh)/3.14;

    pitch = pitch*180/3.14;
    roll = roll*180/3.14;

    //nengahin
    xh -= offsetx;
    yh -= offsety;

     //rotasi
    xhTemp = cos(angle)*xh + sin(angle)*yh;
    yhTemp = -sin(angle)*xh + cos(angle)*yh;

//deformation
    xh = xhTemp*sigma;
    yh = yhTemp*sigma;

//rotasi
    xhTemp = cos(-angle)*xh + sin(-angle)*yh;
    yhTemp = -sin(-angle)*xh + cos(-angle)*yh;

    xh = xhTemp;
    yh = yhTemp;

    float heading2 = 180 * atan2(yh, xh)/3.14;

    if (yh >= 0)
        Value =  heading2;
    else
        Value =  (360 + heading2);

    Value = 360  - Value;


    getAngle8Bit(serial);

    if(averageCounter < 5)
    {
        sum += Value;
        av0 = sum/5;
        averageCounter++;

        if (av0 > 180) //18
          atas = true;
        else 
          atas = false;
    }
    else
    {
        if(atas)
        {
          if(Value < 90)
          {
            Value += 360;
            counter ++;
            if(counter> 15)
            {
            av0 -= 360;         
            Value -= 360;
            }
          }
          else
          {
            counter = 0;
          }
        }
        else
        {
          if(Value > 270)
          {
            Value -=360;
            counter ++;
            if(counter> 15)
            {
            av0 += 360;         
            Value += 360;
            }
          }
          else
          {
            counter = 0;
          }
        }   
        
        av1 = (av0*10+Value)/(10 + 1);
        av0 = av1;

        if (av1 > 180) //18
          atas = true;
        else 
          atas = false;

//        heading = av1;
	//heading8bit = (int)(heading/360*255);
	
	float temp; 
	if (av1 < 0)
	  temp = av1+360;
	else if (av1 > 360)
	  temp = av1-360;
	else
	  temp = av1;


	heading = temp;
	heading8bit = (int)(temp/360*255);
//      printf("%.2f;%.2f;%.2f;%.2f;%.2f\n", heading, xh, yh, magX, magY);
//	printf("\t\t\t\t\t coba : %d;%d\n",heading8bit,(int)heading);
 	}
}

void Compass::IronCalibration(Serial *serial,minIni* ini)
{
    int counter = 0;
    vector <vector <int> > data;
    while(true)
    {
        getMagneticRawData(serial);
        getPitch(serial);
        getRoll(serial);

        pitch *= -3.14/180;
        roll *= -3.14/180;

        float xh = magX * cos(pitch) + magZ * sin(pitch);
        float yh = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

        vector <int> raw;
        raw.push_back(xh);
        raw.push_back(yh);

        data.push_back(raw);
        counter++;

        float deltaX = fabs(xh - data[0][X_var]);
        float deltaY = fabs(yh - data[0][Y_var]);

        printf("%d;%d\n",(int)xh,(int)yh);
        if(counter > 200 && deltaX < 100 && deltaY < 100)
            break;

    }

    int arraySize = data.size();
    printf("data : %d\n",arraySize);
    int centered[arraySize][2];
    int rotated[arraySize][2];
    int transformed[arraySize][2];
    int returned[arraySize][2];

//find maximum and minimum data
    int maxX, maxY, minX, minY;
    maxX = minX = data[0][X_var];
    maxY = minY = data[0][Y_var];

    for(int i=0; i<arraySize; i++)
    {
        if(data[i][X_var] > maxX)
            maxX = data[i][X_var];
        if(data[i][X_var] < minX)
            minX = data[i][X_var];

        if(data[i][Y_var] > maxY)
            maxY = data[i][Y_var];
        if(data[i][Y_var] < minY)
            minY = data[i][Y_var];
    }

//centering
    //int delX = (maxX + minX)/2;
    //int delY = (maxY + minY)/2;

    delX = (maxX + minX)/2;
    delY = (maxY + minY)/2;

    for(int i=0; i<arraySize; i++)
    {
        centered[i][X_var] = data[i][X_var] - delX;
        centered[i][Y_var] = data[i][Y_var] - delY;
    }

//rotating
    int maxR = 0;
    int maxPos = 0;

    for(int i=0; i<arraySize; i++)
    {
        if(maxR < centered[i][X_var] * centered[i][X_var] + centered[i][Y_var] * centered[i][Y_var])
        {
            maxR = centered[i][X_var] * centered[i][X_var] + centered[i][Y_var] * centered[i][Y_var];
            maxPos = i;
        }
    }

    //float r = (float)sqrt(maxR);
    //float angle = asin(centered[maxPos][Y]/r);
      
    r = (float)sqrt(maxR);
    angle = asin(centered[maxPos][Y_var]/r);

    if(centered[maxPos][X_var] < 0)
        angle *= -1;

    for(int i=0; i<arraySize; i++)
    {
        rotated[i][X_var] = centered[i][X_var]*cos(angle) + centered[i][Y_var]*sin(angle);
        rotated[i][Y_var] = -centered[i][X_var]*sin(angle) + centered[i][Y_var]*cos(angle);
    }

//transforming into perfect circle
    int dataminX = abs(rotated[0][X_var]);
    int minPos = 0;

    for(int i=0; i<arraySize; i++)
    {
        if(abs(rotated[i][X_var]) < dataminX)
        {
            dataminX = abs(rotated[i][X_var]);
            minPos = i;
        }
    }

    //float q = (float)sqrt(centered[minPos][X] * centered[minPos][X] + centered[minPos][Y] * centered[minPos][Y]);
    //float sigma = q/r;

    q = (float)sqrt(centered[minPos][X_var] * centered[minPos][X_var] + centered[minPos][Y_var] * centered[minPos][Y_var]);
    sigma = q/r;


    for(int i=0; i<arraySize; i++)
    {
        transformed[i][X_var] = rotated[i][X_var]*sigma;
        transformed[i][Y_var] = rotated[i][Y_var];
    }

//rotate again to return it's angle to it's original
    for(int i=0; i<arraySize; i++)
    {
        returned[i][X_var] = transformed[i][X_var]*cos(-angle) + transformed[i][Y_var]*sin(-angle);
        returned[i][Y_var] = -transformed[i][X_var]*sin(-angle) + transformed[i][Y_var]*cos(-angle);
    }


    cout << "Delta X\t: " << delX << endl;
    cout << "Delta Y\t: " << delY << endl;
    cout << "R\t: " << r << endl;
    cout << "Q\t: " << q << endl;
    cout << "Sigma\t: " << sigma << endl;
    cout << "Angle\t: " << angle << endl;

    SaveINISettings(ini);

}


void Compass::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, COMPASS_SECTION);
}

void Compass::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
     if((value = ini->geti(section, "delX", INVALID_VALUE)) != INVALID_VALUE)             delX = value;
     if((value = ini->geti(section, "delY", INVALID_VALUE)) != INVALID_VALUE)   delY = value;
 
    double dvalue = -2.0;
    if((dvalue = ini->getd(section, "sigma", INVALID_VALUE)) != INVALID_VALUE)     sigma    = dvalue;
    if((dvalue = ini->getd(section, "angle", INVALID_VALUE)) != INVALID_VALUE)     angle = dvalue;
}

void Compass::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, COMPASS_SECTION);
}

void Compass::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "delX",              delX);
    ini->put(section,   "delY",    delY);
    ini->put(section,   "sigma", sigma);
    ini->put(section,   "angle", angle);

}
