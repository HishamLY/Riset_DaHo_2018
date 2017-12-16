/*******************************************************************************
*
* File : Localization.cpp
* @author : Imre Nagi (imre.here@yahoo.co.id)
*           Institut Teknologi Bandung
* Localization System using particle filter based on monte carlo localization
* and k-means clustering
*
********************************************************************************/

#include "Localization.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
//#include <random>
//#include <chrono>

using namespace Robot;
//using namespace std;

vector<Sample> Localization::Samples;
vector<Cluster> Localization::Clusters;
vector<Landmark> Localization::Landmarks;
vector<Sensor> Localization::Sensors;
vector<int> Localization::Weight;

Landmark::Landmark()
{
}

Landmark::~Landmark()
{
}




Localization::Localization(double var, double var2, int fwidth, int flength)
{
  border_strip_width = 70;
  srand((unsigned)time(0));

  variansi  = var;
  variansi2 = var2;
  field_width  = fwidth;
  field_height = flength;

  robotPosition.translation.x =(double)((rand()%field_width)+border_strip_width); //15 lebar border
  robotPosition.translation.y =(double)((rand()%field_height)+border_strip_width);
  robotPosition.rotation =(double)(rand()%360); //arah = (double)(rand()%360);
  K = 10;
  radius = 25;
  bobotmax = 0;
  hipotesis.translation.x = (double)((rand()%field_width)+border_strip_width); //hipotesisx = 0;
  hipotesis.translation.y = (double)((rand()%field_height)+border_strip_width); //hipotesisy = 0;
  hipotesis.rotation =(double)(rand()%360); //hipotesisa = 0;
  hipotesisn = 0;

  OppCenterPost.x=670; OppCenterPost.y= 270;
  OppRightPost.x=670;  OppRightPost.y = 345;
  OppLeftPost.x=670;   OppLeftPost.y  = 195;
  OwnCenterPost.x=70; OwnCenterPost.y = 270;
  OwnRightPost.x=70;  OwnRightPost.y = 195;
  OwnLeftPost.x= 70;   OwnLeftPost.y = 345;
  centerCircle.x= 370;  centerCircle.y = 270;

  Landmarks.push_back(OppCenterPost);
  Landmarks.push_back(OppRightPost);
  Landmarks.push_back(OppLeftPost);
  Landmarks.push_back(OwnCenterPost);
  Landmarks.push_back(OwnRightPost);
  Landmarks.push_back(OwnLeftPost);
  Landmarks.push_back(centerCircle);

}

void Localization::Init()
{
  //printf("%i,%i,%i,%i\n",total_samples, window_width, window_height, border_strip_width );
  BuatSampel(total_samples);
  BuatCluster();
  IsiCluster();
}

void Localization::BuatSampel(int jumlahsampel)
{
  // GENERATING RANDOM SAMPLE samples
  srand((unsigned)time(0));
  Samples.clear();

  Sample temp;
  for(int index=0; index<jumlahsampel; index++)
    {
      temp.Pose.translation.x = (double)((rand()%field_width)+border_strip_width);
      temp.Pose.translation.y = (double)((rand()%field_height)+border_strip_width);
      temp.Pose.rotation = (double)(rand()%360);
      temp.prob = 1;
      temp.cluster_i = K+1;
      Samples.push_back(temp);
      Weight.push_back(1);
    }
}

void Localization::PrintSampel()
{
  int index;
  for(index=0; index<Samples.size(); index++)
    {
      printf("%f\t%f\t%f\t%f\t%i\n", Samples[index].Pose.translation.x,
             Samples[index].Pose.translation.y,
             Samples[index].Pose.rotation,
             Samples[index].prob,
             Samples[index].cluster_i);
    }
}

void Localization::PrintSensor()
{
    int idx;
    for (idx = 0; idx < Sensors.size(); idx++)
    {
        printf("%f \t %f \t %i\n", Sensors[idx].jarak, Sensors[idx].arah, Sensors[idx].n);
    }
}

void Localization::PrintCluster()
{
  int index;
  for(index=0; index<Clusters.size(); index++)
    {
      printf("%f\t%f\t%f\t%i\t%f\t%f\t%f\n", Clusters[index].centroid.translation.x,
             Clusters[index].centroid.translation.y,
             Clusters[index].centroid.rotation,
             Clusters[index].nSample,
             Clusters[index].mean_centroid.translation.x,
             Clusters[index].mean_centroid.translation.y,
             Clusters[index].mean_centroid.rotation);
    }
}

//gunanya untuk bikin cluster dari titik sampel yang acak.
void Localization::BuatCluster()
{
  Clusters.clear();
  Cluster temp;
  int startindex = rand()%(Samples.size()-1);
  double jarak;
  int j = 0;
  int flagCluster;
  int currentindex = (startindex + 1) % Samples.size();
  while ((currentindex != startindex) and (j < K))
    {
      flagCluster = 1;
      if (Clusters.size() == 0)  //cluster doesnt exist
        {
          temp.centroid = Samples[currentindex].Pose;
          temp.nSample = 0;
          temp.mean_centroid = Pose2D(0,0,0);
          Clusters.push_back(temp);
          flagCluster = 0;
          j = j+1;  //inc the number of cluster
        }
      else
        {
          for (int i=0; i<Clusters.size(); i++) //untuk setiap sampel, periksa jarak ke seluruh cluster
            {
              jarak = sqrt(pow((Clusters[i].centroid.translation.x-Samples[currentindex].Pose.translation.x),2) +
                           pow((Clusters[i].centroid.translation.y-Samples[currentindex].Pose.translation.y),2) +
                           pow((Clusters[i].centroid.rotation-Samples[currentindex].Pose.rotation),2));
              if (jarak < 2*radius) //kalau jarak sampel ke pusat cluster < 2*radius
                  flagCluster = 0;
            }
        }

      if (flagCluster == 1)
        {
          temp.centroid = Samples[currentindex].Pose;
          temp.nSample = 0;
          temp.mean_centroid = Pose2D(0,0,0);
          Clusters.push_back(temp);
          j = j + 1;
        }
      currentindex = (currentindex + 1) % Samples.size();
    }

  if (j != K)
    K = j+1;
}

void Localization::IsiCluster()
{
  int j = 0;
  int indexmin, maks;
  double jarak, jarakmin;
  for (int i=0; i<Samples.size(); i++)
    {
      jarakmin = 10000;
      indexmin   = Clusters.size() + 1;
      for (int k=0; k<Clusters.size(); k++)
        {
          //calculate distance between sampel to centroid of cluster

          jarak = sqrt(pow((Clusters[k].centroid.translation.x-Samples[i].Pose.translation.x),2) +
                       pow((Clusters[k].centroid.translation.y-Samples[i].Pose.translation.y),2) +
                       pow((Clusters[k].centroid.rotation-Samples[i].Pose.rotation),2));

          if (jarak < jarakmin) //mencari jarak paling dekat ke cluster yang mana
            {
              jarakmin = jarak;
              indexmin = k;
            }
        }
      Clusters[indexmin].nSample += 1;
      Clusters[indexmin].mean_centroid.translation.x += Samples[i].Pose.translation.x;
      Clusters[indexmin].mean_centroid.translation.y += Samples[i].Pose.translation.y;
      Clusters[indexmin].mean_centroid.rotation += Samples[i].Pose.rotation;
      Samples[j].cluster_i= indexmin; //sample dimasukin ke cluster ke-indexmin
      j ++;
    }

  maks = 0;
  for (int i=0; i<Clusters.size(); i++)
  {
    if (maks < Clusters[i].nSample)
    maks = Clusters[i].nSample; // sampel terbanyak yang dimiliki oleh sebuah cluster
  }

  for (int i=0; i<Clusters.size(); i++)
    {
      Clusters[i].mean_centroid.translation.x = (Clusters[i].mean_centroid.translation.x/(double)Clusters[i].nSample);
      Clusters[i].mean_centroid.translation.y = (Clusters[i].mean_centroid.translation.y/(double)Clusters[i].nSample);
      Clusters[i].mean_centroid.rotation = (Clusters[i].mean_centroid.rotation/(double)Clusters[i].nSample);
      //printf("%i#%f,%f,%f\n",i,Clusters[i].mean_centroid.x,Clusters[i].mean_centroid.y, Clusters[i].mean_centroid.angle );
      if (Clusters[i].nSample == maks)
        {
          hipotesis = Clusters[i].mean_centroid;
          hipotesisn = i;
        }
    }
}

vector<double> Localization::VariansiCluster()
{

  vector<double> variansi(Clusters.size());

  for (int i=0; i<Samples.size(); i++)
    {
      double variansix = pow((Samples[i].Pose.translation.x - Clusters[Samples[i].cluster_i].mean_centroid.translation.x),2);
      double variansiy = pow((Samples[i].Pose.translation.y - Clusters[Samples[i].cluster_i].mean_centroid.translation.y),2);
      double variansiheading = pow(min(fabs(min(Samples[i].Pose.rotation, Clusters[Samples[i].cluster_i].mean_centroid.rotation)
                                            - max(Samples[i].Pose.rotation, Clusters[Samples[i].cluster_i].mean_centroid.rotation))
                                       , (360+min(Samples[i].Pose.rotation, Clusters[Samples[i].cluster_i].mean_centroid.rotation))
                                       -max(Samples[i].Pose.rotation, Clusters[Samples[i].cluster_i].mean_centroid.rotation)),2);
      variansi[Samples[i].cluster_i] += sqrt(variansix+variansiy+ variansiheading);
    }

  for (int i =0; i<variansi.size(); i++)
    {
      variansi[i] = variansi[i]/Clusters[i].nSample;
    }
  return variansi;
}

void Localization::BacaSensor()
{
  Sensors.clear();
  int i = 0;
  float arahsementara;
  Sensor temp;
  for (int j=0; j<Landmarks.size(); j++)
    {

      if ((MATH::Jarak(Landmarks[j].x, Landmarks[j].y, robotPosition.translation.x, robotPosition.translation.y) <= 300) and ((fabs(MATH::Sudut(robotPosition.translation.x, robotPosition.translation.y, Landmarks[j].x, Landmarks[j].y) - robotPosition.rotation) <= 50)
          or (fabs(MATH::Sudut(robotPosition.translation.x, robotPosition.translation.y, Landmarks[j].x, Landmarks[j].y) - robotPosition.rotation) >= 310)))
        {
          arahsementara = (double)(MATH::roundNumber(MATH::Sudut(robotPosition.translation.x, robotPosition.translation.y, Landmarks[j].x, Landmarks[j].y) - robotPosition.rotation,1) % 360);
          if (arahsementara > 180)
            arahsementara -= 360;

          temp.jarak = MATH::Jarak(Landmarks[j].x, Landmarks[j].y, robotPosition.translation.x, robotPosition.translation.y);
          temp.arah = arahsementara;
          temp.n = i;
          Sensors.push_back(temp);
        }
      i++; //untuk netapin landmark ke-i
    }
}

void Localization::BacaSensor(vector<Sensor> SensorReading)
{
  Sensors.clear();
  Sensors = SensorReading;
}


void Localization::Odometri(double xmove, double ymove, double amove)
{

/*
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::normal_distribution<double> xdistribution (xmove,5.0);
  std::normal_distribution<double> ydistribution (ymove,5.0);
  std::normal_distribution<double> adistribution (amove,5.0);
*/
  // construct a trivial random generator engine from a time-based seed:
  int index;
  robotPosition.translation.x += (xmove) * cos(MATH::DegreesToRadians(robotPosition.rotation));
  robotPosition.translation.y -= (xmove) * sin(MATH::DegreesToRadians(robotPosition.rotation));
  robotPosition.translation.x -= (ymove) * cos(MATH::DegreesToRadians(robotPosition.rotation-90));
  robotPosition.translation.y += (ymove) * sin(MATH::DegreesToRadians(robotPosition.rotation-90));

  if (amove <0)
    {
      robotPosition.rotation -= fabs(amove);
      if (robotPosition.rotation <0)
        robotPosition.rotation+= 360;
    }
  else
    {
      robotPosition.rotation += fabs(amove);
      if (robotPosition.rotation>= 360)
        robotPosition.rotation-= 360;
    }

  if(robotPosition.translation.x > field_width+border_strip_width)
    robotPosition.translation.x = field_width+border_strip_width;
  if(robotPosition.translation.x < border_strip_width)
    robotPosition.translation.x = border_strip_width;
  if(robotPosition.translation.y > field_height+border_strip_width)
    robotPosition.translation.y = field_height+border_strip_width;
  if(robotPosition.translation.y < border_strip_width)
    robotPosition.translation.y = border_strip_width;

  for(int index=0; index<Samples.size(); index++)
    {
	/*
        xmove = xdistribution(generator);
        ymove = ydistribution(generator);
        amove = (double)((int)adistribution(generator)%360);
	*/
        xmove = MATH::randomGaussian(xmove,5.0);
        ymove = MATH::randomGaussian(ymove,5.0);
        amove = (double)((int)MATH::randomGaussian(amove,5.0)%360);

          Samples[index].Pose.translation.x += (xmove) * cos(MATH::DegreesToRadians(Samples[index].Pose.rotation));
          Samples[index].Pose.translation.y -= (xmove) * sin(MATH::DegreesToRadians(Samples[index].Pose.rotation));
          Samples[index].Pose.translation.x -= (ymove) * cos(MATH::DegreesToRadians(Samples[index].Pose.rotation-90));
          Samples[index].Pose.translation.y += (ymove) * sin(MATH::DegreesToRadians(Samples[index].Pose.rotation-90));

      if(amove < 0)
        {
          Samples[index].Pose.rotation -= fabs(amove);
          if(Samples[index].Pose.rotation < 0)
            Samples[index].Pose.rotation +=360;
        }
      else
        {
          Samples[index].Pose.rotation += fabs(amove);
          if(Samples[index].Pose.rotation >= 360)
            Samples[index].Pose.rotation -=360;
        }

      if(Samples[index].Pose.translation.x > field_width+border_strip_width)
        Samples[index].Pose.translation.x = field_width+border_strip_width;
      if(Samples[index].Pose.translation.x < border_strip_width)
        Samples[index].Pose.translation.x = border_strip_width;
      if(Samples[index].Pose.translation.y > field_height+border_strip_width)
        Samples[index].Pose.translation.y = field_height+border_strip_width;
      if(Samples[index].Pose.translation.y < border_strip_width)
        Samples[index].Pose.translation.y = border_strip_width;
    }
}

void Localization::Pembobotan()
{
  bobotmax = 0;
  Sample temp;
  vector<Sample> tempSamples; tempSamples.clear();
  for (int i=0; i<Samples.size(); i++)
    {
      double bobot = HitungBobot(Samples[i]);
      bobotmax += bobot;
      temp.Pose = Samples[i].Pose;
      temp.prob = bobotmax;
      temp.cluster_i = Samples[i].cluster_i;
      tempSamples.push_back(temp);
    }
  Samples.clear();
  Samples = tempSamples;
}

double Localization::HitungBobot(Sample sampel)
{
  double bobot=1;
  double suduttemp, suduttemp2;
  for (int i=0; i<Sensors.size(); i++)
    {
      suduttemp = MATH::Sudut(sampel.Pose.translation.x, sampel.Pose.translation.y, Landmarks[Sensors[i].n].x, Landmarks[Sensors[i].n].y)
      - sampel.Pose.rotation;

      if ((suduttemp - Sensors[i].arah) > 180)
        suduttemp2 = suduttemp - 360;
      else if ((suduttemp - Sensors[i].arah) < -180)
        suduttemp2 = suduttemp + 360;
      else
        suduttemp2 = suduttemp;

      //bobot = 1;
      /*
      bobot *= (MATH::normal_distribution_function(Sensors[i].jarak,
                MATH::Jarak(sampel.Pose.translation.x,sampel.Pose.translation.y,Landmarks[Sensors[i].n].x,Landmarks[Sensors[i].n].y),
                variansi)/
                (MATH::normal_distribution_function(
                    MATH::Jarak(sampel.Pose.translation.x,sampel.Pose.translation.y,Landmarks[Sensors[i].n].x,Landmarks[Sensors[i].n].y),
                    MATH::Jarak(sampel.Pose.translation.x,sampel.Pose.translation.y,Landmarks[Sensors[i].n].x,Landmarks[Sensors[i].n].y),
                    variansi)));

      bobot *= (MATH::normal_distribution_function(Sensors[i].arah,suduttemp2,variansi2)/
                MATH::normal_distribution_function(suduttemp2,suduttemp2,variansi2));
      */
      bobot *= MATH::normal_distribution_function(suduttemp2,Sensors[i].arah,variansi2);
      bobot *= MATH::normal_distribution_function(MATH::Jarak(sampel.Pose.translation.x,sampel.Pose.translation.y,Landmarks[Sensors[i].n].x,Landmarks[Sensors[i].n].y),Sensors[i].jarak,variansi);

    }
  return bobot;
}

void Localization::SamplingUlang()
{
  vector<Sample> tempSamples;
  int indeks,lastindeks;
  int total_erased_sample = 0;
  double r,n;
  for (int i=0; i<Samples.size(); i++)
    {
      r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
      n = r * bobotmax;
      indeks = BinarySearch(Samples, n, 0, Samples.size()-1);
      if (indeks==-1){
        total_erased_sample ++;
      }
      else
        tempSamples.push_back(Samples[indeks]);
    }

  Samples.clear();
  Samples = tempSamples;
/*
  int idxtemp;
  if (total_erased_sample >= Samples.size())
  {
    if (total_erased_sample == 400)
      BuatSampel(total_samples);
    else
      for (int i=0; i<total_erased_sample; i++)
      {
        idxtemp = rand()%Samples.size();
        Samples.push_back(Samples[idxtemp]);
      }
  }
  else
  {
    for (int i=0; i<total_erased_sample; i++)
        Samples.push_back(Samples[i]);
  }
*/

  int idxtemp;
  Sample tempSample;
  if (total_erased_sample >= Samples.size())
  {
    if (total_erased_sample == 400)
      BuatSampel(total_samples);
    else
      for (int i=0; i<total_erased_sample; i++)
      {
        idxtemp = rand()%Samples.size();
        tempSample.Pose.translation.x = Samples[idxtemp].Pose.translation.x +  MATH::randomGaussian(0,5.0);
        tempSample.Pose.translation.y = Samples[idxtemp].Pose.translation.y + MATH::randomGaussian(0,5.0);;
        tempSample.Pose.rotation =  Samples[idxtemp].Pose.rotation + (double)((int)MATH::randomGaussian(0,5.0)%360);
        tempSample.prob = Samples[idxtemp].prob;
        tempSample.cluster_i = Samples[idxtemp].cluster_i;
        Samples.push_back(tempSample);
      }
  }
  else
  {
    for (int i=0; i<total_erased_sample; i++)
    {
	      idxtemp = rand()%Samples.size();
        tempSample.Pose.translation.x = Samples[idxtemp].Pose.translation.x +  MATH::randomGaussian(0,5.0);
        tempSample.Pose.translation.y = Samples[idxtemp].Pose.translation.y + MATH::randomGaussian(0,5.0);;
        tempSample.Pose.rotation =  Samples[idxtemp].Pose.rotation + (double)((int)MATH::randomGaussian(0,5.0)%360);
        tempSample.prob = Samples[idxtemp].prob;
        tempSample.cluster_i = Samples[idxtemp].cluster_i;
        Samples.push_back(tempSample);
    }
  }

}

int Localization::BinarySearch(vector<Sample> vSamples,double n, int lo, int hi)
{

  double rangelower, rangeupper;
  if (hi == 9999)
    hi = vSamples.size() - 1;
  while (lo < hi)
    {
      int mid = (lo+hi)/2;
      if ((mid-1) < 0)
        rangelower = 0;
      else
        rangelower = vSamples[mid-1].prob;

      rangeupper = vSamples[mid].prob;
      if (n < rangelower)
        hi = mid-1;
      else if (n >= rangeupper)
        lo = mid + 1;
      else if ((n >= rangelower) and (n< rangeupper))
      {
        //printf("%f,%f,%f,%f,%i\n",vSamples[mid].PositionAndOrientation.x, vSamples[mid].PositionAndOrientation.y, vSamples[mid].PositionAndOrientation.angle,vSamples[mid].prob, vSamples[mid].cluster_i);
        return mid;
      }
    }
  return -1;

}

void Localization::KualitasCluster()
{
}

Point2D Localization::getRobotToPointOnField(Point2D pf, Point2D robotPos, double direction)
{
    Point2D pfOnField;
    double r = sqrt(pf.X*pf.X + pf.Y*pf.Y);
    double pfAngle = MATH::getAngleToPoint(pf.X, pf.Y) + direction;
    pfOnField.X =  robotPos.X + r * cos(MATH::DegreesToRadians(pfAngle));
    pfOnField.Y = robotPos.Y - r * sin(MATH::DegreesToRadians(pfAngle));
    return pfOnField;
}

void Localization::LoadINISettings(minIni* ini)
{
  LoadINISettings(ini, SECTION);
}

void Localization::LoadINISettings(minIni* ini, const std::string &section)
{
  int value = -2;
  if((value = ini->geti(section, "total_sample", INVALID_VALUE)) != INVALID_VALUE)             total_samples = value;
  if((value = ini->geti(section, "window_width", INVALID_VALUE)) != INVALID_VALUE)   window_width = value;
  if((value = ini->geti(section, "window_height", INVALID_VALUE)) != INVALID_VALUE)  window_height = value;
  if((value = ini->geti(section, "border_strip_width", INVALID_VALUE)) != INVALID_VALUE)       border_strip_width = value;
}

void Localization::SaveINISettings(minIni* ini)
{
  SaveINISettings(ini, SECTION);
}

void Localization::SaveINISettings(minIni* ini, const std::string &section)
{
}
