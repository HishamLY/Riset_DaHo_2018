/*******************************************************************************
*
* File : Localization.h
* @author : Imre Nagi (imre.here@yahoo.co.id)
*           Institut Teknologi Bandung
* This file contains member of localization class for estimating robot's position
* on field
*
********************************************************************************/


#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include "MathFunction.h"
#include <algorithm>
#include "minIni.h"
#include "Point.h"
#include "Pose2D.h"

#define SECTION   "Localization"
#define INVALID_VALUE   -1024.0

namespace Robot{
    struct coord{
        double x;
        double y;
        double angle;
    };

    struct RobotPos{
        coord body;
        double cameraAngle;
    };

    struct Sample{
      Pose2D Pose; // sampel position on field
      double prob;   //probabilitas sampel
      int cluster_i; //cluster ke-i tempat penyimpanan sampel
    };

    struct Cluster{
      Pose2D centroid;   //centroid dari cluster
      int nSample;     // jumlah sampel yang disimpan di cluster
      Pose2D mean_centroid; //rata-rata centroid cluster hasil perhitungan
    };

    struct Sensor{
    	double jarak;   // jarak dari sampel/robot ke landmark
		  double arah;    // beda sudut yg dibentuk oleh vektor orientasi dan vektor landmark
		  int n;          // penanda landmark ke-n
    };


    class Landmark{
    public:
        double x,y;
        Landmark();
        ~Landmark();
    };

    class Localization
    {
        private:

            int window_width;           //ukuran panjang file gambar field
            int window_height;          //ukuran lebar file gambar field
            int field_width;            //dimensi lapangan
            int field_height;           //dimensi lapangan
            int border_strip_width;     //panjang karpet


            // asumsi blue itu gawang kita, jadi kalau nyerang jadinya itu gawang lawan.
            //Landmark LeftBluePost;
            //Landmark RightBluePost;
            //Landmark centerBluePost; /*Center point of blue goal posts*/
            //Landmark LeftYellowPost;
            //Landmark RightYellowPost;
            //Landmark centerYellowPost; /*Center point of yellow goal posts */
            //Landmark penaltyBlue; /*Penalty point on blue area*/
            //Landmark penaltyYellow; /*Penalty point on yellow area*/
            //Landmark centerCircle; /*Center circle of field*/

            Landmark OppRightPost;
            Landmark OppLeftPost;
            Landmark OppCenterPost;
            Landmark OwnRightPost;
            Landmark OwnLeftPost;
            Landmark OwnCenterPost;
            Landmark centerCircle;

        public :
            Localization(double var, double var2, int fwidth, int flength);
            ~Localization();

            int total_samples;          //jumlah sampel yang digunakan

            static vector<Cluster> Clusters;    /* array of cluster */
            static vector<Sample> Samples;      /* array of sample */
            static vector<Landmark> Landmarks;  /* array of Landmark */
            static vector<Sensor> Sensors;      /* array of Sensor */
            static vector<int> Weight;

            Pose2D variable;

            Pose2D robotPosition;
            Pose2D hipotesis;
            //coord robotPosition;  /* robot's position on field */
            //coord hipotesis;      /* estimation position of robot on field */

            double variansi;    /* variansi */
            double variansi2;
            double radius;      /* minimum distance between cluster */
            double bobotmax;
            double hipotesisn;
            int K;  /* the number of clusters */

            void Update(vector<Sensor> SensorReading);

            /**
            * Init the localization systems. Creating the samples, clusters, and fill the clusters with samples.
            */
            void Init();

            /**
            * @param jumlah sampel yang akan ditebar di lapangan
            * fungsi ini digunakan untuk membuat sampel pada posisi acak dilapangan
            */
            void BuatSampel(int jumlahsampel);

            /**
            *
            *
            */
            void BuatCluster();

            /**
            *
            *
            */
            void IsiCluster();

              /**
            * @return vector of variansi
            *
            */
            vector<double> VariansiCluster();

            /**
            *
            *
            */
            void BacaSensor();
            void BacaSensor(vector<Sensor> SensorReading);



            /**
            * @param x movement
            * @param y movement
            * @param angle movement
            * Fungsi ini digunakan untuk memindahkan sampel-sampel
            */
            void Odometri(double xmove, double ymove, double amove);

            /** @brief Fungsi untuk melakukan perhitungan bobot terhadap tempat di lapangan dengan posisi robot saat ini
              * Fungsi HitungBobot akan memberikan bobot tertentu pada sebuah tempat (landmark) di lapangan terhadap posisi robot saat ini
              * @param sample variabel untuk menampung sample-sample landmark atau tempat di lapangan
              * @return bobot variabel nilai bobot yang telah dihitung dari sampel yang telah diberikan
              */
            double HitungBobot(Sample sample);

            /** @brief Prosedur untuk melakukan pembobotan pada array Samples yang telah terdefinisi
              * I.S. bobotmax diset dengan nilai 0, variabel array yang mengandung Sample telah terdefinisi
              * F.S. variabel array Sample telah dilakukan pembobotan ulang dan ditampung di dalam variabel vector Samples
              * Pembobotan ulang dilakukan untuk mengetahui nilai terbaru (posisi terbaru) dari posisi robot saat ini
              */
            void Pembobotan();

            /** @brief Prosedur yang digunakan untuk melakukan pengambilan sampel ulang dengan menambahkan nilai elemen pose dengan distribusi random Gaussian
              * I.S. variabel Samples terdefinisi, kemudian melakukan pencarian sampel dengan BinSearch dan melakukan penghapusan sampel yang sudah tidak valid lagi
              * F.S. berdasarkan kondisi yang ada Sampel akan diambil ulang atau Sampel akan ditambah nilai elemen posenya dengan distribusi random Gaussian, Sampel baru sudah terdefinisi
              */
            void SamplingUlang();

            /** @brief Fungsi Binary Search untuk mencari di dalam sebuah vektor Samples
              * @param vSamples array/vektor yang akan dicari elemennya
              * @param n nilai yang akan dicari
              * @param lo indeks pembatas terkecil
              * @param hi indeks pembatas terbesar
              * return indeks dari elemen vektor yang nilainya n atau mendekati  n
              */
            int BinarySearch(vector<Sample> vSamples, double n, int lo=0, int hi=9999);

            /** Fungsi yang digunakan untuk mengarahkan robot ke suatu tempat di lapangan
              * @param pf titik pada kerangka koordinat robot
              * @param robotPos posisi robot saat ini (x,y)
              * @param direction orientasi robot di lapangan
              */
            Point2D getRobotToPointOnField(Point2D pf, Point2D robotPos, double direction); //Point2D gak bisa statik

            /** @brief Prosedur untuk melakukan output elemen-elemen di dalam sample ke layar
              * Print all member of each sample in Samples vector
              */
            void PrintSampel();

            /** @brief Prosedur untuk melakukan output elemen-elemen cluster (container yang lebih besar dari sample) ke layar
              * Print all member of each cluster in Clusters vector
              */
            void PrintCluster();

            /** @brief Prosedur untuk melakukan output hasil deteksi pada variabel Sensor ke layar
              * Print all member of each sensor in Sensors vector
              */
            void PrintSensor();

            /** @brief Prosedur tidak diimplementasi pada Localization.cpp
              */
            void KualitasCluster();

            /** @brief Prosedur untuk melakukan Load dan Save nilai konfigurasi kamera
              */
            void LoadINISettings(minIni* ini);
            void LoadINISettings(minIni* ini, const std::string &section);
            void SaveINISettings(minIni* ini);
            void SaveINISettings(minIni* ini, const std::string &section);
    };
}

#endif
