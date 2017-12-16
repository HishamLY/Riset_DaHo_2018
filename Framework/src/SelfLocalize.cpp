#include "SelfLocalize.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "Image.h"

using namespace Robot;

coord SelfLocalize::lline {0, 0, 0};
coord SelfLocalize::rline {0, 0, 0};
coord SelfLocalize::hline {0, 0, 0};
coord SelfLocalize::tline {0, 0, 0};

int SelfLocalize::line_angle_from_heading = 20;     // SET ANGLE OF SIGHT FROM STRAIGHT HEADING 20 degree

vector<Sample> SelfLocalize::Samples;
Sample SelfLocalize::Solution;

void SelfLocalize::Init()
{
  srand((unsigned)time(0));
  Robot.x = (rand()%field_width)+border_strip_width;
  Robot.y = (rand()%field_height)+border_strip_width;
  Robot.angle = (rand()%360);
}

/**
* @param MaxProbability -> probabilitas maksimum dari masing-masing sample
* @param TotalSamples   -> jumlah particle sample yang ada dilapangan
* prosedur ini digunakan untuk menempatkan random sample dilapangan
*/
void SelfLocalize::generateRandomSamples( double MaxProbability, int TotalSamples)
{
  // GENERATING RANDOM SAMPLE samples
  srand((unsigned)time(0));

  int random_x;
  int random_y;
  int random_angle;
  int index;

  Sample temp;
  for(index=0; index<TotalSamples; index++)
    {
      random_x = (rand()%field_width)+border_strip_width;
      random_y = (rand()%field_height)+border_strip_width;
      random_angle = (rand()%360);

      temp.PositionAndOrientation.x = random_x;
      temp.PositionAndOrientation.y = random_y;
      temp.PositionAndOrientation.angle = random_angle;
      temp.prob = MaxProbability/TotalSamples;
      Samples.push_back(temp);
    }
}

void SelfLocalize::paintModel()
{

}

void SelfLocalize::getSolution()
{
    double maxProb = -1.0;
    int idx;
    for(int index = 0; index < Samples.size(); index++)
    {
      if (maxProb <= Samples[index].prob)
      {
        idx = index;
        maxProb = Samples[index].prob;
      }
    }
    //printf("%i, %i, %i\n",Samples[idx].PositionAndOrientation.x, Samples[idx].PositionAndOrientation.y, Samples[idx].PositionAndOrientation.angle);
    SelfLocalize::Solution = Samples[idx];
}

MATH::Scalar SelfLocalize::check_landmark()
{

}

/**
* @param x, x position of the sample
* @param y, y position of the sample
* @param heading_angle, heading dari badan robot
* Fungsi mengembalikan scalar yang berisi hasil pembacaan dari sample
* pada fungsi ini dilakukan penafsiran vision untuk masing-masing sample
* dengan heading tertentu dan garis heading tertentu akan dilihat apakah landmark
* masuk dalam jangkauan penglihatan
*/
MATH::Scalar SelfLocalize::check_landmark(  int x, int y, int heading_angle )
{
  int yleft, yright, yheading;
  int x2, y2, temp;

  // HEADING LINE
  x2 = x+1000*cos(MATH::DegreesToRadians(heading_angle));
  y2 = y+1000*sin(MATH::DegreesToRadians(heading_angle));

//=== FIND END HEADING POINT IN EDGE SCREEN
  if(heading_angle >= 0 && heading_angle <= 90)
    {
      yheading = MATH::find_y(x, y, x2, y2, 685);
      if(yheading >= 485)
        {
          hline.y = 485;
          hline.x = MATH::find_x(x, y, x2, y2, 485);
        }
      else if(yheading < 485)
        {
          hline.x = 685;
          hline.y = MATH::find_y(x, y, x2, y2, 685);
        }
    }
  else if(heading_angle > 90 && heading_angle < 180)
    {
      yheading = MATH::find_y(x, y, x2, y2, 85);
      if(yheading >= 485)
        {
          hline.y = 485;
          hline.x = MATH::find_x(x, y, x2, y2, 485);
        }
      else if(yheading < 485)
        {
          hline.x = 85;
          hline.y = MATH::find_y(x, y, x2, y2, 85);
        }
    }
  else if(heading_angle >= 180 && heading_angle < 270)
    {
      yheading = MATH::find_y(x, y, x2, y2, 85);
      if(yheading >= 85)
        {
          hline.x = 85;
          hline.y = MATH::find_y(x, y, x2, y2, 85);
        }
      else if(yheading < 85)
        {
          hline.y = 85;
          hline.x = MATH::find_x(x, y, x2, y2, 85);
        }
    }
  else if(heading_angle >= 270 && heading_angle < 360)
    {
      yheading = MATH::find_y(x, y, x2, y2, 685);
      if(yheading >= 85)
        {
          hline.x = 685;
          hline.y = MATH::find_y(x, y, x2, y2, 685);
        }
      else if(yheading < 85)
        {
          hline.y = 85;
          hline.x = MATH::find_x(x, y, x2, y2, 85);
        }
    }

//============

  if(heading_angle >= 0 && heading_angle <= 90)
    {
      yheading = MATH::find_y(x, y, x2, y2, 625);
      if(yheading >= 485)
        {
          tline.y = 485;
          tline.x = MATH::find_x(x, y, x2, y2, 485);
        }
      else if(yheading < 485)
        {
          tline.x = 625;
          tline.y = MATH::find_y(x, y, x2, y2, 625);
        }
    }
  else if(heading_angle > 90 && heading_angle < 180)
    {
      yheading = MATH::find_y(x, y, x2, y2, 83);
      if(yheading >= 415)
        {
          tline.y = 415;
          tline.x = MATH::find_x(x, y, x2, y2, 415);
        }
      else if(yheading < 415)
        {
          tline.x = 83;
          tline.y = MATH::find_y(x, y, x2, y2, 83);
        }
    }
  else if(heading_angle >= 180 && heading_angle < 270)
    {
      yheading = MATH::find_y(x, y, x2, y2, 83);
      if(yheading >= 83)
        {
          tline.x = 83;
          tline.y = MATH::find_y(x, y, x2, y2, 83);
        }
      else if(yheading < 83)
        {
          tline.y = 15;
          tline.x = MATH::find_x(x, y, x2, y2, 83);
        }
    }
  else if(heading_angle >= 270 && heading_angle < 360)
    {
      yheading = MATH::find_y(x, y, x2, y2, 555);
      if(yheading >= 83)
        {
          tline.x = 555;
          tline.y = MATH::find_y(x, y, x2, y2, 555);
        }
      else if(yheading < 83)
        {
          tline.y = 15;
          tline.x = MATH::find_x(x, y, x2, y2, 83);
        }
    }
////////////

  // SET ANGLE OF SIGHT FROM STRAIGHT HEADING 20 degree left and 20 degree right, TOTAL ANGLE OF SIGHT IS 40 degree
  int left_line_angle = heading_angle - line_angle_from_heading;
  int right_line_angle = heading_angle + line_angle_from_heading;
  if(right_line_angle >= 360) right_line_angle -= 360;
  if(left_line_angle < 0) left_line_angle += 360;


  // RIGHT SIGHT LINE LIMIT
  // yright = batas ujung penglihatan sebelah kanan (posisi y)
  rline.x = x+1000*cos(MATH::DegreesToRadians(heading_angle+line_angle_from_heading));
  rline.y = y+1000*sin(MATH::DegreesToRadians(heading_angle+line_angle_from_heading));
  //g_message("LINERIGHT X=%d Y=%d;", rline.x, rline.y);
  temp = (rline.x - x) == 0 ? 1 : (rline.x - x);
  if((right_line_angle <= 90 && right_line_angle >= 0) || (right_line_angle > 270 && right_line_angle <= 360))
    yright = (rline.y-y)*(field_width+border_strip_width-x)/temp+y;
  else if(right_line_angle > 90 && right_line_angle <= 270)
    yright = (rline.y-y)*(border_strip_width-x)/temp+y;
  //g_message("YRIGHT %d;", yright);

  // LEFT SIGHT LINE LIMIT
  // yleft = batas ujung penglihatan sebelah kiri (posisi y)
  lline.x = x+1000*cos(MATH::DegreesToRadians(heading_angle-line_angle_from_heading));
  lline.y = y+1000*sin(MATH::DegreesToRadians(heading_angle-line_angle_from_heading));
  //g_message("LINELEFT X=%d Y=%d;", lline.x, lline.y);
  temp = (lline.x-x) == 0 ? 1 : (lline.x-x);
  if((left_line_angle <= 90 && left_line_angle >= 0) || (left_line_angle > 270 && left_line_angle <= 360))
    yleft = (lline.y-y)*(field_width+border_strip_width-x)/temp+y;
  else if(left_line_angle > 90 && left_line_angle <= 270)
    yleft = (lline.y-y)*(border_strip_width-x)/temp+y;
  //g_message("YLEFT %d;", yleft);

  MATH::Scalar number;
  // CHECK IF FIND SOME GOAL
  // RETURNING WHICH GOAL IS FIND IF (1) YELLOW GOAL FIND ELSE IF (2) BLUE GOAL FIND ELSE (0) NO GOAL FIND
  // AND RETURNING HOW MUCH PART OF GOAL HAS BEEN SEEN (SENSOR READING)

  // nentuin part tiang mana yang kelihatan sama penglihatan si robot.
  if(yright > yleft)
    {
      if(yleft <= y_goal_bottom && yleft >= y_goal_top && yright > y_goal_bottom)
        {
          /*g_message("GOAL RIGHT Found RIGHT SIDE;");*/ number.first_number = YELLOW_GOAL;
          number.second_number = RIGHT_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-438),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yright <= y_goal_bottom && yright >= y_goal_top && yleft < y_goal_top)
        {
          /*g_message("GOAL RIGHT Found LEFT SIDE;");*/ number.first_number = YELLOW_GOAL;
          number.second_number = LEFT_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-438),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yleft < y_goal_top && yright > y_goal_bottom)
        {
          /*g_message("GOAL RIGHT Found ENTIRE SIDE;");*/ number.first_number = YELLOW_GOAL;
          number.second_number = MIDDLE_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-438),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yleft >= y_goal_top && yright <= y_goal_bottom)
        {
          /*g_message("GOAL RIGHT Found INNER SIDE;");*/ number.first_number = YELLOW_GOAL;
          number.second_number = MIDDLE_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-438),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else//{
        // if (Movement == translate_left || Movement == rotate_left) {
        {
          number.first_number = NO_GOAL;
          number.second_number = UNFOUND;
          number.third_number = 0.0;
          number.four_number = (sqrt(pow(double(x-438),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      //return {NO_GOAL, 0.0};
      //}
      //  else

    }
  //}
  else if(yright < yleft)
    {
      if(yleft <= y_goal_bottom && yleft >= y_goal_top && yright < y_goal_top)
        {
          /*g_message("GOAL LEFT Found RIGHT SIDE;");*/
          number.first_number = BLUE_GOAL;
          number.second_number = RIGHT_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-198),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yright <= y_goal_bottom && yright >= y_goal_top && yleft > y_goal_bottom)
        {
          /*g_message("GOAL LEFT Found LEFT SIDE;");*/ number.first_number = BLUE_GOAL;
          number.second_number = LEFT_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number =(sqrt(pow(double(x-198),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yright < y_goal_top && yleft > y_goal_bottom)
        {
          /*g_message("GOAL LEFT Found ENTIRE SIDE;");*/ number.first_number = BLUE_GOAL;
          number.second_number = MIDDLE_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-198),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else if(yright >= y_goal_top && yleft <= y_goal_bottom)
        {
          /*g_message("GOAL LEFT Found INNER SIDE;");*/
          number.first_number = BLUE_GOAL;
          number.second_number = MIDDLE_PART;
          number.third_number = sqrt(pow(double(x-hline.x),2.0)+pow(double(y-hline.y),2.0));
          number.four_number = (sqrt(pow(double(x-198),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      else
        {
          number.first_number = NO_GOAL;
          number.second_number = UNFOUND;
          number.third_number = 0.0;
          number.four_number = (sqrt(pow(double(x-198),2.0)+pow(double(y-216),2.0)));
          return (number);
        }
      //return {NO_GOAL, 0.0};
    }
}

/**
* @param TypeOfGoal = warna gawang yang dilihat
* @param PartOfGoalPostSeen = tiang mana yang dilihat
* @param Distance = jarak terhadap objek
* @param Dis = i dont know what it is
* Prosedur ini digunakan untuk melakukan pembacaan vision tiap sample dan melakukan
* pencocokan vision sample dengan hasil vision dari robot
*/
void SelfLocalize::checkSensorModel(int TypeOfGoal, int PartOfGoalPostSeen, double Distance, double Dis)
{
  double sum = 0;
  int index =0;
  vector<double>Sensor_probability;
  Jarak = Distance;
  Jarak1 = Dis;

  for(int index = 0; index < Samples.size(); index++)
    {
      // pengambilan pembacaan sensor untuk setiap sample
      SampleTypeOfGoal = SelfLocalize::check_landmark(Samples[index].PositionAndOrientation.x,
                         Samples[index].PositionAndOrientation.y,
                         Samples[index].PositionAndOrientation.angle).first_number;
      SamplePartOfGoalPostSeen = SelfLocalize::check_landmark(Samples[index].PositionAndOrientation.x,
                                 Samples[index].PositionAndOrientation.y,
                                 Samples[index].PositionAndOrientation.angle).second_number;
      //sampleDistance hasil pengukuran masing-masing sample
      SampleDistance = SelfLocalize::check_landmark(Samples[index].PositionAndOrientation.x,
                       Samples[index].PositionAndOrientation.y,
                       Samples[index].PositionAndOrientation.angle).third_number;
      SampleDis = SelfLocalize::check_landmark(Samples[index].PositionAndOrientation.x,
                  Samples[index].PositionAndOrientation.y,
                  Samples[index].PositionAndOrientation.angle).four_number;


      // melakukan pencocokan hasil dari vision robot terhadap masing-masing vision dari sample
      if(TypeOfGoal == YELLOW_GOAL)
        {
          if(SampleTypeOfGoal == YELLOW_GOAL)
            {
              if ((PartOfGoalPostSeen == LEFT_PART && SamplePartOfGoalPostSeen == LEFT_PART) || (PartOfGoalPostSeen == RIGHT_PART && SamplePartOfGoalPostSeen == RIGHT_PART) || (PartOfGoalPostSeen == MIDDLE_PART && SamplePartOfGoalPostSeen == MIDDLE_PART) || (PartOfGoalPostSeen == BOTH_PART && SamplePartOfGoalPostSeen == BOTH_PART) || (PartOfGoalPostSeen == UNFOUND && SamplePartOfGoalPostSeen == UNFOUND) || (PartOfGoalPostSeen == UNIDENTIFIED && SamplePartOfGoalPostSeen == UNIDENTIFIED) && SampleDistance == Distance)
                {
                  Sensor_probability.push_back(MATH::normal_distribution_function(SampleDistance, Distance, 0.1));
                }
              else
                {
                  Sensor_probability.push_back(0.0001);
                }
            }
          else
            {
              Sensor_probability.push_back(0.0001);
            }
        }
      else if(TypeOfGoal == BLUE_GOAL )
        {
          if(SampleTypeOfGoal == BLUE_GOAL)
            {
              if ((PartOfGoalPostSeen == LEFT_PART && SamplePartOfGoalPostSeen == LEFT_PART) || (PartOfGoalPostSeen == RIGHT_PART && SamplePartOfGoalPostSeen == RIGHT_PART) || (PartOfGoalPostSeen == MIDDLE_PART && SamplePartOfGoalPostSeen == MIDDLE_PART) || (PartOfGoalPostSeen == BOTH_PART && SamplePartOfGoalPostSeen == BOTH_PART) || (PartOfGoalPostSeen == UNFOUND && SamplePartOfGoalPostSeen == UNFOUND) || (PartOfGoalPostSeen == UNIDENTIFIED && SamplePartOfGoalPostSeen == UNIDENTIFIED) && SampleDistance == Distance)
                {
                  Sensor_probability.push_back(MATH::normal_distribution_function(SampleDistance, Distance, 0.1));
                }
              else
                {
                  Sensor_probability.push_back(0.0001);
                }
            }
          else
            {
              Sensor_probability.push_back(0.0001);
            }
        }
      else if(TypeOfGoal == NO_GOAL)
        {
          if(SampleTypeOfGoal == NO_GOAL)
            {
              Sensor_probability.push_back(MATH::normal_distribution_function(SampleDistance, Distance, 0.1));
            }
          else
            {
              Sensor_probability.push_back(0.0001);
            }
        }
      sum += (Samples[index].prob * Sensor_probability.back());
    }

  for(int index = 0; index < Samples.size(); index++)
    {
        //bayesian ?
      Samples[index].prob = ( Samples[index].prob * Sensor_probability[index]) / sum;
    }
}


void SelfLocalize::resampling()
{
  double total_erased_prob = 0;
  int total_erased_circles = 0;

  printf("Sample Size BEfore Delete= %i\n", Samples.size());

  for(vector<Sample>::iterator it = Samples.begin(); it < Samples.end(); it++)
    {
      if((*it).prob < 0.001)
        {
          total_erased_prob += (*it).prob;
          Samples.erase(it);
          total_erased_circles++;
        }
    }

  printf("total erased circle = %i , prob = %f \n", total_erased_circles, total_erased_prob);

  vector<int> N;
for(vector<Sample>::iterator it = Samples.begin(); it < Samples.end(); it++)
    {
      N.push_back(MATH::roundNumber(total_erased_circles * ((*it).prob), 1));
    }

  int size = Samples.size();
  printf("Sample Size = %i ------ vector size = %i \n", size , N.size());
  for(int i = 0; i < size; i++)
    {
      for(int j = 0; j < N[i]; j++)
        {
          int random_x = (rand()%(2*delta_x_clusters))+(Samples[i].PositionAndOrientation.x-delta_x_clusters);

          if(random_x < border_strip_width)
            random_x = border_strip_width;
          else if(random_x > field_width+border_strip_width)
            random_x = field_width+border_strip_width;

          int random_y = (rand()%(2*delta_y_clusters))+(Samples[i].PositionAndOrientation.y-delta_y_clusters);

          if(random_y < border_strip_width)
            random_y = border_strip_width;
          else if(random_y > field_height+border_strip_width)
            random_y = field_height+border_strip_width;

          int random_angle = (rand()%(2*delta_angle_clusters))+(Samples[i].PositionAndOrientation.angle-delta_angle_clusters);

          if(random_angle > 360)
            random_angle -= 360;
          else if(random_angle < 0)
            random_angle += 360;

          Sample temp;
          temp.PositionAndOrientation.x = random_x;
          temp.PositionAndOrientation.y = random_y;
          temp.PositionAndOrientation.angle = random_angle;
          temp.prob = total_erased_prob/total_erased_circles;
          Samples.push_back(temp);
        }
    }

  printf("Sample Size After resampling = %i\n", Samples.size());
}

void SelfLocalize::addSamples()
{
  int temp_total_circles = 0;
  double temp_total_prob = 0;
  for(vector<Sample>::iterator it = Samples.begin(); it < Samples.end(); it++)
    {
      temp_total_circles++;
      temp_total_prob += (*it).prob;
    }
    printf("temp total circle %i \n", temp_total_circles);
  if(temp_total_circles < total_samples)
    {
      if (SampleDistance != Jarak  || SampleDis != Jarak1 )
        {
          generateRandomSamples( 1.000, total_samples);
        }
      if(temp_total_prob < 1.00)
        {
          generateRandomSamples(1.000 - temp_total_prob, total_samples - temp_total_circles);
        }
    }
  printf("SAMPLE SIZE :%d\tTOTAL PROB. :%f\n", temp_total_circles, temp_total_prob);
}


void SelfLocalize::Odometri(double xmove, double ymove, double amove)
{
    if (xmove >= 0)
      {
        Robot.x += xmove * cos(MATH::DegreesToRadians(Robot.angle));
        Robot.y += xmove * sin(MATH::DegreesToRadians(Robot.angle));
      }
      else if(xmove < 0)
      {
        Robot.x -= xmove * cos(MATH::DegreesToRadians(Robot.angle));
        Robot.y -= xmove * sin(MATH::DegreesToRadians(Robot.angle));
      }

      if (ymove>=0)
      {
      Robot.x += ymove * cos(MATH::DegreesToRadians(Robot.angle-90));
      Robot.y += ymove * sin(MATH::DegreesToRadians(Robot.angle-90));

      }
      else if (ymove <0)
      {
      Robot.x += ymove * cos(MATH::DegreesToRadians(Robot.angle+90));
      Robot.y += ymove * sin(MATH::DegreesToRadians(Robot.angle+90));
      }

      if (amove>=0)
      {
          Robot.angle += degrees;
          if(Robot.angle >= 360)
              Robot.angle -=360;
      }
      else if (amove < 0)
      {
          Robot.angle -= degrees;
          if(Robot.angle < 0)
              Robot.angle +=360;
      }

  for(int index=0; index<Samples.size(); index++)
  {
      if (xmove >= 0)
      {
        Samples[index].PositionAndOrientation.x += xmove * cos(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle));
        Samples[index].PositionAndOrientation.y += xmove * sin(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle));
      }
      else if(xmove < 0)
      {
         Samples[index].PositionAndOrientation.x -= xmove * cos(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle));
      Samples[index].PositionAndOrientation.y -= xmove * sin(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle));
      }

      if (ymove>=0)
      {
      Samples[index].PositionAndOrientation.x += ymove * cos(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle-90));
      Samples[index].PositionAndOrientation.y += ymove * sin(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle-90));

      }
      else if (ymove <0)
      {
      Samples[index].PositionAndOrientation.x += ymove * cos(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle+90));
      Samples[index].PositionAndOrientation.y += ymove * sin(MATH::DegreesToRadians(Samples[index].PositionAndOrientation.angle+90));
      }

      if (amove>=0)
      {
       for(index=0; index<Samples.size(); index++)
        {
          Samples[index].PositionAndOrientation.angle += degrees;
          if(Samples[index].PositionAndOrientation.angle >= 360)
            {
              Samples[index].PositionAndOrientation.angle -=360;
            }
        }
      }
      else if (amove < 0)
      {
        for(index=0; index<Samples.size(); index++)
        {
          Samples[index].PositionAndOrientation.angle -= degrees;
          if(Samples[index].PositionAndOrientation.angle < 0)
            {
              Samples[index].PositionAndOrientation.angle +=360;
            }
        }
      }

      if(Samples[index].PositionAndOrientation.x > field_width+border_strip_width)
        {
          Samples[index].PositionAndOrientation.x = field_width+border_strip_width;
          Samples[index].prob = 0.00001;
        }
      if(Samples[index].PositionAndOrientation.x < border_strip_width)
        {
          Samples[index].PositionAndOrientation.x = border_strip_width;
          Samples[index].prob = 0.00001;
        }
      if(Samples[index].PositionAndOrientation.y > field_height+border_strip_width)
        {
          Samples[index].PositionAndOrientation.y = field_height+border_strip_width;
          Samples[index].prob = 0.00001;
        }
      if(Samples[index].PositionAndOrientation.y < border_strip_width)
        {
          Samples[index].PositionAndOrientation.y = border_strip_width;
          Samples[index].prob = 0.00001;
        }      
  }
}

void SelfLocalize::LoadINISettings(minIni* ini)
{
  LoadINISettings(ini, SECTION);
}

void SelfLocalize::LoadINISettings(minIni* ini, const std::string &section)
{
  int value = -2;
  if((value = ini->geti(section, "total_sample", INVALID_VALUE)) != INVALID_VALUE)             total_samples = value;
  if((value = ini->geti(section, "window_width", INVALID_VALUE)) != INVALID_VALUE)   window_width = value;
  if((value = ini->geti(section, "window_height", INVALID_VALUE)) != INVALID_VALUE)  window_height = value;
  if((value = ini->geti(section, "field_width", INVALID_VALUE)) != INVALID_VALUE)       field_width = value;
  if((value = ini->geti(section, "field_height", INVALID_VALUE)) != INVALID_VALUE)       field_height = value;
  if((value = ini->geti(section, "border_strip_width", INVALID_VALUE)) != INVALID_VALUE)       border_strip_width = value;
  if((value = ini->geti(section, "y_goal_top", INVALID_VALUE)) != INVALID_VALUE)       y_goal_top = value;
  if((value = ini->geti(section, "y_goal_bottom", INVALID_VALUE)) != INVALID_VALUE)       y_goal_bottom = value;
  if((value = ini->geti(section, "delta_x_clusters", INVALID_VALUE)) != INVALID_VALUE)       delta_x_clusters = value;
  if((value = ini->geti(section, "delta_y_clusters", INVALID_VALUE)) != INVALID_VALUE)       delta_y_clusters = value;
  if((value = ini->geti(section, "delta_angle_clusters", INVALID_VALUE)) != INVALID_VALUE)       delta_angle_clusters = value;
}

void SelfLocalize::SaveINISettings(minIni* ini)
{
  SaveINISettings(ini, SECTION);
}

void SelfLocalize::SaveINISettings(minIni* ini, const std::string &section)
{
}
