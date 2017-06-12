#include <ros/ros.h>
#include <sys/types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include "stdio.h"
#include <fstream>
#include <iostream>
#include<vector>
#include"math.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <stdlib.h>

#include "../include/chess.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;


//----------IMPORTANT!!!!---------------------


std::string savePath = "./src/calibration_cam_laser";//PATH TO SAVE EVERYTHING

std::string namePic ="/RGB";//name of pictures
std::string nameLas ="/laser";//name of laser data
std::string las =savePath+ nameLas + ".txt";//ścieżka pliku tekstowego
 const char *laserPath = las.c_str();


int samplenum= 30;//numbers of samples


//----------ZMIENNE UŻYWANE W FUNCKJACH---------------------------

    cv::Mat obraz;


//funkcja działająca na prawdziwych odczytach i obrazach
void calibration(const ImageConstPtr& image, const LaserScan::ConstPtr& laser)
{
    static int probka = 1;//numer próbki

    ROS_INFO("Working %d",1);
    cv_bridge::CvImagePtr box;
    box = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

    obraz=box->image;//box->image, bo z klasy wyciągamy obraz


     cv::imshow("view", obraz);//pokaz obrazu
    cv::waitKey(20);


      std::string sample = boost::lexical_cast<std::string>(probka);
     cv::imwrite(savePath + namePic + sample + ".png", obraz);//zapis obrazu


        std::fstream file;
        file.open(laserPath,  std::fstream::in | std::fstream::out | std::fstream::app );

        {
                int number = (laser->angle_max - laser->angle_min)/laser->angle_increment;
                ROS_INFO("\n number of taken picture %d \n hokuyo info: \n angle_max %f \n angle_min %f \n angle_inc  %f \n length %d", probka, laser->angle_max, laser->angle_min, laser->angle_increment, number);

                for(int i=0; i<number-1; i++){
                        file<<laser->ranges[i]<<", ";
                }
                file<<laser->ranges[number-1]<<std::endl;

                file.close();
        }

    if(probka<samplenum)
    {
    probka++;
    }
    else
    {
        //TUTAJ FUNKACJA KALIBRACJI
        //chess();
    cv::waitKey(0);
     calculateTransformation(savePath);
    ros::shutdown();



    }




}



//-----------------------------------------------------------------------------------------------
//------------------------------------MAIN-------------------------------------------------------
//------------------------------------------------------------------------------------------------


int main(int argc, char** argv)
{


   std::system("pwd");

  ros::init(argc, argv, "calibration_node");
  std::cout<<"Calibration begin"<<std::endl;
  ros::NodeHandle nh;

ROS_INFO("STARTING PROGRAM %d",1);

//Czyszczenie zawartości pliku tekstowego
/*std::ofstream ofs;
ofs.open(laserPath, std::ofstream::out | std::ofstream::trunc);
ofs.close();*/


cv::namedWindow("view");
cv::startWindowThread();


message_filters::Subscriber<Image> image_sub(nh, "/usb_cam/image_raw", 1);
message_filters::Subscriber<LaserScan> laserscan_sub(nh, "scan", 1);
typedef sync_policies::ApproximateTime<Image, LaserScan> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, laserscan_sub);
  sync.registerCallback(boost::bind(&calibration, _1, _2));

 ros::spin();
cv::destroyWindow("view");

}
