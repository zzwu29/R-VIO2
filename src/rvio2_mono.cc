/**
* This file is part of R-VIO2.
*
* Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO2> 
*
* R-VIO2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "rvio2/System.h"


class ImageGrabber
{
public:
    ImageGrabber(RVIO2::System* pSys) : mpSys(pSys) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    RVIO2::System* mpSys;
};


class ImuGrabber
{
public:
    ImuGrabber(RVIO2::System* pSys) : mpSys(pSys) {}

    void GrabImu(const sensor_msgs::ImuConstPtr& msg);

    RVIO2::System* mpSys;
};


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int lastseq = -1;
    if ((int)msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_DEBUG("Image message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);
    lastseq = msg->header.seq;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    RVIO2::ImageData* pData = new RVIO2::ImageData();
    pData->Image = cv_ptr->image.clone();
    pData->Timestamp = cv_ptr->header.stamp.toSec();

    mpSys->PushImageData(pData);

    mpSys->run();
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    static int lastseq = -1;
    if ((int) msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_DEBUG("IMU message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);
    lastseq = msg->header.seq;

    Eigen::Vector3f angular_velocity(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Eigen::Vector3f linear_acceleration(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);

    double currtime = msg->header.stamp.toSec();

    RVIO2::ImuData* pData = new RVIO2::ImuData();
    pData->AngularVel = angular_velocity;
    pData->LinearAccel = linear_acceleration;
    pData->Timestamp = currtime;

    static double lasttime = -1;
    if (lasttime!=-1)
        pData->TimeInterval = currtime-lasttime;
    else
        pData->TimeInterval = 0;
    lasttime = currtime;

    mpSys->PushImuData(pData);
}

std::string GetImuTopic(const std::string& strSettingsFile)
{
    // Read settings file
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
       ROS_ERROR("Failed to open settings file at: %s", strSettingsFile.c_str());
       exit(-1);
    }

    std::string s = fsSettings["IMU.topic"];
    return s;
}

std::string GetCameraTopic(const std::string& strSettingsFile)
{
    // Read settings file
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
       ROS_ERROR("Failed to open settings file at: %s", strSettingsFile.c_str());
       exit(-1);
    }

    std::string s = fsSettings["Camera.topic"];
    return s;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rvio2_mono");

    ros::start();

    RVIO2::System Sys(argv[1]);

    ImageGrabber igb1(&Sys);
    ImuGrabber igb2(&Sys);

    ros::NodeHandle nodeHandler;

    std::string ImuTopic, CameraTopic;
    if((ImuTopic=GetImuTopic(argv[1])).empty())
    {
        ImuTopic="/imu0";
    }
    if((CameraTopic=GetCameraTopic(argv[1])).empty())
    {
        CameraTopic="/cam0/image_raw";
    }

    std::cout<<"ImuTopic: "<<ImuTopic<<std::endl;
    std::cout<<"CameraTopic: "<<CameraTopic<<std::endl;

    ros::Subscriber imu_sub = nodeHandler.subscribe(ImuTopic, 100, &ImuGrabber::GrabImu, &igb2);
    ros::Subscriber image_sub = nodeHandler.subscribe(CameraTopic, 1, &ImageGrabber::GrabImage, &igb1);

    ros::spin();

    ros::shutdown();

    return 0;
}
