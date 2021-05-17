/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "../../../include/Converter.h"
#include "../../../include/MapPoint.h"

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM), pub(nh.advertise<geometry_msgs::PoseStamped>("/vodom", 1)){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

public:
    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pub;
};

class MapSaver {
public:
    MapSaver(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM), service(nh.advertiseService("save_map", &MapSaver::save_map, this)) {}

    bool save_map(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        using ORB_SLAM3::MapPoint;

        auto mpAtlas = mpSLAM->getAtlas();
        const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
        const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty()) {
            return true;
        }

        std::vector<float> points;

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) {
                continue;
            }
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            points.push_back(pos.at<float>(0));
            points.push_back(pos.at<float>(1));
            points.push_back(pos.at<float>(2));
        }

        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad()) {
                continue;
            }
            cv::Mat pos = (*sit)->GetWorldPos();
            points.push_back(pos.at<float>(0));
            points.push_back(pos.at<float>(1));
            points.push_back(pos.at<float>(2));
        }

        int num_points = points.size() / 3;
        std::cout << "saving " << num_points << " points..." << std::endl;

        std::ofstream ofs("/tmp/map_points.txt");
        for(int i=0; i<points.size(); i+=3) {
            ofs << points[i] << " " << points[i+1] << " " << points[i+2] << std::endl;
        }

        return true;
    }

public:
    ORB_SLAM3::System* mpSLAM;
    ros::ServiceServer service;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ros::NodeHandle nodeHandler;
    ImageGrabber igb(&SLAM, nodeHandler);
    MapSaver saver(&SLAM, nodeHandler);

    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if(Tcw.data == nullptr) {
        return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "vodom";
    pose.header.stamp = msg->header.stamp;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

    tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    new_transform.setRotation(quaternion);

    tf::poseTFToMsg(new_transform, pose.pose);
    pub.publish(pose);
}


