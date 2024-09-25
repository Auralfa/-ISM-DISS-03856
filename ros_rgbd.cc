/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//!
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//!

#include<opencv2/core/core.hpp>

// #include"../../../include/System.h" //!mia
#include "../../../../include/System.h"
#include "../../../../Thirdparty/Sophus/sophus/se3.hpp"

using namespace std;

ros::Publisher pubVisualOdometry;
image_transport::Publisher pubTraImg;
Sophus::SE3f cur_pose;
Sophus::SE3f Twc;
Sophus::Vector6f cur_pose_6d;
Eigen::Matrix3f c; 
ros::Time msg_time; //!

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    //!
    Sophus::SE3f cur_pose;
    Sophus::Vector6f cur_pose_6d;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    //   /camera/color/image_raw        /camera/aligned_depth_to_color/image_raw             /camera/depth/image_rect_raw
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);//!
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/infra1/image_rect_raw", 100);//!
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 100);//!
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    //! publish message
    pubVisualOdometry = nh.advertise<nav_msgs::Odometry>("/orb3_odom", 100);

    image_transport::ImageTransport image_transport(nh);
    pubTraImg = image_transport.advertise("/tracking_image", 1);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    std::vector<Sophus::SE3f> c = SLAM.GetAllKeyframePoses();
    // Open a file in write mode.
    std::ofstream outFile("keyframe_poses.txt");

    if (outFile.is_open()) {
        for (const auto& pose : c) {
            // Extract the translation part of the SE3f object.
            Eigen::Vector3f translation = pose.translation();

            // Write the x, y, and z coordinates to the file.
            outFile << translation.x() << " " << translation.y() << " " << translation.z() << std::endl;
        }
        outFile.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    //!
    // find the pose
    cur_pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Twc = cur_pose.inverse();
    cur_pose_6d = Twc.log();
    msg_time = msgRGB->header.stamp;
    std::cout << "cur_pose : " << cur_pose_6d << std::endl;
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)){ // avoid publishing NaN
        ROS_INFO("Twc is NaN");
        cout << "Twc is NaN" << endl;
    }
    else{
        // publish Twc & tf
        nav_msgs::Odometry visualOdomMsg;
        visualOdomMsg.header.frame_id = "map"; 
        visualOdomMsg.child_frame_id = "camera_link";
        visualOdomMsg.header.stamp = msg_time; //msg->header.stamp;
        // c << 0,0,1,-1,0,0,0,-1,0;
        Eigen::Matrix3f R_mat = Twc.rotationMatrix();
        Eigen::Vector3f t_vec = Twc.translation();
        Eigen::Matrix3f R_mat_ = c*R_mat;
        Eigen::Quaternionf quat(R_mat_);
        // Eigen::Quaternionf quat(R_mat);
        visualOdomMsg.pose.pose.position.x = Twc.translation().z();
        visualOdomMsg.pose.pose.position.y = -Twc.translation().x();
        visualOdomMsg.pose.pose.position.z = -Twc.translation().y();
        // visualOdomMsg.pose.pose.position.x = Twc.translation().x();
        // visualOdomMsg.pose.pose.position.y = Twc.translation().y();
        // visualOdomMsg.pose.pose.position.z = Twc.translation().z();
        visualOdomMsg.pose.pose.orientation.w = quat.w();
        visualOdomMsg.pose.pose.orientation.x = quat.x();
        visualOdomMsg.pose.pose.orientation.y = quat.y();
        visualOdomMsg.pose.pose.orientation.z = quat.z();

        pubVisualOdometry.publish(visualOdomMsg);
        // tf::Matrix3x3 R_tf(
        //     R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        //     R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        //     R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
        // );

        // tf::Vector3 t_tf(
        //     t_vec(0),
        //     t_vec(1),
        //     t_vec(2)
        // );

        tf::Matrix3x3 R_tf(
            R_mat(2, 0), R_mat(2, 1), R_mat(2, 2),
            -R_mat(0, 0), -R_mat(0, 1), -R_mat(0, 2),
            -R_mat(1, 0), -R_mat(1, 1), -R_mat(1, 2)
        );

        tf::Vector3 t_tf(
            t_vec(2),
            -t_vec(0),
            -t_vec(1)
        );

        static tf::TransformBroadcaster br;
        tf::Transform transform = tf::Transform(R_tf, t_tf);
        br.sendTransform(tf::StampedTransform(transform, msg_time, "/map", "/orb3_link"));
    }
    // find the tracking image and publish
    cv::Mat tra_img = mpSLAM->GetCurrentFrame();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tra_img).toImageMsg();
    pubTraImg.publish(msg);


}


