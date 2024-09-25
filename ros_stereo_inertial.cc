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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../../include/System.h" //!zw
#include"../include/ImuTypes.h"

#include <tf/transform_broadcaster.h> //!zw
#include <nav_msgs/Odometry.h> //!zw


using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe);

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    void PublishTransform(const Sophus::SE3f &Tcw, const ros::Time &stamp); // 添加函数声明

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    tf::TransformBroadcaster tfBroadcaster; // 添加tf广播成员
};


ImageGrabber::ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber* pImuGb, bool bRect, bool bClahe)
    : mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe)
{
    if(mbClahe)
    {
        mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    }
}

// void publishTF()
// {
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    
//     // 创建一个90度旋转矩阵，绕y轴旋转使z轴对齐x轴
//     tf::Quaternion q;
//     q.setRPY(0, -M_PI/2, 0); // 这里是绕y轴旋转90度
//     transform.setRotation(q);

//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_corrected"));
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);
  
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/stereo_inertial_publisher/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_left = n.subscribe("/stereo_inertial_publisher/left/image_rect", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/stereo_inertial_publisher/right/image_rect", 100, &ImageGrabber::GrabImageRight,&igb);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("orb3_odom", 1000);
  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  // ros::Rate loop_rate(10000);  // 控制主循环频率

  // while (ros::ok())
  // {
  //   //  发布变换
  //   publishTF();  // 在这里调用 

  //   ros::spinOnce();  // 处理回调函数
  //   loop_rate.sleep();  // 控制循环频率
  // }
    tf::TransformBroadcaster odom_broadcaster;
    Sophus::SE3f pose = SLAM.GetCurrentFrame(); 
    auto currentFrame = SLAM.GetCurrentFrame();
    Eigen::Matrix4f identity_matrix = Eigen::Matrix4f::Identity();
    if (!pose.matrix().isApprox(identity_matrix)) 
    {
        // 发布tf变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "orb3_odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose.translation().x();
        odom_trans.transform.translation.y = pose.translation().y();
        odom_trans.transform.translation.z = pose.translation().z();
        
        Eigen::Quaternionf q(pose.unit_quaternion());
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        // 发布tf变换
        odom_broadcaster.sendTransform(odom_trans);

        // 同时发布odom消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = odom_trans.header.stamp;
        odom_msg.header.frame_id = "orb3_odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = pose.translation().x();
        odom_msg.pose.pose.position.y = pose.translation().y();
        odom_msg.pose.pose.position.z = pose.translation().z();
        odom_msg.pose.pose.orientation = odom_trans.transform.rotation;

        odom_pub.publish(odom_msg);
    }


  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrame_Stereo_Inertial.txt");
  SLAM.SaveTrajectoryTUM("Trajectory_Stereo_Inertial.txt");

  ros::shutdown();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                continue;
            }
            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if(mbClahe)
            {
                mClahe->apply(imLeft,imLeft);
                mClahe->apply(imRight,imRight);
            }

            if(do_rectify)
            {
                cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
                cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);

            // 获取位姿并发布
            PublishTransform(Tcw, ros::Time(tImLeft));

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


void ImageGrabber::PublishTransform(const Sophus::SE3f &Tcw, const ros::Time &stamp)
{
    if (!Tcw.translation().isZero() || !Tcw.so3().unit_quaternion().coeffs().isZero()) {
        tf::Transform transform;
        tf::Vector3 translation(Tcw.translation()[0], Tcw.translation()[1], Tcw.translation()[2]);
        tf::Quaternion rotation(Tcw.so3().unit_quaternion().x(), Tcw.so3().unit_quaternion().y(),
                                Tcw.so3().unit_quaternion().z(), Tcw.so3().unit_quaternion().w());

        transform.setOrigin(translation);
        transform.setRotation(rotation);

        tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "map", "camera_link"));
    }


}
