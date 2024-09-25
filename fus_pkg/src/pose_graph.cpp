#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <vector>
#include <algorithm>
#include <deque>
#include <map>
#include <nav_msgs/Path.h> 

class OdomFusion {
public:
    OdomFusion(ros::NodeHandle& nh) : nh_(nh), index_(0) {
        sub_odom_lidar_ = nh_.subscribe("/aft_mapped_to_init", 10, &OdomFusion::odomCallbackLidar, this);
        sub_odom_radar_ = nh_.subscribe("/radar_graph_slam/aftmapped_odom", 10, &OdomFusion::odomCallbackRadar, this);
        sub_odom_camera_ = nh_.subscribe("/odom", 10, &OdomFusion::odomCallbackCamera, this);
        
        // 发布融合后的 Odom 消息
        pub_fused_ = nh_.advertise<nav_msgs::Odometry>("odom_fused", 10);

        // 发布路径
        pub_path_ = nh_.advertise<nav_msgs::Path>("trajectory_fused", 10);
        
        // 初始化路径
        path_.header.frame_id = "map"; // 设置路径的坐标系
        
        graph_ = gtsam::NonlinearFactorGraph();
        initialEstimate_ = gtsam::Values();
        
        lidar_offset_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(1.57, 0, 0), gtsam::Point3(0, 0, 0.2));
        radar_offset_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        camera_offset_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0.1));

        // 设定触发周期
        output_period_ = 0.1; // 每0.1秒触发一次
        last_output_time_ = ros::Time::now().toSec();
    }

    void odomCallbackLidar(const nav_msgs::Odometry::ConstPtr& msg) {
        storeOdomData(msg, lidar_offset_, 'l');
    }

    void odomCallbackRadar(const nav_msgs::Odometry::ConstPtr& msg) {
        storeOdomData(msg, radar_offset_, 'r');
    }

    void odomCallbackCamera(const nav_msgs::Odometry::ConstPtr& msg) {
        storeOdomData(msg, camera_offset_, 'c');
    }

    void storeOdomData(const nav_msgs::Odometry::ConstPtr& msg, const gtsam::Pose3& offset, char sensor_type) {
        double current_time = msg->header.stamp.toSec();
        odom_data_[sensor_type].emplace_back(current_time, msg);

        // 移除过旧的数据
        while (!odom_data_[sensor_type].empty() && 
               (current_time - odom_data_[sensor_type].front().first > time_window_)) {
            odom_data_[sensor_type].pop_front();
        }

        synchronizeAndFuseData();
    }

    void synchronizeAndFuseData() {
        // 检查是否可以进行融合
        if (checkIfDataReady()) {
            // 执行数据融合逻辑
            for (const auto& sensor : odom_data_) {
                // 从每个传感器队列中获取最新数据
                const auto& latest_data = sensor.second.back();
                processOdom(latest_data.second, sensor.first);
            }

            // 优化
            auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initialEstimate_).optimize();
            publishFusedOdom(result);
        }
    }

    bool checkIfDataReady() {
        // 检查每个传感器是否都有数据
        return !odom_data_['l'].empty() && !odom_data_['r'].empty() && !odom_data_['c'].empty();
    }

    void processOdom(const nav_msgs::Odometry::ConstPtr& msg, char sensor_type) {
        // 提取位置和朝向
        gtsam::Pose3 pose(
            gtsam::Rot3(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
            gtsam::Point3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)
        );

        initialEstimate_.insert(gtsam::Symbol(sensor_type, index_), pose);

        gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.1));
        
        if (index_ == 0) {
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(sensor_type, index_), pose, noise_model));
        }

        if (index_ > 0) {
            for (int i = 0; i < index_; i++) {
                gtsam::Pose3 previous_pose = initialEstimate_.at<gtsam::Pose3>(gtsam::Symbol(sensor_type, i));
                gtsam::Pose3 relative_pose = previous_pose.between(pose);
                graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(sensor_type, i), gtsam::Symbol(sensor_type, index_), relative_pose, noise_model));
            }
        }

        // 输出周期触发
        double current_time = ros::Time::now().toSec();
        if (current_time - last_output_time_ >= output_period_) {
            last_output_time_ = current_time;
            publishFusedOdom(gtsam::LevenbergMarquardtOptimizer(graph_, initialEstimate_).optimize());
        }

        index_++;
    }

    void publishFusedOdom(const gtsam::Values& result) {
        // 获取融合后的位姿
        gtsam::Pose3 fused_pose = result.at<gtsam::Pose3>(gtsam::Symbol('l', index_ - 1)); // 从最后一个传感器的索引中获取

        // 创建 Odom 消息
        nav_msgs::Odometry fused_odom;
        fused_odom.header.stamp = ros::Time::now();
        fused_odom.header.frame_id = "map";
        fused_odom.child_frame_id = "odom_fused";
        fused_odom.pose.pose.position.x = fused_pose.translation().x();
        fused_odom.pose.pose.position.y = fused_pose.translation().y();
        fused_odom.pose.pose.position.z = fused_pose.translation().z();
        fused_odom.pose.pose.orientation.x = fused_pose.rotation().quaternion()[1];
        fused_odom.pose.pose.orientation.y = fused_pose.rotation().quaternion()[2];
        fused_odom.pose.pose.orientation.z = fused_pose.rotation().quaternion()[3];
        fused_odom.pose.pose.orientation.w = fused_pose.rotation().quaternion()[0];

        pub_fused_.publish(fused_odom);

        // 更新路径
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = fused_odom.header;
        pose_stamped.pose = fused_odom.pose.pose;
        path_.poses.push_back(pose_stamped); // 将新点添加到路径
        pub_path_.publish(path_); // 发布路径

        // 发布坐标变换
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(fused_odom.pose.pose.position.x, fused_odom.pose.pose.position.y, fused_odom.pose.pose.position.z));
        tf::Quaternion q(fused_odom.pose.pose.orientation.x, fused_odom.pose.pose.orientation.y, fused_odom.pose.pose.orientation.z, fused_odom.pose.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, fused_odom.header.stamp, "map", "odom_fused"));
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_lidar_;
    ros::Subscriber sub_odom_radar_;
    ros::Subscriber sub_odom_camera_;
    ros::Publisher pub_fused_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initialEstimate_;
    gtsam::Pose3 lidar_offset_;
    gtsam::Pose3 radar_offset_;
    gtsam::Pose3 camera_offset_;
    std::map<char, std::deque<std::pair<double, nav_msgs::Odometry::ConstPtr>>> odom_data_;
    nav_msgs::Path path_; // 存储路径信息
    ros::Publisher pub_path_; // 用于发布路径
    double time_window_ = 0.2; // 时间窗口，单位为秒
    double output_period_; // 输出触发周期
    double last_output_time_; // 上一次输出的时间
    int index_; // 用于追踪索引
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_fusion");
    ros::NodeHandle nh;
    OdomFusion odom_fusion(nh);
    ros::spin();
    return 0;
}
