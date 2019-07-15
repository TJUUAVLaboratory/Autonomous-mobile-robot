#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <array>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "robot_msgs/wheelSpeed.h"
#include <tf/transform_broadcaster.h>
double L_pub;
double theta_pub;

struct OptPose
{
    int offset;
    Eigen::Quaterniond q_i_g;
    Eigen::Vector3d t_i_g;
};
struct Pose
{
public:
    Eigen::Matrix4d toMatrix()
    {
        Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
        result.block(0, 0, 3, 3) = q_i_g.toRotationMatrix();
        result.block(0, 3, 3, 1) = t_i_g;
        return result;
    }
    Pose(double time, Eigen::Quaterniond &q, Eigen::Vector3d &trans) : timestamp(time), q_i_g(q.cast<double>()), t_i_g(trans.cast<double>()){};
    double timestamp;
    Eigen::Quaterniond q_i_g;
    Eigen::Vector3d t_i_g;
};

struct WheelParameters
{
    WheelParameters(float l_r, float r_r, float w_b) : left_wheel_radius(l_r), right_wheel_radius(r_r), wheel_base(w_b){};
    float left_wheel_radius;  // 150mm
    float right_wheel_radius; // 150mm
    float wheel_base;         //412mm
};

struct WheelSpeed
{
    WheelSpeed(double t, float r, float l) : timestamp(t), right_rpm(r), left_rpm(l){};
    float left_rpm;
    float right_rpm;
    double timestamp;
};
struct RPMData
{
    RPMData(){};
    RPMData(double t, float r) : timestamp(t), rpm(r){};
    double timestamp;
    float rpm;
};

class WheelSpeedIntergration
{
public:
    WheelSpeedIntergration(WheelParameters &wheel_para);
    ~WheelSpeedIntergration() = default;
    void processMeasure(WheelSpeed &wheel_speed);
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d translation_;
    double time_stamp;

private:
    WheelParameters wheel_para_;
    double RPM_2_SPEED_l;
    double RPM_2_SPEED_r;
    WheelSpeed pre_wheel_speed;
};

WheelSpeedIntergration::WheelSpeedIntergration(WheelParameters &wheel_para)
    : wheel_para_(wheel_para), pre_wheel_speed(-1, 0, 0)
{
    RPM_2_SPEED_l = wheel_para_.left_wheel_radius * 2.0 * M_PI / 60.0;  // RPM -> speed (m/s)
    RPM_2_SPEED_r = wheel_para_.right_wheel_radius * 2.0 * M_PI / 60.0; // RPM -> speed (m/s)
};

void WheelSpeedIntergration::processMeasure(WheelSpeed &wheel_speed)
{
    if (pre_wheel_speed.timestamp < 0)
    {
        pre_wheel_speed = wheel_speed;
        rotation_ = Eigen::Quaterniond::Identity();
        translation_ = Eigen::Vector3d(0, 0, 0);
        time_stamp = wheel_speed.timestamp;
        return;
    }
    // Center of a circle is(0,y)
    double dist[2];
    double dt = wheel_speed.timestamp - pre_wheel_speed.timestamp;

    dist[0] = 0.5 * (wheel_speed.left_rpm + pre_wheel_speed.left_rpm) * RPM_2_SPEED_l * dt;
    dist[1] = 0.5 * (wheel_speed.right_rpm + pre_wheel_speed.right_rpm) * RPM_2_SPEED_r * dt;

    std::cout << "dt : " << dt << std::endl;
    std::cout << "dist 1: " << dist[0] << "dist 2 : " << dist[1] << std::endl;
    //std::cout<<"dt: "<<dt<<std::endl;
    //
    double L = 0.5 * (dist[0] + dist[1]);
    double theta = (dist[1] - dist[0]) / wheel_para_.wheel_base;

    L_pub = L / dt;
    theta_pub = theta / dt;
    //std::cout<<"dis1 : "<<dist[1]<<" dis 2: "<<dist[0]<<std::endl;
    //std::cout<<"theta: "<<wheel_speed.timestamp<<"   "<<theta<<std::endl;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double p; // sin(theta)/theta
    double q; // (1-cos(theta))/theta
    if (std::abs(theta) > 1.e-5)
    {
        p = sin_theta / theta;
        q = (1.0 - cos_theta) / theta;
    }
    // Limits for theta -> 0
    else
    {
        p = 1.0;
        q = 0.0;
        theta = 0;
    }
    // Local pose update matrix
    Eigen::Matrix3d M;
    M << p, -q, 0,
        q, p, 0,
        0, 0, 1;

    Eigen::Vector3d v(L, 0, theta);
    Eigen::Vector3d dpose = M * v;
    Eigen::Vector3d t_cur_to_pre(dpose[0], dpose[1], 0);
    Eigen::Quaterniond q_cur_to_pre(std::cos(0.5 * theta), 0, 0,
                                    std::sin(0.5 * theta));
    //
    translation_ = rotation_.toRotationMatrix() * t_cur_to_pre + translation_;
    rotation_ = rotation_ * q_cur_to_pre;
    pre_wheel_speed = wheel_speed;
    time_stamp = wheel_speed.timestamp;
    //std::cout<<"translation: "<<translation_.transpose()<<std::endl;
}

class Node
{
public:
    Node(WheelParameters &wheel_para, const std::string &topic);
    ~Node() = default;

private:
    void HandleWheelSpeed(const robot_msgs::wheelSpeed::ConstPtr &msg);

private:
    WheelSpeedIntergration wheel_intergrater;
    std::vector<WheelSpeed> wheel_datas;
    std::map<double, float> left_rpms, right_rpms;
    ::ros::NodeHandle node_handle_;
    ::ros::Subscriber sub_;
    ::ros::Publisher odom_publisher;
    tf::TransformBroadcaster odom_broadcaster;
};

Node::Node(WheelParameters &wheel_para, const std::string &topic)
    : wheel_intergrater(wheel_para)
{

    sub_ = node_handle_.subscribe(
        "/wheelSpeed", 200, &Node::HandleWheelSpeed, this);
    odom_publisher = node_handle_.advertise<nav_msgs::Odometry>("odom", 50);
}

// callback  /wheelSpeed
void Node::HandleWheelSpeed(const robot_msgs::wheelSpeed::ConstPtr &msg)
{
    int32_t timestamp = msg->timeStamp;
    long long timestamp_ = timestamp;
    int16_t speed = msg->speed[0];
    int8_t group_id = msg->group;
    int rpm = (int)speed;
    double time_stamp_sec = (double)timestamp_ / 1e4;
    RPMData rpm_data;
    rpm_data.timestamp = time_stamp_sec;
    rpm_data.rpm = rpm;
    if (group_id == 0)
    {
        left_rpms.insert({time_stamp_sec, rpm});
        if (left_rpms.size() > 10)
            left_rpms.erase(left_rpms.begin());
    }
    else
    {
        right_rpms.insert({time_stamp_sec, rpm});
        if (right_rpms.size() > 10)
            right_rpms.erase(right_rpms.begin());
    }
    if (!left_rpms.empty() && !right_rpms.empty())
    {
        RPMData base(left_rpms.begin()->first, left_rpms.begin()->second);
        auto it_1 = right_rpms.begin();
        auto it_2 = it_1;
        it_2++;
        for (; it_2 != right_rpms.end(); ++it_1, ++it_2)
        {
            if (it_1->first < base.timestamp && it_2->first >= base.timestamp)
            {
                std::cout << "base : " << base.timestamp << std::endl;
                double ratio = (it_2->first - base.timestamp) / (it_2->first - it_1->first);
                double rpm_right = it_2->second * (1 - ratio) + ratio * it_1->second;
                WheelSpeed new_wheel_speed_syned(base.timestamp, rpm_right, base.rpm);
                wheel_intergrater.processMeasure(new_wheel_speed_syned);
                // TODO can publish odom here
                nav_msgs::Odometry odom;
                std::cout << wheel_intergrater.time_stamp << std::endl;
                odom.header.stamp = ros::Time((float)wheel_intergrater.time_stamp);
                odom.header.frame_id = "odom";
                //set the position
                odom.pose.pose.position.x = wheel_intergrater.translation_[0];
                odom.pose.pose.position.y = wheel_intergrater.translation_[1];
                odom.pose.pose.position.z = wheel_intergrater.translation_[2];
                odom.pose.pose.orientation.w = wheel_intergrater.rotation_.w();
                odom.pose.pose.orientation.x = wheel_intergrater.rotation_.x();
                odom.pose.pose.orientation.y = wheel_intergrater.rotation_.y();
                odom.pose.pose.orientation.z = wheel_intergrater.rotation_.z();

                //set the velocity
                odom.child_frame_id = "base_link";
                odom.twist.twist.linear.x = L_pub;
                // odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = theta_pub;

                //publish odom message
                odom_publisher.publish(odom);
                left_rpms.erase(left_rpms.begin());

                // broadcast odometry transform
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = odom.header.stamp;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = wheel_intergrater.translation_[0];
                odom_trans.transform.translation.y = wheel_intergrater.translation_[1];
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom.pose.pose.orientation;
                odom_broadcaster.sendTransform(odom_trans);

                // base_link ==> IMU
                // geometry_msgs::TransformStamped base_trans;
                // base_trans.header.stamp = odom.header.stamp;
                // base_trans.header.frame_id = "base_link";
                // base_trans.child_frame_id = "horizontal_vlp16_link";

                // base_trans.transform.translation.x = -0.1;
                // base_trans.transform.translation.y = 0;
                // base_trans.transform.translation.z = 0.0;
                // base_trans.transform.rotation.w = -0.001;
                // base_trans.transform.rotation.x = 0;
                // base_trans.transform.rotation.y = 0;
                // base_trans.transform.rotation.z = 1;
                // odom_broadcaster.sendTransform(base_trans);

                // imu ==> horizontal_vlp16_link
                // imu ==> horizontal_vlp16_link

                break;
            }
        }
    }
}
void HandleWheelSpeed_(const robot_msgs::wheelSpeed::ConstPtr &msg)
{
    std::cout << "get!!!" << std::endl;
}
int main(int argc, char *argv[])
{
    ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
    ::ros::start();
    WheelParameters wheel_para(0.075, 0.075, 0.412);
    Node node(wheel_para, "/wheelSpeed");
    ::ros::spin();
    ::ros::shutdown();
}
