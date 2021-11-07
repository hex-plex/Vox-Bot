#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include<iostream>
#include <ros/console.h>

double f_now = 0;
double b_now = 0;
double l_now = 0;
double r_now = 0;
double f_prev = 0;
double b_prev = 0;
double l_prev = 0;
double r_prev = 0;
double omega_f = 0;
double omega_b = 0;
double omega_l = 0;
double omega_r = 0;
double vx = 0;
double vy = 0;
double vtheta = 0;
bool yaw_flag = true;
double yaw_offset = 0;
double odom_theta = 0;
double odom_x = 0;
double odom_y = 0;
float wheel_radius = 0.015;
float wheel_separation = 0.3;
double ro, pit, yaw;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& input)
{
    b_now= input->position[0];
    f_now= input->position[1];
    l_now= input->position[2];
    r_now= input->position[3];
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(ro, pit, yaw);
    if(yaw_flag==true)
    {
      yaw_flag=false;
      yaw_offset = yaw;
    }
    odom_theta = yaw - yaw_offset;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber joint_state = n.subscribe("vox/joint_states", 1, joint_state_callback);
    ros::Subscriber imu = n.subscribe("imu", 1, imu_callback);



    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1.0);

    while(n.ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        double dt = std::max((current_time - last_time).toSec(),0.0001);

        omega_b = (b_now-b_prev)/dt;
        omega_f = (f_now-f_prev)/dt;
        omega_l = (l_now-l_prev)/dt;
        omega_r = (r_now-r_prev)/dt;

        double v_b = omega_b*wheel_radius;
        double v_f = omega_f*wheel_radius;
        double v_l = omega_l*wheel_radius;
        double v_r = omega_r*wheel_radius;
        vx = (v_b+v_f)/2;
        vy = (v_r+v_l)/2;
        vtheta = (v_r-v_l)/ wheel_separation;
        double X = cos(odom_theta)*vx - sin(odom_theta)*vy;
        double Y = sin(odom_theta)*vx + cos(odom_theta)*vy;
        ROS_INFO("%f",odom_theta);
        odom_x += X*dt;
        odom_y += Y*dt;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_theta);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "dummy_link";

        odom_trans.transform.translation.x = vx;
        odom_trans.transform.translation.y = vy;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = odom_x;
        odom.pose.pose.position.y = odom_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vtheta;

        odom_pub.publish(odom);

        b_prev = b_now;
        f_prev = f_now;
        l_prev = l_now;
        r_prev = r_now;

        last_time = current_time;
        r.sleep();
    }
}
