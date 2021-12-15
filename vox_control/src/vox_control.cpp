
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <cstdlib>

ros::Subscriber  cmd_vel_sub;
ros::Publisher l_pub_, r_pub_, b_pub_, f_pub_;

float wheel_separation = 0.3;
double vx=0;
double vy=0;
double wp=0;

void velocity_callback(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vy = msg.linear.y;
    wp = msg.angular.z;
    //std::cout<<vx<<" "<<vy<<" "<<wp<<" "<<"\n";
}




int main(int argc, char** argv){

    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n("");
    // timePrevious = ros::Time::now();
    // int debug = getenv("DEBUG")?atoi(getenv("DEBUG")):0;
    // double wheel_speed = getenv("WSP")?atoi(getenv("WSP")):10;
    double hz=100;
    ros::Rate rate(hz);
    
    // OmniDriver* div;
    // div = new OmniDriver(&n);

    cmd_vel_sub = n.subscribe("cmd_vel", 1000, velocity_callback);
    // jnt_sub = n.subscribe("joint_states", 1000, jnt_state_callback);
    // imu_sub = n.subscribe("imu/data", 100, imu_callback); // use the one with madwigk filter not this
    
    l_pub_ = n.advertise<std_msgs::Float64>("lwheel_joint_velocity_controller/command", 10);
    r_pub_ = n.advertise<std_msgs::Float64>("rwheel_joint_velocity_controller/command", 10);
    b_pub_ = n.advertise<std_msgs::Float64>("bwheel_joint_velocity_controller/command", 10);
    f_pub_ = n.advertise<std_msgs::Float64>("fwheel_joint_velocity_controller/command", 10);
    long int i = 0;

    while(ros::ok()){
       
        
        double vmx= vx;
        double vmy= vy;
        double wmp = wp*wheel_separation ; // Body frame
        
        double v1, v2, v3, v4;
        v1 = vmx - wmp;
        v2 = vmy - wmp;
        v3 = -vmx - wmp;
        v4 = -vmy - wmp;
        

        std_msgs::Float64 wheel_vel;
        wheel_vel.data = v1;
        l_pub_.publish(wheel_vel);
        wheel_vel.data = v2;
        b_pub_.publish(wheel_vel);
        wheel_vel.data = v3;
        r_pub_.publish(wheel_vel);
        wheel_vel.data = v4;
        f_pub_.publish(wheel_vel);


        

        ros::spinOnce();
        rate.sleep();
        
    }
    return 0;
}
