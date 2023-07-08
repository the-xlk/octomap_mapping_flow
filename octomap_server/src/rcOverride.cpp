/**
 * @file rcOverride.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/ManualControl.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
//#include <transform_listener.h>
//#include <transformListener.h>


geometry_msgs::PoseStamped droneTarget;
geometry_msgs::PoseStamped pubTarget;
mavros_msgs::ManualControl input;
tf::TransformListener* tfl;
bool recievedTarget =false;
ros::Publisher manualOverride;

//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

void updateTarget(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_WARN_STREAM("target input");
    droneTarget = *msg;
    recievedTarget = true;
    //ROS_WARN_STREAM("before");
    //tfl->transformPoint("base_link", odomTarget, odomDrone);
    //ROS_WARN_STREAM("after");
}

void overrideRC(const mavros_msgs::ManualControl::ConstPtr& msg){
    input = *msg;
    ROS_WARN_STREAM("joystic input");
    
    //ROS_WARN_STREAM("drone: x :"<< droneTarget.point.x<<
    //                "drone: y :"<< droneTarget.point.y<<
    //                "drone: z :"<< droneTarget.point.z);
    //geometry_msgs::PoseStamped message = droneTarget;
    //message.pose.position.x+=msg.x;
    //message.pose.position.y+=msg.y;
    //message.pose.position.z+=msg.z;
    //manualOverride.publish(message);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rcOverride");
    ros::NodeHandle nh;

    tfl = new tf::TransformListener(ros::Duration(10));
    ros::Duration(3.0).sleep();
    
    ros::Subscriber targetSub = nh.subscribe<geometry_msgs::PoseStamped>
            ("target_pose", 10, updateTarget);

    ros::Subscriber rcSub = nh.subscribe<mavros_msgs::ManualControl>
            ("mavros/manual_control/control", 10, overrideRC);
    manualOverride = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //        ("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //        ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok()){
        if(recievedTarget){
            ROS_WARN_STREAM("sending");
            pubTarget = droneTarget;
            pubTarget.pose.position.x+=input.x;
            pubTarget.pose.position.y+=input.y;
            pubTarget.pose.position.z+=input.z*2.0-1.0;
            manualOverride.publish(pubTarget);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


