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
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
//#include <transform_listener.h>
//#include <transformListener.h>


geometry_msgs::PoseStamped droneTarget;
geometry_msgs::PoseStamped pubTarget;
mavros_msgs::RCIn input;
bool recievedTarget =false;
bool recievedInput =false;
ros::Publisher manualOverride;
float yaw =0;
tf::Quaternion myQuaternion;

//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

void updateTarget(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //ROS_WARN_STREAM("target input");
    droneTarget = *msg;
    recievedTarget = true;
}

void overrideRC(const mavros_msgs::RCIn::ConstPtr& msg){
    input = *msg;
    recievedInput=true;
    //ROS_WARN_STREAM("joystic input");
    
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

    ros::Duration(3.0).sleep();
    
    ros::Subscriber targetSub = nh.subscribe<geometry_msgs::PoseStamped>
            ("target_pose", 10, updateTarget);

    ros::Subscriber rcSub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 10, overrideRC);
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
        if(recievedTarget && recievedInput){
            //ROS_WARN_STREAM("sending");
            //yaw -= (((float)input.channels[0])-1494.0f)/51200.0f;
            myQuaternion.setRPY(0,0,-(((float)input.channels[0])-1494.0f)/512.0f+M_PI/2);
            tf::Quaternion tempQuat;
            tempQuat.setValue(droneTarget.pose.orientation.x,
                              droneTarget.pose.orientation.y,
                              droneTarget.pose.orientation.z,
                              droneTarget.pose.orientation.w);
            myQuaternion = myQuaternion*tempQuat;
            myQuaternion.normalize();
            tf::Matrix3x3 m(myQuaternion);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            pubTarget = droneTarget;
            float x = (((float)input.channels[2])-1494.0f)/512.0f;
            float y = (((float)input.channels[3])-1494.0f)/512.0f;
            float z = (((float)input.channels[1])-1494.0f)/512.0f;
            
            //ROS_WARN_STREAM("in yaw "<<(float)input.channels[0]<<" centered "<<(float)input.channels[0]-1494.0f<<" normalized "<<(((float)input.channels[0])-1494.0f)/512.0f);
            pubTarget.pose.position.x+=x*cos(yaw)+sin(yaw)*y;
            pubTarget.pose.position.y+=-y*cos(yaw)+sin(yaw)*x;
            pubTarget.pose.position.z+=z;
            
            //ROS_WARN_STREAM("yaw: " << yaw);
            myQuaternion.setRPY(0,0,yaw);
            pubTarget.pose.orientation.x=myQuaternion.getX();
            pubTarget.pose.orientation.y=myQuaternion.getY();
            pubTarget.pose.orientation.z=myQuaternion.getZ();
            pubTarget.pose.orientation.w=myQuaternion.getW();
            manualOverride.publish(pubTarget);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


