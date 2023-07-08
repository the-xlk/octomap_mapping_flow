#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

static std::string topic = "/odom";
static std::string pub = "/mavros/vision_pose/pose";
static ros::Publisher publisher;
geometry_msgs::PoseStamped pose_publish;

void odom_callback(const nav_msgs::OdometryConstPtr& odom){
  geometry_msgs::Pose odom_pose = odom->pose.pose;
  std_msgs::Header odom_header = odom->header;
  pose_publish.pose =odom_pose;
  pose_publish.header =odom_header;
  pose_publish.header.stamp = ros::Time::now();
  //ROS_WARN("callback");
  
}

int main(int argc, char **argv){ 

  ros::init(argc, argv, "odom2pos");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  publisher = nh.advertise<geometry_msgs::PoseStamped>(pub, 10);
  ros::NodeHandle private_nh("~");

  private_nh.getParam("odom_topic", topic);
  private_nh.getParam("pub_topic", pub);

  std::cout << "Topic: " << topic << std::endl;
  std::cout << "Pub: " << pub << std::endl;
  //std::cout << "Child frame: " << child_frame << std::endl;

  ros::Subscriber odom_sub = nh.subscribe(topic, 10, odom_callback);
  
  while(ros::ok()){
    publisher.publish(pose_publish);
    ros::spinOnce();
    rate.sleep();
  }
  
  

  return 0;
}
