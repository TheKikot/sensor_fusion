#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>

//#include <sstream>
class SubAndPub
{

public:

  SubAndPub()
  {
    // create the subscriber and the pubisher so they can reamin in the same scope
    ros::Subscriber lasSub = n.subscribe<geometry_msgs::Pose2D>("/pose2D", 100, &SubAndPub::processLaser, this);
    ros::Publisher posePub = n.advertise<nav_msgs::Odometry>("/laser_position", 100);
    printf("created SubAndPub object");
  }
  
  void processLaser(const geometry_msgs::Pose2D::ConstPtr& lasMsg)
  {
    printf("callback called");
    nav_msgs::Odometry odomMsg;
    odomMsg.pose.pose.position.x = lasMsg->x;
    odomMsg.pose.pose.position.y = lasMsg->y;
    odomMsg.pose.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, lasMsg->theta);
    quat.normalize(); 
    odomMsg.pose.pose.orientation.x = quat[0];
    odomMsg.pose.pose.orientation.y = quat[1];
    odomMsg.pose.pose.orientation.z = quat[2];
    odomMsg.pose.pose.orientation.w = quat[3];
    printf("publishing message");
    this->posePub.publish(odomMsg);
  }
  
private:
  ros::NodeHandle n; 
  ros::Publisher posePub;
  ros::Subscriber lasSub;
};


int main(int argc, char **argv)
{
  // initialize the node and create a subscriber for Pose2D message
  ros::init(argc, argv, "convert_pose");
  //ros::NodeHandle n;
  printf("creating object...");
  SubAndPub sp;
  
  ros::Rate loop_rate(10);
  printf("spinning ros...");
  ros::spin();


  return 0;
}
