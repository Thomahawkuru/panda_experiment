#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>

static const int QUEUE_LENGTH = 1;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::Publisher trans_pub = node.advertise<geometry_msgs::Transform>("panda/handTransform", QUEUE_LENGTH);
  tf::TransformListener listener;

  ros::Rate rate(100.0); //hz
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/panda_hand", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //ROS_INFO_STREAM(" Transform: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " <<transform.getOrigin().z() << ", " << transform.getRotation().x() << ", " << transform.getRotation().y() << ", " << transform.getRotation().z());    
    
    geometry_msgs::Transform msg;
    msg.translation.x = transform.getOrigin().x();
    msg.translation.y = transform.getOrigin().y();
    msg.translation.z = transform.getOrigin().z();
    msg.rotation.x = transform.getRotation().x();
    msg.rotation.y = transform.getRotation().y();
    msg.rotation.z = transform.getRotation().z();

    trans_pub.publish(msg);

    rate.sleep();
  }
  return 0;
};