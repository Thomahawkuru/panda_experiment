#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_state_initializer_node");
  ros::NodeHandle node_handle;

  std::vector<double> panda_ready_state{0, 0.527, 0, -4.712, 0, 5.214, 1.571};

  // define variables
  std::string joint_position_command_topic{"/joint_position_controller/command"},
      controller_manager_switch_topic{"/controller_manager/switch_controller"};
  ros::ServiceClient switch_controller_client =
      node_handle.serviceClient<controller_manager_msgs::SwitchController>(controller_manager_switch_topic);
  ros::Publisher joint_position_publisher =
      node_handle.advertise<std_msgs::Float64MultiArray>(joint_position_command_topic, 1);

  // sleep for 2 seconds in order to make sure that the system is up and running
  ros::Duration(2.0).sleep();

  // 1. switch from default joint trajectory controller to custom position controller
  std::string panda_arm_controller{"panda_arm_controller"}, panda_hand_controller{"panda_hand_controller"}, panda_hand_position_controller{"panda_hand_position_controller"},
      joint_position_controller{"joint_position_controller"}, joint_group_position_controller{"joint_group_position_controller"};
  std::vector<std::string> plan_controllers{panda_arm_controller, panda_hand_controller};
  std::vector<std::string> initilize_controllers{joint_position_controller};
  std::vector<std::string> jog_controllers{joint_group_position_controller, panda_hand_position_controller};

  ROS_INFO_STREAM("Initializing robot state");
  controller_manager_msgs::SwitchController srv_switch_controller;
  srv_switch_controller.request.stop_controllers = plan_controllers;
  srv_switch_controller.request.start_controllers = initilize_controllers;
  srv_switch_controller.request.strictness = 2;


  // housekeeping for logging
  std::ostringstream stream_start_controllers, stream_stop_controllers;
  for (const auto &elem : plan_controllers) {
    stream_start_controllers << elem << ", ";
  }
  for (const auto &elem : initilize_controllers) {
    stream_stop_controllers << elem << ", ";
  }

  if (switch_controller_client.call(srv_switch_controller)) {
    ROS_INFO_STREAM("Success switching controllers from " << stream_stop_controllers.str() << " to "
                                                          << stream_start_controllers.str());
  } else {
    ROS_WARN_STREAM("Error switching controllers from " << stream_stop_controllers.str() << " to "
                                                        << stream_start_controllers.str());
    return -1;
  }

  // 2. publish the desired joint position to the custom controller
  std_msgs::Float64MultiArray command_msg;
  command_msg.data = panda_ready_state;
  joint_position_publisher.publish(command_msg);

  // sleep for 1 seconds in order to make sure that the controller finishes moving the robot
  ros::Duration(1.0).sleep();

  // 3. Restore default controllers
  ROS_INFO_STREAM("Starting Jog Controllers");

  srv_switch_controller.request.stop_controllers = initilize_controllers;
  srv_switch_controller.request.start_controllers = jog_controllers;

  if (switch_controller_client.call(srv_switch_controller)) {
    ROS_INFO_STREAM("Success switching controllers from " << stream_stop_controllers.str() << " to "
                                                          << stream_start_controllers.str());
  } else {
    ROS_WARN_STREAM("Error switching controllers from " << stream_stop_controllers.str() << " to "
                                                        << stream_start_controllers.str());
    return -1;
  }

  return 0;
}