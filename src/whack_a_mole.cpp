#include <unistd.h>
#include <iostream>
#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <geometry_msgs/Transform.h>

static std::string participantNumber;
static std::string spawnNumber;
static std::string levelNumber;
static std::string LF = "panda::panda_leftfinger::panda_leftfinger_collision";
static std::string RF = "panda::panda_rightfinger::panda_rightfinger_collision";
static geometry_msgs::Pose pose;
static geometry_msgs::Pose reset;
static int counter = 1;
static int trails;
static int level = 0;
static int endlevel;
static double mole_positions[] = {-0.3, -0.2, -0.1, 0.1, 0.2, 0.3}; 
static bool begin = true;
static bool ready = false;
static bool check = true;
static int difficultyArray[6][3] = {{1, 2, 3}, {1, 3, 2}, {2, 1, 3}, {2, 3, 1}, {3, 1, 2}, {3, 2, 1}};

ros::Publisher gazebo_model_state_pub;
ros::Publisher whack_check_pub;
ros::Publisher trail_number_pub;
ros::Publisher mole_position_pub;
ros::Publisher level_number_pub;

gazebo_msgs::ModelState model_state;
std_msgs::Int64 trail_number;
std_msgs::Float64 mole_position;

void spawner() 
{
  srand (time(NULL));
  int index = rand() % 6;
  ROS_INFO_STREAM(index);
  
  ROS_INFO_STREAM("Level: " + std::to_string(level) + " Trail: " + std::to_string(counter));
  trail_number.data = counter;
  trail_number_pub.publish(trail_number);
  
  pose.position.x = 0.5;
  pose.position.y = mole_positions[index];
  pose.position.z = 0.04;

  model_state.pose = pose;
  mole_position.data = pose.position.y;
  gazebo_model_state_pub.publish(model_state);
  mole_position_pub.publish(mole_position);      
}

void resetSpawn()
{
  trail_number.data = 0;
  trail_number_pub.publish(trail_number);

  reset.position.x = 0.0;
  reset.position.y = 0.0;
  reset.position.z = 0.0;

  model_state.pose = reset;
  mole_position.data = reset.position.y;
  gazebo_model_state_pub.publish(model_state);
  mole_position_pub.publish(mole_position);
}

void whackCallback(const gazebo_msgs::ContactsState &whack_check)
{
  gazebo_msgs::ContactsState whacks = whack_check;
  std_msgs::Bool whacked;
  std_msgs::Int64 level_number;

  level_number.data = level;
  level_number_pub.publish(level_number);

  model_state.model_name = std::string("mole");    
  model_state.reference_frame = std::string("world");
  int states = whack_check.states.size();
  bool whack;  

  //checking if the mole is hit
  for (int i = 0; i < states; i++)
  {
    gazebo_msgs::ContactState contact = whack_check.states[i];
    if (contact.collision2_name == LF || contact.collision2_name == RF)
    {      
      whack = true;       
      break;
    }
    else 
    {
      whack = false;
    }
  }

  whacked.data = whack;
  whack_check_pub.publish(whacked);

  if ( whack == true)
  {
      counter++;
      resetSpawn();
      whack = false;
      ROS_INFO_STREAM("Hit!");
      usleep(0.1 * 1000000);
      check = true;
  }  
}

void readyCallback(const geometry_msgs::Transform &handTransform)
{
  geometry_msgs::Transform hand = handTransform;
  double z = handTransform.translation.z;
  double y = handTransform.translation.y;

  if (begin == true) 
    {
      begin = false;
      check = true;
      resetSpawn();
    }      

  if (counter <= trails && check == true)
  {
    if (y < 0.015 && y > -0.015 && z > 0.4)
    {
      ready = true;
      usleep(0.5 * 1000000);
      ROS_INFO_STREAM("Spawning");
      spawner();
      check = false; 
    }
    else
    {
      ready = false;
      //ROS_INFO_STREAM("Arm not in ready state");
    }
  }
  else if (counter > trails)
  {      
    level++;
    counter = 0;
    trail_number.data = counter;
    trail_number_pub.publish(trail_number);
    begin = true;

    if (level > endlevel)
    {
      ROS_INFO_STREAM("Last Level Reached, shutting down");
      ros::shutdown();
    }
    else
    {
      std::cout << "Enter Spawn-number to start next level:";
      std::getline(std::cin, spawnNumber);
      trails = std::stoi(spawnNumber);
    }       
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "whack_a_mole_node");
  ros::NodeHandle node_handle;

  std::cout << "Give level number:";
  std::getline(std::cin, levelNumber);
  endlevel = std::stoi(levelNumber);

  std::cout << "Enter spawn number to start practice level:";
  std::getline(std::cin, spawnNumber);
  trails = std::stoi(spawnNumber);

  //starting message callbacks
  ros::Subscriber whack_state_sub = node_handle.subscribe("/whack_check", 1, whackCallback);
  ros::Subscriber reset_state_sub = node_handle.subscribe("/panda/handTransform", 1, readyCallback);

  gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  whack_check_pub = node_handle.advertise<std_msgs::Bool>("/experiment/whacked", 1);
  trail_number_pub = node_handle.advertise<std_msgs::Int64>("/experiment/trail_number", 1);
  mole_position_pub = node_handle.advertise<std_msgs::Float64>("/experiment/mole_position",1);
  level_number_pub = node_handle.advertise<std_msgs::Int64>("/experiment/level_number", 1);

  ros::spin();
  return 0;
}