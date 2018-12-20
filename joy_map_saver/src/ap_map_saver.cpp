#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
//#include <geometry_msgs/Twist.h>
#include <ros/console.h>

#include "joy_map_saver/SaveMap.h"

#include <stdlib.h>
#include <string.h>


// #if WITH_TELEOP
// 	void buttonCallback(const scitos_teleop::action_buttons::ConstPtr& msg)
// 	{
// 	  if(msg->A) {
// 		std::string command("rosrun map_server map_saver -f ");
// 		command += map_name;
// 		ROS_INFO("Saving map as: %s", map_name.c_str());
// 		system(command.c_str());
// 	  }
// 	}
// #endif


std::string map_name;

bool saveMap(std::string file_name){
  	std::string command("rosrun map_server map_saver -f ");
	command += file_name;
	ROS_INFO("Saving map as: %s", command.c_str());

	system(command.c_str());
	return true;
}




void controlCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(msg->buttons[0]) {
    saveMap(map_name);
  }
}



bool saveMapSrv(joy_map_saver::SaveMap::Request  &req, joy_map_saver::SaveMap::Response &res)
{
	saveMap(req.file_name);
	return true;
}	


  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ap_map_saver");
  ros::NodeHandle n("ap_map_saver");
  


  ros::Subscriber sub = n.subscribe("joy", 1000, controlCallback);

  //Check if map name was given as argument to the launch file, and create default map name otherwise
  map_name=std::string(argv[1]);
  if (!map_name.compare(std::string("default_map_name")))   {
    ROS_WARN("No file name given for map, map will be saved with default name on home directory");
    std::string home(getenv("HOME"));
    home+="/";
    char buff[20];
    time_t now = time(NULL);
    strftime(buff, 20, "%Y_%m_%d_%H_%M_%S", localtime(&now));
    map_name = std::string("~/") + std::string(buff) + std::string("map");
  }


  ros::ServiceServer service = n.advertiseService("SaveMap", saveMapSrv);

  ros::spin();

  return 0;
}
