#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_mode_switcher_node");
  ROS_DEBUG("Starting Control Mode Switcher Node");

  ros::NodeHandle nh;
  control_mode_switcher::ControlModeSwitcher control_mode_switcher(nh);
  ros::spin();
}
