#ifndef THOR_MANG_CONTROL_MODE_SWITCHER_H
#define THOR_MANG_CONTROL_MODE_SWITCHER_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>

#include <thor_mang_control_msgs/thor_mang_control_msgs.h>
#include <thor_mang_control_mode_switcher/trajectory_control_helper.h>



namespace control_mode_switcher
{
class ControlModeSwitcher
{
public:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;
  typedef boost::function<bool ()> ModeSwitchHandle;

  ControlModeSwitcher(ros::NodeHandle& nh);
  virtual ~ControlModeSwitcher();

protected:
  thor_mang_control_msgs::ControlModeStatus getCurrentControlModeStatus() const;

  void getStartedAndStoppedRosControllers();
  bool switchRosControllers(std::vector<std::string> controllers_to_start);

  void getControlModesCallback(const thor_mang_control_msgs::GetControlModesGoalConstPtr& goal);
  void executeSwitchControlModeCallback(const thor_mang_control_msgs::ChangeControlModeGoalConstPtr& goal);

  void allowAllModeTransitionsCb(const std_msgs::Bool& allow);

  thor_mang_control_msgs::ControlModeStatus changeControlMode(std::string requested_mode);

  void executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal);
  void resultFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResultConstPtr& result);

private:
  void parseParameters(ros::NodeHandle nh);
  std::map<std::string, std::string> parseJointCtrlModules(XmlRpc::XmlRpcValue param) const;

  ros::NodeHandle nh_;

  std::map<std::string, controller_manager_msgs::ControllerState> controllers_;

  bool allow_all_mode_transitions_;

  struct Mode
  {
    std::map<std::string, std::string> joint_ctrl_modules_; // joint -> module mapping
    std::vector<std::string> ctrl_modules_;
    std::vector<std::string> desired_controllers_;
    std::vector<std::string> allowed_transitions_;
    ModeSwitchHandle mode_switch_handle_;
  };
  std::map<std::string, Mode> modes_;
  std::string current_mode_name_;

  // subscriber
  ros::Subscriber execute_footstep_sub_;
  ros::Subscriber result_footstep_sub_;
  ros::Subscriber allow_all_mode_transitions_sub_;

  // publisher
  ros::Publisher robot_init_pose_pub_;
  ros::Publisher control_mode_status_pub_;
  ros::Publisher allow_all_mode_transitions_ack_pub_;

  // service clients
  ros::ServiceClient set_joint_ctrl_modules_client_;
  ros::ServiceClient enable_ctrl_modules_client_;
  ros::ServiceClient switch_ros_controllers_client_;
  ros::ServiceClient list_ros_controllers_client_;

  // action clients
  TrajectoryActionClient* trajectory_client_left_;
  TrajectoryActionClient* trajectory_client_right_;

  // action servers
  actionlib::SimpleActionServer<thor_mang_control_msgs::GetControlModesAction> get_control_modes_action_server_;
  actionlib::SimpleActionServer<thor_mang_control_msgs::ChangeControlModeAction> change_control_mode_action_server_;
};
}

#endif // THOR_MANG_CONTROL_MODE_SWITCHER_H
