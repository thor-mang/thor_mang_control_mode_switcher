#include <thor_mang_control_mode_switcher/thor_mang_mode_switch_handles.h>



namespace control_mode_switcher
{
void goToStandPrepMode(ros::Publisher pub)
{
  std_msgs::String msg;
  msg.data = "ini_pose";
  pub.publish(msg);
}

void goToStandMode(TrajectoryControlHelper::Ptr trajectory_control_helper)
{
  std::map<TrajectoryController, std::vector<double>> joint_config;
  joint_config[LEFT_ARM]  = {0.44, 1.31, 0.09, -0.52, 0.0, 0.0, 0.0};
  joint_config[RIGHT_ARM] = {-0.44, -1.31, -0.09, 0.52, 0.0, 0.0, 0.0};
  joint_config[HEAD]      = {0.4, 0.0};
  trajectory_control_helper->goToJointConfiguration(joint_config, 3.0, false);
}

void goToSoftStop(StepPlanActionClientPtr step_plan_action_client)
{
  if (!step_plan_action_client->waitForServer(ros::Duration(5.0)))
    ROS_WARN("[ControlModeSwitcher] Time out while waititing for step plan server");

  if (step_plan_action_client->isServerConnected())
    step_plan_action_client->sendGoal(vigir_footstep_planning_msgs::ExecuteStepPlanGoal());
}

void goToShutdownMode(TrajectoryControlHelper::Ptr trajectory_control_helper)
{
  for (int phase = 0; phase < 5; phase++)
  {
    std::map<TrajectoryController, std::vector<double>> joint_config;

    switch (phase)
    {
      case 0:
        joint_config[LEFT_ARM]  = {1.2, -0.27, 0.0, -2.0, 1.55, 0.0, 0.0};
        joint_config[RIGHT_ARM] = {-1.2, 0.27, 0.0, 2.0, -1.55, 0.0, 0.0};
        joint_config[TORSO]     = {0.0, 0.3};
        break;
      case 1:
        joint_config[LEFT_LEG]  = {1.42, -0.17, 3.02, -1.67, -0.15, 0.0};
        joint_config[RIGHT_LEG] = {-1.42, 0.17, -3.02, 1.67, 0.15, 0.0};
        break;
      case 2:
        joint_config[LEFT_ARM]  = {-1.2, 0.6, 0.0, 2.8, -1.55, 0.0, 0.0};
        joint_config[RIGHT_ARM] = {-1.2, 0.6, 0.0, 2.8, -1.55, 0.0, 0.0};
        joint_config[HEAD]      = {0.0, -0.9};
        break;
      case 3:
        joint_config[LEFT_ARM]  = {0, -0.6, 0.0, -2.2, 1.55, 0.0, 0.0};
        joint_config[RIGHT_ARM] = {0, 0.6, 0.0, 2.2, -1.55, 0.0, 0.0};
        break;
      case 4:
        joint_config[LEFT_ARM]  = {-0.7, -0.6, 0.0, -1.2, 1.55, 0.0, 0.0};
        joint_config[RIGHT_ARM] = {0.7, 0.6, 0.0, 1.2, -1.55, 0.0, 0.0};
        joint_config[TORSO]     = {0.0, 0.45};
        break;
//      case 5:
//        joint_config[LEFT_ARM] = {-1.4, 0.0, 1.57, 0.23, 0.0, 0.28, 0.0};
//        joint_config[RIGHT_ARM] = {1.4, 0.0, -1.57, -0.23, 0.0, -0.28, 0.0};
//        joint_config[TORSO] = {0.0, 0.96};
//        break;
    }

    trajectory_control_helper->goToJointConfiguration(joint_config, 3.0, true);
  }
}
}
