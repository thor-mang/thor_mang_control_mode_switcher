#include <thor_mang_control_mode_switcher/trajectory_control_helper.h>



namespace control_mode_switcher
{
TrajectoryControlHelper::TrajectoryControlHelper()
{
  left_hand_trajectory_client_ = new TrajectoryActionClient("joints/left_hand_traj_controller/follow_joint_trajectory", true);
  right_hand_trajectory_client_ = new TrajectoryActionClient("joints/right_hand_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> left_arm_joints;
  left_arm_joints.push_back("l_arm_sh_p1");
  left_arm_joints.push_back("l_arm_sh_r");
  left_arm_joints.push_back("l_arm_sh_p2");
  left_arm_joints.push_back("l_arm_el_y");
  left_arm_joints.push_back("l_arm_wr_r");
  left_arm_joints.push_back("l_arm_wr_y");
  left_arm_joints.push_back("l_arm_wr_p");
  controller_joints[LEFT_ARM] = left_arm_joints;
  controller_clients[LEFT_ARM] = new TrajectoryActionClient("joints/left_arm_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> right_arm_joints;
  right_arm_joints.push_back("r_arm_sh_p1");
  right_arm_joints.push_back("r_arm_sh_r");
  right_arm_joints.push_back("r_arm_sh_p2");
  right_arm_joints.push_back("r_arm_el_y");
  right_arm_joints.push_back("r_arm_wr_r");
  right_arm_joints.push_back("r_arm_wr_y");
  right_arm_joints.push_back("r_arm_wr_p");
  controller_joints[RIGHT_ARM] = right_arm_joints;
  controller_clients[RIGHT_ARM] = new TrajectoryActionClient("joints/right_arm_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> left_leg_joints;
  left_leg_joints.push_back("l_leg_hip_y");
  left_leg_joints.push_back("l_leg_hip_r");
  left_leg_joints.push_back("l_leg_hip_p");
  left_leg_joints.push_back("l_leg_kn_p");
  left_leg_joints.push_back("l_leg_an_r");
  left_leg_joints.push_back("l_leg_an_p");
  controller_joints[LEFT_LEG] = left_leg_joints;
  controller_clients[LEFT_LEG] = new TrajectoryActionClient("joints/left_leg_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> right_leg_joints;
  right_leg_joints.push_back("r_leg_hip_y");
  right_leg_joints.push_back("r_leg_hip_r");
  right_leg_joints.push_back("r_leg_hip_p");
  right_leg_joints.push_back("r_leg_kn_p");
  right_leg_joints.push_back("r_leg_an_r");
  right_leg_joints.push_back("r_leg_an_p");
  controller_joints[RIGHT_LEG] = right_leg_joints;
  controller_clients[RIGHT_LEG] = new TrajectoryActionClient("joints/right_leg_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> head_joints;
  head_joints.push_back("head_p");
  head_joints.push_back("head_y");
  controller_joints[HEAD] = head_joints;
  controller_clients[HEAD] = new TrajectoryActionClient("joints/head_traj_controller/follow_joint_trajectory", true);

  std::vector<std::string> torso_joints;
  torso_joints.push_back("torso_y");
  controller_joints[TORSO] = torso_joints;
  controller_clients[TORSO] = new TrajectoryActionClient("joints/torso_traj_controller/follow_joint_trajectory", true);
}

TrajectoryControlHelper::~TrajectoryControlHelper()
{}

void TrajectoryControlHelper::goToJointConfiguration(std::map<TrajectoryController, std::vector<double>> joint_config, float duration, bool wait_till_finished)
{
  wait_till_trajectory_finished_ = wait_till_finished;
  completion_counter_ = joint_config.size();

  for (std::map<TrajectoryController, std::vector<double> >::iterator iter = joint_config.begin(); iter != joint_config.end(); iter++)
  {
    std::vector<std::string> names = controller_joints[iter->first];
    std::vector<double> positions = iter->second;

    control_msgs::FollowJointTrajectoryGoal trajectory_goal;

    if (!controller_clients[iter->first]->waitForServer(ros::Duration(5.0)))
      ROS_WARN("[ControlModeSwitcher] Time out while waiting for '%s' trajectory controller", trajectoryControllerIdToString(iter->first).c_str());

    if (controller_clients[iter->first]->isServerConnected())
    {
      trajectory_msgs::JointTrajectory joint_trajectory;
      joint_trajectory.joint_names = names;

      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = ros::Duration(duration);
      joint_trajectory.points.push_back(point);

      trajectory_goal.trajectory = joint_trajectory;

      // Send goals to controllers
      controller_clients[iter->first]->sendGoal(trajectory_goal, boost::bind(&TrajectoryControlHelper::trajectoryDoneCb, this, _1, _2),
                                                boost::bind(&TrajectoryControlHelper::trajectoryActiveCB, this),
                                                boost::bind(&TrajectoryControlHelper::trajectoryFeedbackCB, this, _1));
    }
  }

  if (wait_till_trajectory_finished_)
  {
    ROS_INFO("[ControlModeSwitcher] waiting for finished feedback of trajectory control action ...");
    //TODO testing
    ros::Duration max_wait_time(2.0f*duration);
    ros::Time begin = ros::Time::now();
    ros::Rate rate(ros::Duration(0.1));
    while (completion_counter_ > 0)
    {
      if (ros::Time::now() - begin > max_wait_time)
      {
        ROS_WARN("[ControlModeSwitcher] Timeout in send trajectory");
        completion_counter_ = 0;
        break;
      }
      rate.sleep();
    }
  }

}

void TrajectoryControlHelper::trajectoryActiveCB()
{
}

void TrajectoryControlHelper::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
}

void TrajectoryControlHelper::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  completion_counter_--;
}


std::string TrajectoryControlHelper::trajectoryControllerIdToString(int id) const
{
  switch(id)
  {
    case TORSO: return std::string("torso"); break;
    case HEAD: return std::string("head"); break;
    case LEFT_ARM: return std::string("left_arm"); break;
    case RIGHT_ARM: return std::string("right_arm"); break;
    case LEFT_HAND: return std::string("left_hand"); break;
    case RIGHT_HAND: return std::string("right_hand"); break;
    case LEFT_LEG: return std::string("left leg"); break;
    case RIGHT_LEG: return std::string("right leg"); break;
    default: std::string("unknown"); break;
  }
}
}
