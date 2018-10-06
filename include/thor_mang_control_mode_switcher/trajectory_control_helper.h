#ifndef THOR_MANG_TRAJECTORY_HELPER_H
#define THOR_MANG_TRAJECTORY_HELPER_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>



namespace control_mode_switcher
{
enum TrajectoryController
{
  TORSO,
  HEAD,
  LEFT_ARM,
  RIGHT_ARM,
  LEFT_HAND,
  RIGHT_HAND,
  LEFT_LEG,
  RIGHT_LEG
};

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

class TrajectoryControlHelper
{
public:
  // typedefs
  typedef boost::shared_ptr<TrajectoryControlHelper> Ptr;
  typedef boost::shared_ptr<const TrajectoryControlHelper> ConstPtr;

  TrajectoryControlHelper();
  virtual ~TrajectoryControlHelper();

  void goToJointConfiguration(std::map<TrajectoryController , std::vector<double> > joint_config, float duration, bool wait_till_finished);

protected:
  void trajectoryActiveCB();
  void trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
  void trajectoryDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);

private:
  std::string trajectoryControllerIdToString(int id) const;

  ros::NodeHandle nh_;

  TrajectoryActionClient* left_hand_trajectory_client_;
  TrajectoryActionClient* right_hand_trajectory_client_;

  int completion_counter_;
  bool wait_till_trajectory_finished_;

  std::map<TrajectoryController, std::vector<std::string> > controller_joints;
  std::map<TrajectoryController, TrajectoryActionClient*> controller_clients;
};
}

#endif // THOR_MANG_TRAJECTORY_HELPER_H
