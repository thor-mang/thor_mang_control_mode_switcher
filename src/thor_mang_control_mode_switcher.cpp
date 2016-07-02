#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>

#include <robotis_controller_msgs/JointCtrlModule.h>
#include <robotis_controller_msgs/SetJointCtrlModule.h>
#include <robotis_controller_msgs/SetCtrlModule.h>

#include <thor_mang_control_msgs/ControlModeStatus.h>

#include <thor_mang_control_mode_switcher/thor_mang_mode_switch_handles.h>



namespace control_mode_switcher
{
ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle& nh)
  : nh_(nh)
  , allow_all_mode_transitions_(false)
  , current_mode_name_("none")
  , get_control_modes_action_server_(nh, "control_mode_switcher/get_control_modes", boost::bind(&ControlModeSwitcher::getControlModesCallback, this, _1), false)
  , change_control_mode_action_server_(nh, "control_mode_switcher/change_control_mode", boost::bind(&ControlModeSwitcher::executeSwitchControlModeCallback, this, _1), false)

{
  // load control mode table
  parseParameters(nh_);

  // ros subscriber
  execute_footstep_sub_ = nh_.subscribe("/vigir/footstep_manager/execute_step_plan/goal", 1, &ControlModeSwitcher::executeFootstepCb, this);
  // result_footstep_sub_ = nh_.subscribe("/vigir/footstep_manager/execute_step_plan/result", 1, &ControlModeSwitcher::resultFootstepCb, this);
  allow_all_mode_transitions_sub_ = nh_.subscribe("control_mode_switcher/allow_all_mode_transitions", 1, &ControlModeSwitcher::allowAllModeTransitionsCb, this);

  // ros publisher
  // mode_changed_pub_ = nh_.advertise<vigir_control_msgs::VigirControlMode>("/flor/controller/mode", 10, true);
  robot_init_pose_pub_ = nh_.advertise<std_msgs::String>("robotis/base/ini_pose", 1, false);
  control_mode_status_pub_ = nh_.advertise<thor_mang_control_msgs::ControlModeStatus>("control_mode_switcher/status", 1, true);
  allow_all_mode_transitions_ack_pub_ = nh_.advertise<std_msgs::Bool>("control_mode_switcher/allow_all_mode_transitions_ack", 1, true);

  // ros services
  set_joint_ctrl_modules_client_ = nh_.serviceClient<robotis_controller_msgs::SetJointCtrlModule>("robotis/set_joint_ctrl_modules");
  enable_ctrl_modules_client_ = nh_.serviceClient<robotis_controller_msgs::SetCtrlModule>("robotis/enable_ctrl_module");
  switch_ros_controllers_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("joints/controller_manager/switch_controller");
  list_ros_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("joints/controller_manager/list_controllers");

  // ros action clients
  //trajectory_client_ = new  TrajectoryActionClient("/vigir_move_group", true);
  trajectory_client_left_ = new TrajectoryActionClient("joints/left_arm_traj_controller/follow_joint_trajectory", true);
  trajectory_client_right_ = new TrajectoryActionClient("joints/right_arm_traj_controller/follow_joint_trajectory", true);

  // start all action servers
  get_control_modes_action_server_.start();
  change_control_mode_action_server_.start();

  // add handlers
  TrajectoryControlHelper::Ptr trajectory_control_helper(new TrajectoryControlHelper());
  //StepPlanActionClientPtr step_plan_action_client(new StepPlanActionClient("step_controller/execute_step_plan", true));

  /// TODO: More generic
  modes_["stand_prep"].mode_switch_handle_ = boost::bind(&goToStandPrepMode, robot_init_pose_pub_);
  modes_["stand"].mode_switch_handle_ = boost::bind(&goToStandMode, trajectory_control_helper);
  //modes_["soft_stop"].mode_switch_handle_ = boost::bind(&goToSoftStop, step_plan_action_client);
  modes_["shutdown"].mode_switch_handle_ = boost::bind(&goToShutdownMode, trajectory_control_helper);

  // publish initial info
  control_mode_status_pub_.publish(getCurrentControlModeStatus());

  std_msgs::Bool ack;
  ack.data = false;
  allow_all_mode_transitions_ack_pub_.publish(ack);

  getStartedAndStoppedRosControllers();

  //       vigir_control_msgs::VigirControlMode changed_mode_msg;
  //       changed_mode_msg.header.stamp = ros::Time::now();
  //       changed_mode_msg.bdi_current_behavior = 1;
  //       changed_mode_msg.control_mode = 0;
  //
  //       notifyNewControlMode("none", 0, changed_mode_msg);
}

ControlModeSwitcher::~ControlModeSwitcher()
{}

thor_mang_control_msgs::ControlModeStatus ControlModeSwitcher::getCurrentControlModeStatus() const
{
  thor_mang_control_msgs::ControlModeStatus status;

  std::map<std::string, Mode>::const_iterator itr = modes_.find(current_mode_name_);
  if (itr == modes_.end())
  {
    ROS_ERROR("[ControlModeSwitcher] getCurrentControlModeStatus: We are in a unknown mode! Fix it immediately!");
    return status;
  }

  const Mode& mode = itr->second;

  status.header.stamp = ros::Time::now();
  status.current_control_mode = current_mode_name_;
  status.allowed_control_modes = mode.allowed_transitions_;
  status.status = thor_mang_control_msgs::ControlModeStatus::NO_ERROR;
}

void ControlModeSwitcher::getStartedAndStoppedRosControllers()
{
  started_controllers_.clear();
  stopped_controllers_.clear();

  controller_manager_msgs::ListControllers srv;
  list_ros_controllers_client_.call(srv);

  for (int i = 0; i < srv.response.controller.size(); i++)
  {
    controller_manager_msgs::ControllerState controller = srv.response.controller[i];
    if (controller.state == "running")
      started_controllers_.push_back(controller.name);
    else
      stopped_controllers_.push_back(controller.name);
  }
}

bool ControlModeSwitcher::switchRosControllers(std::vector<std::string> desired_controllers_to_start)
{
  std::vector<std::string> controllers_to_stop;
  std::vector<std::string> controllers_to_start;

  // Add undesired running controllers to stoping list
  for (int i = 0; i < started_controllers_.size(); i++)
  {
    bool should_be_stopped = true;
    for (int j = 0; j < desired_controllers_to_start.size(); j++)
    {
      if (started_controllers_[i] == desired_controllers_to_start[j])
        should_be_stopped = false;
    }

    if (should_be_stopped)
      controllers_to_stop.push_back(started_controllers_[i]);
  }

  // Remove already running controllers from starting list
  for (int i = 0; i < desired_controllers_to_start.size(); i++)
  {
    bool already_running = false;
    for (int j = 0; j < started_controllers_.size(); j++)
    {
      if (started_controllers_[j] == desired_controllers_to_start[i])
        already_running = true;
    }
    if (!already_running)
      controllers_to_start.push_back(desired_controllers_to_start[i]);
  }

  if ((controllers_to_start.size() > 0) || (controllers_to_stop.size() > 0))
  {
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = controllers_to_start;
    srv.request.stop_controllers = controllers_to_stop;
    if (!allow_all_mode_transitions_)
      srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    else
      srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    switch_ros_controllers_client_.call(srv);
    return srv.response.ok;
  }
  else
  {
    ROS_DEBUG("[ControlModeSwitcher] No changes made, all controllers already running");
    return true;
  }
}

void ControlModeSwitcher::getControlModesCallback(const thor_mang_control_msgs::GetControlModesGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_control_modes_action_server_.isPreemptRequested())
  {
    get_control_modes_action_server_.setPreempted();
    return;
  }

  thor_mang_control_msgs::GetControlModesResult result;
  for (std::map<std::string, Mode>::const_iterator itr = modes_.begin(); itr != modes_.end(); itr++)
  {
    if (itr->first != "all" && itr->first != "none")
      result.available_control_modes.push_back(itr->first);
  }

  get_control_modes_action_server_.setSucceeded(result);
}

void ControlModeSwitcher::executeSwitchControlModeCallback(const thor_mang_control_msgs::ChangeControlModeGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (change_control_mode_action_server_.isPreemptRequested())
  {
    change_control_mode_action_server_.setPreempted();
    return;
  }

  std::string mode_request = goal->mode_request;
  ROS_INFO("[ControlModeSwitcher] Processing new mode change request for %s ... ", mode_request.c_str());

  thor_mang_control_msgs::ChangeControlModeResult result;
  result.result = changeControlMode(mode_request);

  // If requested mode in known publish changed mode
  if (result.result.status == thor_mang_control_msgs::ControlModeStatus::MODE_ACCEPTED)
  {
    // Set Action Goal as succeeded
    change_control_mode_action_server_.setSucceeded(result);
    ROS_INFO("DONE!");
  }
  else
  {
    change_control_mode_action_server_.setAborted(result);
    ROS_INFO("FAILED!");
  }
}

void ControlModeSwitcher::allowAllModeTransitionsCb(const std_msgs::Bool& allow)
{
  allow_all_mode_transitions_ = allow.data;

  std_msgs::Bool ack;
  ack.data = allow_all_mode_transitions_;
  allow_all_mode_transitions_ack_pub_.publish(ack);

  if (allow_all_mode_transitions_)
    ROS_INFO("[ControlModeSwitcher] Allowing all transitions between modes");
  else
    ROS_INFO("[ControlModeSwitcher] Restrictive transitions between modes");
}

thor_mang_control_msgs::ControlModeStatus ControlModeSwitcher::changeControlMode(std::string requested_mode)
{
  // Ignore case
  std::transform(requested_mode.begin(), requested_mode.end(), requested_mode.begin(), ::tolower);

  Mode current_mode = modes_[current_mode_name_];

  // init feedback message
  thor_mang_control_msgs::ControlModeStatus control_mode_status;
  control_mode_status.header.stamp = ros::Time::now();
  control_mode_status.current_control_mode = current_mode_name_;
  control_mode_status.requested_control_mode = requested_mode;
  control_mode_status.allowed_control_modes = current_mode.allowed_transitions_;

  // test if the requested transition is allowed
  bool transition_ok = current_mode_name_ == requested_mode;

  // otherwise check if current state allows requested transition
  if (!transition_ok)
    transition_ok = std::find(current_mode.allowed_transitions_.begin(), current_mode.allowed_transitions_.end(), requested_mode) != current_mode.allowed_transitions_.end();

  if (allow_all_mode_transitions_)
  {
    if (!transition_ok)
      ROS_WARN("[ControlModeSwitcher] Normally not allowed to switch from '%s' to '%s' - but currently manually enabled", current_mode_name_.c_str(), requested_mode.c_str());

    transition_ok = true;
  }

  if (!transition_ok)
  {
    ROS_WARN("[ControlModeSwitcher] Not allowed to switch from '%s' to '%s' - returning NOT SUCEEDED", current_mode_name_.c_str(), requested_mode.c_str());
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::MODE_REJECTED;
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::ERR_INVALID_TRANSITION;
    return control_mode_status;
  }

  // get new mode info
  std::map<std::string, Mode>::iterator itr = modes_.find(requested_mode);
  if (itr == modes_.end())
  {
    ROS_WARN("[ControlModeSwitcher] Mode '%s' is unknown!", requested_mode.c_str());
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::MODE_REJECTED;
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::ERR_INVALID_BEHAVIOR_MODE;
    return control_mode_status;
  }
  Mode new_mode = itr->second;

  // Publish changed mode
  //changed_mode_msg.header.stamp = ros::Time::now();

  bool switch_successfull = true;

  // set control modules
  for (std::string ctrl_module_name : new_mode.ctrl_modules_)
  {
    robotis_controller_msgs::SetCtrlModule srv;
    srv.request.module_name = ctrl_module_name;
    switch_successfull &= enable_ctrl_modules_client_.call(srv);
  }

  // set joint control modules
  if (!new_mode.joint_ctrl_modules_.empty())
  {
    robotis_controller_msgs::SetJointCtrlModule srv;
    for (std::pair<std::string, std::string> joint_ctrl_module : new_mode.joint_ctrl_modules_)
    {
      srv.request.joint_ctrl_modules.module_name.push_back(joint_ctrl_module.first);
      srv.request.joint_ctrl_modules.joint_name.push_back(joint_ctrl_module.second);
    }
    switch_successfull &= set_joint_ctrl_modules_client_.call(srv);
  }

  // switch ros controllers
  if (requested_mode != "none")
  {
    getStartedAndStoppedRosControllers();

    std::vector<std::string> controllers_to_start = new_mode.desired_controllers_;
    switch_successfull = switchRosControllers(controllers_to_start);
  }

  //changed_mode_msg.bdi_current_behavior = bdi_control_modes[mode_idx_int];
  //changed_mode_msg.control_mode = mode_idx_int;

  if (allow_all_mode_transitions_)
    switch_successfull = true;

  // call switch mode handler
  if (switch_successfull)
  {
    if (!new_mode.mode_switch_handle_.empty())
      new_mode.mode_switch_handle_();

    current_mode_name_ = requested_mode;

    control_mode_status.current_control_mode = requested_mode;
    control_mode_status.allowed_control_modes = new_mode.allowed_transitions_;
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::MODE_ACCEPTED;

    //notifyNewControlMode(mode_request, mode_idx_int, changed_mode_msg);
  }
  else
  {
    ROS_WARN("[ControlModeSwitcher] Not possible to switch to requested mode '%s'", requested_mode.c_str());
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::MODE_REJECTED;
    control_mode_status.status |= thor_mang_control_msgs::ControlModeStatus::ERR_INVALID_TRANSITION;
  }

  control_mode_status_pub_.publish(control_mode_status);

  return control_mode_status;
}

//     void ControlModeSwitcher::notifyNewControlMode(std::string new_mode, int new_idx, vigir_control_msgs::VigirControlMode msg){
//         mode_changed_pub_.publish(msg);
//         std_msgs::String mode_name;
//         mode_name.data = new_mode;
//         mode_name_pub_.publish(mode_name);
//         current_mode_ = new_mode;
//         current_mode_int_ = new_idx;
//         ROS_INFO("[ControlModeSwitcher] Successfully switched to mode %s !", new_mode.c_str());
//     }

void ControlModeSwitcher::trajectoryActiveCB()
{
}

void ControlModeSwitcher::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
}

void ControlModeSwitcher::trajectoryLeftArmDoneCb(const actionlib::SimpleClientGoalState& state,
                                                  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  stand_complete_right_ = true;
}

void ControlModeSwitcher::trajectoryRightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                                                   const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  stand_complete_left_ = true;
}

void ControlModeSwitcher::executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal)
{
  //if (!goal->goal.step_plan.steps.empty()){
  std::string new_mode = (current_mode_name_ == "stand_manipulate")? "walk_manipulate" : "walk";
  changeControlMode(new_mode);
  //}
  //TODO check for empty plans
  //}
}

void ControlModeSwitcher::resultFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResultConstPtr& result)
{
  //  if ( (current_mode_ == "walk") || (current_mode_ =="walk_manipulate") )
  //  {
  //    std::string new_mode = (current_mode_ == "walk_manipulate")? "stand_manipulate" : "stand";
  //    changeControlMode(new_mode);
  //  }
}

void ControlModeSwitcher::stepPlanActiveCb()
{
}

void ControlModeSwitcher::stepPlanDoneCb(const actionlib::SimpleClientGoalState& state,
                                         const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
}

void ControlModeSwitcher::parseParameters(ros::NodeHandle nh)
{
  std::vector<std::string> default_ctrl_modules;
  nh.getParam("control_mode_switcher/control_mode_to_controllers/all/ctrl_modules", default_ctrl_modules);

  XmlRpc::XmlRpcValue default_ctrl_modules_param;
  nh.getParam("control_mode_switcher/control_mode_to_controllers/all/joint_ctrl_modules", default_ctrl_modules_param);
  std::vector<std::pair<std::string, std::string>> default_joint_ctrl_modules = parseJointCtrlModules(default_ctrl_modules_param);

  std::vector<std::string> default_desired_controllers;
  nh.getParam("control_mode_switcher/control_mode_to_controllers/all/desired_controllers_to_start", default_desired_controllers);

  std::vector<std::string> default_allowed_transitions;
  nh.getParam("control_mode_switcher/control_mode_to_controllers/all/transitions", default_allowed_transitions);

  // add default transitions to initial state
  modes_[current_mode_name_].allowed_transitions_ = default_allowed_transitions;

  XmlRpc::XmlRpcValue modes;
  nh.getParam("control_mode_switcher/control_mode_to_controllers", modes);

  // iterate over all given modes
  if (modes.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (XmlRpc::XmlRpcValue::iterator modes_itr = modes.begin(); modes_itr != modes.end(); modes_itr++)
    {
      std::string mode_name = modes_itr->first;
      XmlRpc::XmlRpcValue attributes = modes_itr->second;

      // get module assignment for each joint for given mode
      if (attributes.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for (XmlRpc::XmlRpcValue::iterator attributes_itr = attributes.begin(); attributes_itr != attributes.end(); attributes_itr++)
        {
          std::string attribute_name = attributes_itr->first;

          // get control modules for each joint
          if (attribute_name == "joint_ctrl_modules")
          {
            modes_[mode_name].joint_ctrl_modules_ = parseJointCtrlModules(attributes_itr->second);
          }
          // load control modules to be started
          else if (attribute_name == "ctrl_modules" && attributes_itr->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            // get controllers to be active in given mode
            for (size_t idx = 0; idx < attributes_itr->second.size(); idx++)
              modes_[mode_name].ctrl_modules_.push_back(attributes_itr->second[idx]);
          }
          // load ros controllers to be started
          else if (attribute_name == "desired_controllers_to_start" && attributes_itr->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            // get controllers to be active in given mode
            for (size_t idx = 0; idx < attributes_itr->second.size(); idx++)
              modes_[mode_name].desired_controllers_.push_back(attributes_itr->second[idx]);
          }
          // load list of valid transitions from this mode
          else if (attribute_name == "transitions" && attributes_itr->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            // get allowed transitions in given mode
            for (size_t idx = 0; idx < attributes_itr->second.size(); idx++)
              modes_[mode_name].allowed_transitions_.push_back(attributes_itr->second[idx]);
          }
        }
      }
      else
        ROS_WARN("[ControlModeSwitcher] Couldn't load attributes of mode '%s' from param server.", mode_name.c_str());

      // add default control modules
      modes_[mode_name].ctrl_modules_.insert(modes_[mode_name].ctrl_modules_.end(), default_ctrl_modules.begin(), default_ctrl_modules.end());
      // add default joint control modules
      modes_[mode_name].joint_ctrl_modules_.insert(modes_[mode_name].joint_ctrl_modules_.end(), default_joint_ctrl_modules.begin(), default_joint_ctrl_modules.end());
      // add default controllers
      modes_[mode_name].desired_controllers_.insert(modes_[mode_name].desired_controllers_.end(), default_desired_controllers.begin(), default_desired_controllers.end());
      // add default transitions
      modes_[mode_name].allowed_transitions_.insert(modes_[mode_name].allowed_transitions_.end(), default_allowed_transitions.begin(), default_allowed_transitions.end());
    }
  }
  else
  {
    ROS_ERROR("[ControlModeSwitcher] Couldn't load modes from param server.");
    exit(-1);
  }
}

std::vector<std::pair<std::string, std::string>> ControlModeSwitcher::parseJointCtrlModules(XmlRpc::XmlRpcValue param) const
{
  std::vector<std::pair<std::string, std::string>> joint_ctrl_modules;

  if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // iterate over each motion module
    for (size_t module_idx = 0; module_idx < param.size(); module_idx++)
    {
      XmlRpc::XmlRpcValue module = param[module_idx];
      if (module.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("Couldn't load joint_ctrl_modules!");
        continue;
      }

      for (XmlRpc::XmlRpcValue::iterator module_itr = module.begin(); module_itr != module.end(); module_itr++)
      {
        std::string module_name = module_itr->first;
        XmlRpc::XmlRpcValue joints = module_itr->second;

        for (size_t joints_idx = 0; joints_idx < joints.size(); joints_idx++)
          joint_ctrl_modules.push_back(std::make_pair(module_name, joints[joints_idx]));
      }
    }
  }

  return joint_ctrl_modules;
}
}
