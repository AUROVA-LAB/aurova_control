#include "control_demo_alg.h"

ControlDemoAlgorithm::ControlDemoAlgorithm(void)
{
  this->flag_goal_active_ = false;

  this->control_ = new StateSpaceControl();

  pthread_mutex_init(&this->access_, NULL);
}

ControlDemoAlgorithm::~ControlDemoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ControlDemoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// ControlDemoAlgorithm Public API
int ControlDemoAlgorithm::controlLoop(geometry_msgs::PoseWithCovarianceStamped last_pose,
                                      geometry_msgs::PoseStamped last_goal,
                                      ackermann_msgs::AckermannDriveStamped& ackermann_state)
{
  struct PoseCtrl st_goal;
  struct PoseCtrl st_pose;
  int restrictions_flag = OK;

  //parse last position from ROS msg to control class structure
  st_pose.x = last_pose.pose.pose.position.x;
  st_pose.y = last_pose.pose.pose.position.y;
  st_pose.z = last_pose.pose.pose.position.z;
  tf::Quaternion q_pose(last_pose.pose.pose.orientation.x, last_pose.pose.pose.orientation.y,
                        last_pose.pose.pose.orientation.z, last_pose.pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(st_pose.roll, st_pose.pitch, st_pose.yaw);

  //parse last goal from ROS msg to control class structure
  st_goal.x = last_goal.pose.position.x;
  st_goal.y = last_goal.pose.position.y;
  st_goal.z = last_goal.pose.position.z;
  tf::Quaternion q_goal(last_goal.pose.orientation.x, last_goal.pose.orientation.y, last_goal.pose.orientation.z,
                        last_goal.pose.orientation.w);
  tf::Matrix3x3 m_goal(q_goal);
  m_goal.getRPY(st_goal.roll, st_goal.pitch, st_goal.yaw);

  //////////////////////////////////////////////////////////////////////
  ////// 1) CHECK THE RECTRICTIONS IN THE RELATION POSE-GOAL
  restrictions_flag = this->control_->checkRestrictions(st_pose, st_goal);

  if (restrictions_flag == OK)
  {
    //////////////////////////////////////////////////////////////////////
    ////// 2) GET ERROR SIGNALS

    //////////////////////////////////////////////////////////////////////
    ////// 3) GENERATE CONTROL SIGNAL
  }

  return restrictions_flag;
}
