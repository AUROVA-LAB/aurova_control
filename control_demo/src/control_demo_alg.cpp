#include "control_demo_alg.h"

ControlDemoAlgorithm::ControlDemoAlgorithm(void)
{
  this->flag_goal_active_ = false;

  this->flag_pose_active_ = false;

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
                                      geometry_msgs::PoseStamped last_goal, float d_vehicle,
                                      ackermann_msgs::AckermannDriveStamped& ackermann_state,
                                      geometry_msgs::Twist& twist_state)
{
  int status = OK;
  float error_d, error_a;
  float steering, speed;

  //parse last position from ROS msg to control class structure
  this->control_->st_pose_.x = last_pose.pose.pose.position.x;
  this->control_->st_pose_.y = last_pose.pose.pose.position.y;
  this->control_->st_pose_.z = last_pose.pose.pose.position.z;
  tf::Quaternion q_pose(last_pose.pose.pose.orientation.x, last_pose.pose.pose.orientation.y,
                        last_pose.pose.pose.orientation.z, last_pose.pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(this->control_->st_pose_.roll, this->control_->st_pose_.pitch, this->control_->st_pose_.yaw);

  //parse last goal from ROS msg to control class structure
  this->control_->st_goal_.x = last_goal.pose.position.x;
  this->control_->st_goal_.y = last_goal.pose.position.y;
  this->control_->st_goal_.z = last_goal.pose.position.z;
  tf::Quaternion q_goal(last_goal.pose.orientation.x, last_goal.pose.orientation.y, last_goal.pose.orientation.z,
                        last_goal.pose.orientation.w);
  tf::Matrix3x3 m_goal(q_goal);
  m_goal.getRPY(this->control_->st_goal_.roll, this->control_->st_goal_.pitch, this->control_->st_goal_.yaw);

  //////////////////////////////////////////////////////////////////////
  ////// 1) CHECK THE RECTRICTIONS IN THE RELATION POSE-GOAL
  status = this->control_->checkRestrictions();

  if (status == OK)
  {
    //////////////////////////////////////////////////////////////////////
    ////// 2) GET ERROR SIGNALS
    this->control_->calculationErrorSignals(error_d, error_a);
    error_a = error_a * 180 / PI;
    ROS_INFO("error d: %f", error_d); //debug
    ROS_INFO("error a: %f", error_a);

    //////////////////////////////////////////////////////////////////////
    ////// 3) GENERATE CONTROL SIGNALS
    this->control_->calculationControlSignals(steering, speed, error_d, error_a);
    ROS_INFO("speed    d: %f", speed); //debug
    ROS_INFO("steering a: %f", steering);
    ackermann_state.drive.steering_angle = steering;
    ackermann_state.drive.speed = speed;
    
    //////////////////////////////////////////////////////////////////////
    ////// 3.b) GENERATE CONTROL SIGNALS FOR TWIST MSG
    twist_state.linear.x = speed;
    twist_state.angular.z = (speed / d_vehicle) * sin((steering*3.1416) / 180.0);
    
  }

  return status;
}
