#include "control_demo_alg_node.h"

ControlDemoAlgNode::ControlDemoAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<ControlDemoAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 25; //in [Hz]
  this->flag_request_goal_.data = true;
  this->last_pose_.pose.pose.position.x = 0.0;
  this->last_goal_.pose.position.x = 0.0;

  // [init publishers]
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);
  this->twist_publisher_ = this->public_node_handle_.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);
  this->request_publisher_ = this->public_node_handle_.advertise < std_msgs::Bool > ("/request_goal", 1);

  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/amcl_pose", 1, &ControlDemoAlgNode::cb_getPoseMsg,
                                                               this);
  this->goal_subscriber_ = this->public_node_handle_.subscribe("/move_base_simple/goal", 1,
                                                               &ControlDemoAlgNode::cb_getGoalMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

ControlDemoAlgNode::~ControlDemoAlgNode(void)
{
  // [free dynamic memory]
}

void ControlDemoAlgNode::mainNodeThread(void)
{
  static bool first_exec = true;
  static double t_1;
  static double t_2;
  int status = -1;

  //parameter reading
  if (first_exec)
  {
    this->public_node_handle_.getParam("/control_demo/ku_d", this->alg_.control_->ku_d_);
    this->public_node_handle_.getParam("/control_demo/ku_a", this->alg_.control_->ku_a_);
    this->public_node_handle_.getParam("/control_demo/kv_d", this->alg_.control_->kv_d_);
    this->public_node_handle_.getParam("/control_demo/kv_a", this->alg_.control_->kv_a_);
    this->public_node_handle_.getParam("/control_demo/v_base", this->alg_.control_->v_base_);
    this->public_node_handle_.getParam("/control_demo/v_max", this->alg_.control_->v_max_);
    this->public_node_handle_.getParam("/control_demo/max_steering", this->alg_.control_->max_steering_);
    this->public_node_handle_.getParam("/control_demo/d_vehicle", this->alg_.control_->d_vehicle_);
    this->public_node_handle_.getParam("/time_out_wait_goal", this->alg_.control_->time_out_wait_goal_);
    this->public_node_handle_.getParam("/error_d_sat", this->alg_.control_->error_d_sat_);

    first_exec = false;
  }

  //control loop
  if (this->alg_.flag_goal_active_ && this->alg_.flag_pose_active_)
  {
    status = this->alg_.controlLoop(this->last_pose_, this->last_goal_, this->alg_.control_->d_vehicle_,
                                    this->desired_ackermann_state_, this->desired_twist_state_);

    //debug
    if (status == OK)
    {
      ROS_INFO("OK");
    }
    else if (status == CROSSED_GOAL)
    {
      ROS_INFO("CROSSED_GOAL");
      this->alg_.flag_goal_active_ = false;
    }
    else if (status == BAD_ORIENTATION)
    {
      ROS_INFO("BAD_ORIENTATION");
      this->alg_.flag_goal_active_ = false;
    }

    this->flag_request_goal_.data = false;
  }else{
    t_2 = (double)ros::Time::now().toSec();
    if ((t_2 - t_1) > this->alg_.control_->time_out_wait_goal_)
    {
      this->desired_ackermann_state_.drive.steering_angle = 0.0;
      this->desired_ackermann_state_.drive.speed = 0.0;
      this->desired_twist_state_.linear.x = 0.0;
      this->desired_twist_state_.angular.z = 0.0;
      ROS_INFO("TIME_OUT_WAITING_NEW_GOAL");
    }
  }

  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->ackermann_publisher_.publish(this->desired_ackermann_state_);
  this->twist_publisher_.publish(this->desired_twist_state_);
  this->request_publisher_.publish(this->flag_request_goal_);

  //this is for generate up flank in the next loop.
  if (status == CROSSED_GOAL || status == BAD_ORIENTATION)
  {
    this->flag_request_goal_.data = true;
    t_1 = (double)ros::Time::now().toSec();
  }
}

/*  [subscriber callbacks] */
void ControlDemoAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();
  this->last_pose_.pose.pose.position.x = pose_msg->pose.pose.position.x;
  this->last_pose_.pose.pose.position.y = pose_msg->pose.pose.position.y;
  this->last_pose_.pose.pose.position.z = pose_msg->pose.pose.position.z;
  this->last_pose_.pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
  this->last_pose_.pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
  this->last_pose_.pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
  this->last_pose_.pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
  this->alg_.flag_pose_active_ = true;
  this->alg_.unlock();
}
void ControlDemoAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();
  this->last_goal_.pose.position.x = goal_msg->pose.position.x;
  this->last_goal_.pose.position.y = goal_msg->pose.position.y;
  this->last_goal_.pose.position.z = goal_msg->pose.position.z;
  this->last_goal_.pose.orientation.x = goal_msg->pose.orientation.x;
  this->last_goal_.pose.orientation.y = goal_msg->pose.orientation.y;
  this->last_goal_.pose.orientation.z = goal_msg->pose.orientation.z;
  this->last_goal_.pose.orientation.w = goal_msg->pose.orientation.w;
  this->alg_.flag_goal_active_ = true;
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void ControlDemoAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void ControlDemoAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < ControlDemoAlgNode > (argc, argv, "control_demo_alg_node");
}
