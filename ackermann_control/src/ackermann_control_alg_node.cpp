#include "ackermann_control_alg_node.h"

AckermannControlAlgNode::AckermannControlAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<AckermannControlAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->last_pose_.pose.pose.position.x = 0.0;
  this->last_goal_.pose.position.x = 0.0;

  // [init publishers]
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);

  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_sim", 1, &AckermannControlAlgNode::cb_getPoseMsg,
                                                               this);
  this->goal_subscriber_ = this->public_node_handle_.subscribe("/move_base_simple/goal", 1,
                                                               &AckermannControlAlgNode::cb_getGoalMsg, this);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

AckermannControlAlgNode::~AckermannControlAlgNode(void)
{
  // [free dynamic memory]
}

void AckermannControlAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void AckermannControlAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();
  this->last_pose_.pose.pose.position.x = pose_msg->pose.pose.position.x;
  this->last_pose_.pose.pose.position.y = pose_msg->pose.pose.position.y;
  this->last_pose_.pose.pose.position.z = pose_msg->pose.pose.position.z;
  this->last_pose_.pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
  this->last_pose_.pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
  this->last_pose_.pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
  this->last_pose_.pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
  ROS_INFO("pose -> x: %f, y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
  this->alg_.unlock();
}
void AckermannControlAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();
  this->last_goal_.pose.position.x = goal_msg->pose.position.x;
  this->last_goal_.pose.position.y = goal_msg->pose.position.y;
  this->last_goal_.pose.position.z = goal_msg->pose.position.z;
  this->last_goal_.pose.orientation.x = goal_msg->pose.orientation.x;
  this->last_goal_.pose.orientation.y = goal_msg->pose.orientation.y;
  this->last_goal_.pose.orientation.z = goal_msg->pose.orientation.z;
  this->last_goal_.pose.orientation.w = goal_msg->pose.orientation.w;
  ROS_INFO("goal -> x: %f, y: %f", goal_msg->pose.position.x, goal_msg->pose.position.y);
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AckermannControlAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void AckermannControlAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<AckermannControlAlgNode>(argc, argv, "ackermann_control_alg_node");
}
