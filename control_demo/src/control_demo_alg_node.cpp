#include "control_demo_alg_node.h"

ControlDemoAlgNode::ControlDemoAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<ControlDemoAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->last_pose_.pose.pose.position.x = 0.0;
  this->last_goal_.pose.position.x = 0.0;

  // [init publishers]
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);

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

  if (this->alg_.flag_goal_active_)
  {
    int debug = this->alg_.controlLoop(this->last_pose_, this->last_goal_, this->desired_ackermann_state_);

    //debug
    if (debug == OK)
    {
      ROS_INFO("OK");
    }
    else if (debug == CROSSED_GOAL)
    {
      ROS_INFO("CROSSED_GOAL");
      this->alg_.flag_goal_active_ = false;
    }
    else if (debug == BAD_ORIENTATION)
    {
      ROS_INFO("BAD_ORIENTATION");
      this->alg_.flag_goal_active_ = false;
    }
  }

  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->ackermann_publisher_.publish(this->desired_ackermann_state_);
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
