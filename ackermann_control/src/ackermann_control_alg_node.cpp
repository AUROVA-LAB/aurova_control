#include "ackermann_control_alg_node.h"

AckermannControlAlgNode::AckermannControlAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannControlAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  //get application parameters
  double t_length, delta_time, delta_angle, t_velocity;
  int vectors_size = 4;
  this->public_node_handle_.getParam("/ackermann_control/max_angle", this->params_.maxAngle);
  this->public_node_handle_.getParam("/ackermann_control/v_length", this->params_.l);
  this->public_node_handle_.getParam("/ackermann_control/v_width", this->params_.w);
  this->public_node_handle_.getParam("/ackermann_control/v_height", this->params_.h);
  this->public_node_handle_.getParam("/ackermann_control/t_length", t_length);
  this->public_node_handle_.getParam("/ackermann_control/delta_time", delta_time);
  this->public_node_handle_.getParam("/ackermann_control/delta_angle", delta_angle);
  this->public_node_handle_.getParam("/ackermann_control/t_velocity", t_velocity);
  this->public_node_handle_.getParam("/ackermann_control/v_min", this->v_min_);
  this->public_node_handle_.getParam("/ackermann_control/v_max", this->v_max_);

  // class constructor for control
  this->control_ = new Steering_Control(this->params_, t_length, delta_time, delta_angle, t_velocity);

  // inicializations of poses
  this->pose_.coordinates.resize(vectors_size);
  this->pose_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->pose_.matrix[i].resize(vectors_size);
  this->goal_.coordinates.resize(vectors_size);
  this->goal_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->goal_.matrix[i].resize(vectors_size);

  // [init publishers]
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);

  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_sim", 1, &AckermannControlAlgNode::cb_getPoseMsg,
                                                               this);
  this->goal_subscriber_ = this->public_node_handle_.subscribe("/semilocal_goal", 1,
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
  float k_sp = (this->v_max_ - this->v_min_) / this->params_.maxAngle;
  double speed = 0.0;
  this->direction_ = this->control_->getBestSteering(this->pose_, this->goal_);

  switch (this->direction_.sense)
  {
    case 0:
      speed = 0.0;
      break;
    case 1:
      speed = this->v_max_ - fabs(this->direction_.angle) * k_sp;
      break;
    case -1:
      speed = -1 * (this->v_max_ - fabs(this->direction_.angle) * k_sp);
      break;
  }

  this->ackermann_state_.drive.steering_angle = this->direction_.angle;
  this->ackermann_state_.drive.speed = speed;

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->ackermann_publisher_.publish(this->ackermann_state_);

}

/*  [subscriber callbacks] */
void AckermannControlAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
                        pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);

  this->pose_.coordinates.at(0) = pose_msg->pose.pose.position.x;
  this->pose_.coordinates.at(1) = pose_msg->pose.pose.position.y;
  this->pose_.coordinates.at(2) = pose_msg->pose.pose.position.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = pose_msg->pose.covariance[35];

  this->alg_.unlock();
}
void AckermannControlAlgNode::cb_getGoalMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(goal_msg->pose.pose.orientation.x, goal_msg->pose.pose.orientation.y,
                        goal_msg->pose.pose.orientation.z, goal_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);

  this->goal_.coordinates.at(0) = goal_msg->pose.pose.position.x;
  this->goal_.coordinates.at(1) = goal_msg->pose.pose.position.y;
  this->goal_.coordinates.at(2) = goal_msg->pose.pose.position.z;
  this->goal_.coordinates.at(3) = yaw;
  this->goal_.matrix[0][0] = goal_msg->pose.covariance[0];
  this->goal_.matrix[1][1] = goal_msg->pose.covariance[7];
  this->goal_.matrix[2][2] = goal_msg->pose.covariance[14];
  this->goal_.matrix[3][3] = goal_msg->pose.covariance[35];

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AckermannControlAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void AckermannControlAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < AckermannControlAlgNode > (argc, argv, "ackermann_control_alg_node");
}
