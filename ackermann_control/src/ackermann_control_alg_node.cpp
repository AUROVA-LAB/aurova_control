#include "ackermann_control_alg_node.h"

AckermannControlAlgNode::AckermannControlAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannControlAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  int vectors_size = 4;
  this->pose_.coordinates.resize(vectors_size);
  this->pose_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->pose_.matrix[i].resize(vectors_size);
  this->goal_.coordinates.resize(vectors_size);
  this->goal_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->goal_.matrix[i].resize(vectors_size);

  /////////////////////////////////////////////////////////////
  //// TODO: get from params
  this->params_.maxAngle = 25;
  this->params_.l = 1.08;
  this->params_.w = 1.25;
  this->params_.h = 0.5;
  double length = 5.0;
  double deltaTime = 0.1;
  double deltaAngle = 1.0;
  double velocity = 1.0;
  this->control_ = new Steering_Control(this->params_, length, deltaTime, deltaAngle, velocity);
  /////////////////////////////////////////////////////////////

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
  float v_max = 1.3; // TODO: get from params
  float v_min = 0.6;
  float k_sp = (v_max - v_min) / this->params_.maxAngle;
  double speed = 0.0;
  this->direction_ = this->control_->getBestSteering(this->pose_, this->goal_);
  //ROS_INFO("action -> a: %f, s: %d", (float)(this->direction_.angle), this->direction_.sense);

  switch (this->direction_.sense)
  {
    case 0:
      speed = 0.0;
      break;
    case 1:
      speed = v_max - fabs(this->direction_.angle) * k_sp;
      break;
    case -1:
      speed = -1 * (v_max - fabs(this->direction_.angle) * k_sp);
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
  this->pose_.matrix[0][0] = 1.0; //pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = 1.0; //pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = 1.0; //pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = 0.001; //pose_msg->pose.covariance[35];

  //ROS_INFO("pose -> x: %f, y: %f", this->pose_.coordinates.at(1), this->pose_.matrix[1].at(1));
  this->alg_.unlock();
}
void AckermannControlAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(goal_msg->pose.orientation.x, goal_msg->pose.orientation.y, goal_msg->pose.orientation.z,
                        goal_msg->pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);

  this->goal_.coordinates.at(0) = goal_msg->pose.position.x;
  this->goal_.coordinates.at(1) = goal_msg->pose.position.y;
  this->goal_.coordinates.at(2) = goal_msg->pose.position.z;
  this->goal_.coordinates.at(3) = yaw;
  this->goal_.matrix[0][0] = 1.0;
  this->goal_.matrix[1][1] = 1.0;
  this->goal_.matrix[2][2] = 1.0;
  this->goal_.matrix[3][3] = 0.001;

  //ROS_INFO("goal -> x: %f, y: %f", this->goal_.coordinates.at(0), this->goal_.coordinates.at(1));
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
