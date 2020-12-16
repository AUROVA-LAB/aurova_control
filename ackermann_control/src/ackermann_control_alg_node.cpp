#include "ackermann_control_alg_node.h"

AckermannControlAlgNode::AckermannControlAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannControlAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  //get application parameters
  double t_length, delta_time, delta_angle, t_velocity;
  int vectors_size = 4;
  this->flag_odom_ = false;
  this->flag_goal_ = false;
  this->public_node_handle_.getParam("/ackermann_control/frame_id", this->frame_id_);
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
  this->public_node_handle_.getParam("/ackermann_control/rad_reached", this->rad_reached_);
  this->public_node_handle_.getParam("/ackermann_control/flag_prop", this->flag_prop_);
  this->public_node_handle_.getParam("/ackermann_control/kp", this->kp_);

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
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDrive
      > ("/desired_ackermann_state", 1);
  this->twist_publisher_ = this->public_node_handle_.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);

  // [init subscribers]
  //this->odom_subscriber_ = this->public_node_handle_.subscribe("/odom", 1, &AckermannControlAlgNode::cb_getOdomMsg, this);
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_plot", 1, &AckermannControlAlgNode::cb_getPoseMsg, this);
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
  static float speed_prev = 0.0;

  float speed_current;
  float k_sp = (this->v_max_ - this->v_min_) / this->params_.maxAngle;

  if (this->flag_odom_ && this->flag_goal_)
  {
    // [fill msg structures]
    this->direction_ = this->control_->getBestSteering(this->pose_, this->goal_);

    switch (this->direction_.sense)
    {
      case 0:
        speed_current = 0.0;
        break;
      case 1:
        speed_current = this->v_max_ - fabs(this->direction_.angle) * k_sp;
        break;
      case -1:
        speed_current = -1 * (this->v_max_ - fabs(this->direction_.angle) * k_sp);
        break;
    } 
  }
  else
  {
    this->direction_.angle = 0.0;
    speed_current = 0.0;
  }
  
  //proportional filtration
  if (this->flag_prop_)
  {
    speed_current = speed_prev + (speed_current - speed_prev) * this->kp_;
    speed_prev = speed_current;
  }
  
  this->ackermann_state_.drive.steering_angle = (this->direction_.angle*PI)/180.0;
  this->ackermann_state_.drive.speed = speed_current;
  this->twist_state_.linear.x = speed_current;
  this->twist_state_.angular.z = (speed_current / this->params_.l) * sin((this->direction_.angle*PI)/180.0);
  
  // [publish messages]
  this->ackermann_publisher_.publish(this->ackermann_state_.drive);
  this->twist_publisher_.publish(this->twist_state_);

}

/*  [subscriber callbacks] */
void AckermannControlAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  // in base_link frame
  this->pose_.coordinates.at(0) = 0.0;
  this->pose_.coordinates.at(1) = 0.0;
  this->pose_.coordinates.at(2) = 0.0;
  this->pose_.coordinates.at(3) = 0.0;
  this->pose_.matrix[0][0] = pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = pose_msg->pose.covariance[35];
  
  this->flag_odom_ = true;

  this->alg_.unlock();
}
void AckermannControlAlgNode::cb_getOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  // in base_link frame
  this->pose_.coordinates.at(0) = 0.0;
  this->pose_.coordinates.at(1) = 0.0;
  this->pose_.coordinates.at(2) = 0.0;
  this->pose_.coordinates.at(3) = 0.0;
  this->pose_.matrix[0][0] = odom_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = odom_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = odom_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = odom_msg->pose.covariance[35];
  
  this->flag_odom_ = true;

  this->alg_.unlock();
}
void AckermannControlAlgNode::cb_getGoalMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();
  
  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO BASE_LINK FARME
  geometry_msgs::PointStamped goal_tf;
  geometry_msgs::PointStamped goal_base;
  goal_tf.header.frame_id = this->frame_id_;
  goal_tf.header.stamp = ros::Time(0); //ros::Time::now();
  goal_tf.point.x = goal_msg->pose.pose.position.x;
  goal_tf.point.y = goal_msg->pose.pose.position.y;
  goal_tf.point.z = goal_msg->pose.pose.position.z;
  try
  {
    this->listener_.transformPoint("base_link", goal_tf, goal_base);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  
  geometry_msgs::QuaternionStamped orient_tf;
  geometry_msgs::QuaternionStamped orient_base;
  orient_tf.header.frame_id = this->frame_id_;
  orient_tf.header.stamp = ros::Time(0);
  orient_tf.quaternion.x = goal_msg->pose.pose.orientation.x;
  orient_tf.quaternion.y = goal_msg->pose.pose.orientation.y;
  orient_tf.quaternion.z = goal_msg->pose.pose.orientation.z;
  orient_tf.quaternion.w = goal_msg->pose.pose.orientation.w;
  try
  {
    this->listener_.transformQuaternion("base_link", orient_tf, orient_base);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_base.quaternion.x, orient_base.quaternion.y,
                        orient_base.quaternion.z, orient_base.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
  ///////////////////////////////////////////////////////////

  
  double r = sqrt(pow(goal_base.point.x, 2) + pow(goal_base.point.y, 2));
  double yaw_to_pose = asin(goal_base.point.y / r);
  yaw_to_pose = (yaw_to_pose * 180.0) / PI;
  if (goal_base.point.x < 0.0 && goal_base.point.y <= 0.0)
  {
    yaw_to_pose = -180 - yaw_to_pose;
  }
  else if (goal_base.point.x < 0.0 && goal_base.point.y > 0.0)
  {
    yaw_to_pose = 180 - yaw_to_pose;
  }
  
  this->goal_.coordinates.at(0) = goal_base.point.x;
  this->goal_.coordinates.at(1) = goal_base.point.y;
  this->goal_.coordinates.at(2) = goal_base.point.z;
  this->goal_.coordinates.at(3) = yaw_to_pose;
  this->goal_.matrix[0][0] = goal_msg->pose.covariance[0];
  this->goal_.matrix[1][1] = goal_msg->pose.covariance[7];
  this->goal_.matrix[2][2] = goal_msg->pose.covariance[14];
  this->goal_.matrix[3][3] = goal_msg->pose.covariance[35];
  
  
  float distance = sqrt(pow(goal_base.point.x, 2) + pow(goal_base.point.y, 2));
  if (distance <= this->rad_reached_) this->flag_goal_ = false;
  else this->flag_goal_ = true;

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
