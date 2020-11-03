#include "ackermann_control_alg_node.h"

AckermannControlAlgNode::AckermannControlAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannControlAlgorithm>()
{
  //init class attributes if necessary
  loop_rate_ = 10; //in [Hz]
  velodyne_pcl_cloud_ptr_ = pcl::PointCloud < pcl::PointXYZI > ::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  //get application parameters
  int vectors_size = 4;
  flag_odom_ = false;
  flag_goal_ = false;
  flag_velodyne_ = false;

  public_node_handle_.getParam("/ackermann_control/frame_id", frame_id_);
  public_node_handle_.getParam("/ackermann_control/control_in_map_frame", control_in_map_frame_);

  public_node_handle_.getParam("/ackermann_control/robot_x_distance_from_velodyne_to_base_link",
                               robot_params_.x_distance_from_velodyne_to_base_link);
  public_node_handle_.getParam("/ackermann_control/robot_x_distance_from_velodyne_to_front",
                               robot_params_.x_distance_from_velodyne_to_front);
  public_node_handle_.getParam("/ackermann_control/robot_x_distance_from_velodyne_to_back",
                               robot_params_.x_distance_from_velodyne_to_back);

  public_node_handle_.getParam("/ackermann_control/robot_abs_max_steering_angle_deg",
                               robot_params_.abs_max_steering_angle_deg);
  public_node_handle_.getParam("/ackermann_control/robot_wheelbase", robot_params_.wheelbase);

  public_node_handle_.getParam("/ackermann_control/robot_width", robot_params_.width);
  public_node_handle_.getParam("/ackermann_control/robot_length", robot_params_.length);
  public_node_handle_.getParam("/ackermann_control/robot_height", robot_params_.height);

  public_node_handle_.getParam("/ackermann_control/robot_ground_z_coordinate_in_sensor_frame_origin",
                               robot_params_.ground_z_coordinate_in_sensor_frame_origin);

  public_node_handle_.getParam("/ackermann_control/robot_max_speed_meters_per_second",
                               robot_params_.max_speed_meters_per_second);

  public_node_handle_.getParam("/ackermann_control/control_max_speed_meters_per_second",
                               ackermann_control_params_.max_speed_meters_per_second);
  public_node_handle_.getParam("/ackermann_control/control_min_speed_meters_per_second",
                               ackermann_control_params_.min_speed_meters_per_second);

  public_node_handle_.getParam("/ackermann_control/safety_lateral_margin",
                               collision_avoidance_params_.safety_lateral_margin);
  public_node_handle_.getParam("/ackermann_control/safety_above_margin",
                               collision_avoidance_params_.safety_above_margin);
  public_node_handle_.getParam("/ackermann_control/safety_longitudinal_margin",
                               collision_avoidance_params_.safety_longitudinal_margin);
  public_node_handle_.getParam("/ackermann_control/safety_min_obstacle_height",
                               collision_avoidance_params_.min_obstacle_height);

  public_node_handle_.getParam("/ackermann_control/prediction_temporal_horizon",
                               ackermann_prediction_params_.temporal_horizon);
  public_node_handle_.getParam("/ackermann_control/prediction_delta_time", ackermann_prediction_params_.delta_time);
  public_node_handle_.getParam("/ackermann_control/prediction_delta_steering_deg",
                               ackermann_prediction_params_.delta_steering);

  // class constructor for control
  control_ = new SteeringControl(robot_params_, ackermann_prediction_params_, ackermann_control_params_,
                                 collision_avoidance_params_);

  // inicializations of poses
  pose_.coordinates.resize(vectors_size);
  pose_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    pose_.matrix[i].resize(vectors_size);

  goal_.coordinates.resize(vectors_size);
  goal_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    goal_.matrix[i].resize(vectors_size);

  // [init publishers]
  ackermann_publisher_ = public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);
  twist_publisher_ = public_node_handle_.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);

  filtered_velodyne_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2
      > ("/velodyne_close_obstacle_points", 1);

  // [init subscribers]
  odom_subscriber_ = public_node_handle_.subscribe("/odom", 1, &AckermannControlAlgNode::cb_getOdomMsg, this);
  pose_subscriber_ = public_node_handle_.subscribe("/pose_sim", 1, &AckermannControlAlgNode::cb_getPoseMsg, this);
  goal_subscriber_ = public_node_handle_.subscribe("/semilocal_goal", 1, &AckermannControlAlgNode::cb_getGoalMsg, this);

  velodyne_subscriber_ = public_node_handle_.subscribe("/velodyne_points", 1, &AckermannControlAlgNode::cb_velodyne,
                                                       this);

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

  if (flag_odom_ && flag_goal_ && flag_velodyne_)
  {
    velodyne_mutex_enter();
    //std::cout << "Reset flags!" << std::endl;
    flag_odom_ = false;
    flag_goal_ = false;
    flag_velodyne_ = false;

    // [fill msg structures]
    float k_sp = (ackermann_control_params_.max_speed_meters_per_second
        - ackermann_control_params_.min_speed_meters_per_second) / robot_params_.abs_max_steering_angle_deg;

    //std::cout << "Getting best steering!" << std::endl;
    direction_ = control_->getBestSteering(this->pose_, this->goal_, velodyne_pcl_cloud_ptr_);

    double speed = 0.0;
    switch (this->direction_.sense)
    {
      case 0:
        speed = 0.0;
        break;
      case 1:
        speed = ackermann_control_params_.max_speed_meters_per_second - fabs(direction_.angle) * k_sp;
        break;
      case -1:
        speed = -1 * (ackermann_control_params_.max_speed_meters_per_second - fabs(this->direction_.angle) * k_sp);
        break;
    }

    //std::cout << "Preparing outputs!" << std::endl;
    ackermann_state_.drive.steering_angle = direction_.angle;
    ackermann_state_.drive.speed = speed;

    twist_state_.linear.x = speed;
    twist_state_.angular.z = (speed / robot_params_.wheelbase) * sin(direction_.angle * M_PI / 180.0);

    //////////////////////////////////////////////////////
    //// DEBUG
    /*
     ROS_INFO("goal -> x: %f, y: %f, z: %f, yaw: %f", this->goal_.coordinates[0], this->goal_.coordinates[1],
     this->goal_.coordinates[2], this->goal_.coordinates[3]);
     ROS_INFO("control -> steering: %f, speed: %f", this->direction_.angle, speed);
     ROS_INFO("control -> angular: %f, linear: %f", this->twist_state_.angular.z, speed);
     */
    //////////////////////////////////////////////////////
    // [fill srv structure and make request to the server]
    // [fill action structure and make request to the action server]
    // [publish messages]
    //std::cout << "Publishing messages!!" << std::endl;
    ackermann_publisher_.publish(ackermann_state_);
    twist_publisher_.publish(twist_state_);
    filtered_velodyne_publisher_.publish(velodyne_ros_cloud_);
    velodyne_pcl_cloud_ptr_->clear(); // Cleaning up to prepare next iteration
    velodyne_mutex_exit();
    flag_velodyne_ = false;
  }
  else
  {
    if (flag_velodyne_)
    {
      //std::cout << "Publishing only velodyne message!!" << std::endl;
      velodyne_mutex_enter();
      filtered_velodyne_publisher_.publish(velodyne_ros_cloud_);
      velodyne_pcl_cloud_ptr_->clear(); // Cleaning up to prepare next iteration
      velodyne_mutex_exit();
      flag_velodyne_ = false;
    }
  }
}

/*  [subscriber callbacks] */
void AckermannControlAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  //std::cout << "AckermannControlAlgNode::cb_getPoseMsg --> pose_msg received!" << std::endl;
  alg_.lock();

  if (control_in_map_frame_)
  {
    pose_.coordinates.at(0) = pose_msg->pose.pose.position.x;
    pose_.coordinates.at(1) = pose_msg->pose.pose.position.y;
    pose_.coordinates.at(2) = pose_msg->pose.pose.position.z;

    double roll, pitch, yaw_rad, yaw_deg;
    tf::Quaternion q_pose(pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
                          pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m_pose(q_pose);
    m_pose.getRPY(roll, pitch, yaw_rad);
    yaw_deg = yaw_rad * 180.0 / M_PI;

    pose_.coordinates.at(3) = yaw_deg;

    pose_.matrix[0][0] = pose_msg->pose.covariance[0]; //x    variance
    pose_.matrix[0][1] = pose_msg->pose.covariance[1]; //xy   covariance
    pose_.matrix[0][2] = pose_msg->pose.covariance[2]; //xz   covariance
    pose_.matrix[0][3] = pose_msg->pose.covariance[5]; //xyaw covariance

    pose_.matrix[1][0] = pose_msg->pose.covariance[6]; //yx   covariance
    pose_.matrix[1][1] = pose_msg->pose.covariance[7]; //y    variance
    pose_.matrix[1][2] = pose_msg->pose.covariance[8]; //yz   covariance
    pose_.matrix[1][3] = pose_msg->pose.covariance[11]; //yyaw covariance

    pose_.matrix[2][0] = pose_msg->pose.covariance[12]; //zx   covariance
    pose_.matrix[2][1] = pose_msg->pose.covariance[13]; //zy   covariance
    pose_.matrix[2][2] = pose_msg->pose.covariance[14]; //z    variance
    pose_.matrix[2][3] = pose_msg->pose.covariance[17]; //zyaw variance

    pose_.matrix[3][0] = pose_msg->pose.covariance[30]; //yawx covariance
    pose_.matrix[3][1] = pose_msg->pose.covariance[31]; //yawy covariance
    pose_.matrix[3][3] = pose_msg->pose.covariance[32]; //yawz covariance
    pose_.matrix[3][3] = pose_msg->pose.covariance[35]; //yaw  variance
  }
  else
  {
    // in base_link frame
    pose_.coordinates.at(0) = 0.0;
    pose_.coordinates.at(1) = 0.0;
    pose_.coordinates.at(2) = 0.0;
    pose_.coordinates.at(3) = 0.0;

    pose_.matrix[0][0] = pose_msg->pose.covariance[0];
    pose_.matrix[1][1] = pose_msg->pose.covariance[7];
    pose_.matrix[2][2] = pose_msg->pose.covariance[14];
    pose_.matrix[3][3] = pose_msg->pose.covariance[35];
  }

  flag_odom_ = true;

  alg_.unlock();
}
void AckermannControlAlgNode::cb_getOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  //std::cout << "AckermannControlAlgNode::cb_getOdomMsg --> odom_msg received!" << std::endl;
  alg_.lock();

  if (control_in_map_frame_)
  {
    pose_.coordinates.at(0) = odom_msg->pose.pose.position.x;
    pose_.coordinates.at(1) = odom_msg->pose.pose.position.y;
    pose_.coordinates.at(2) = odom_msg->pose.pose.position.z;

    double roll, pitch, yaw_rad, yaw_deg;
    tf::Quaternion q_pose(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                          odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m_pose(q_pose);
    m_pose.getRPY(roll, pitch, yaw_rad);
    yaw_deg = yaw_rad * 180.0 / M_PI;

    pose_.coordinates.at(3) = yaw_deg;

    pose_.matrix[0][0] = odom_msg->pose.covariance[0]; //x    variance
    pose_.matrix[0][1] = odom_msg->pose.covariance[1]; //xy   covariance
    pose_.matrix[0][2] = odom_msg->pose.covariance[2]; //xz   covariance
    pose_.matrix[0][3] = odom_msg->pose.covariance[5]; //xyaw covariance

    pose_.matrix[1][0] = odom_msg->pose.covariance[6]; //yx   covariance
    pose_.matrix[1][1] = odom_msg->pose.covariance[7]; //y    variance
    pose_.matrix[1][2] = odom_msg->pose.covariance[8]; //yz   covariance
    pose_.matrix[1][3] = odom_msg->pose.covariance[11]; //yyaw covariance

    pose_.matrix[2][0] = odom_msg->pose.covariance[12]; //zx   covariance
    pose_.matrix[2][1] = odom_msg->pose.covariance[13]; //zy   covariance
    pose_.matrix[2][2] = odom_msg->pose.covariance[14]; //z    variance
    pose_.matrix[2][3] = odom_msg->pose.covariance[17]; //zyaw variance

    pose_.matrix[3][0] = odom_msg->pose.covariance[30]; //yawx covariance
    pose_.matrix[3][1] = odom_msg->pose.covariance[31]; //yawy covariance
    pose_.matrix[3][3] = odom_msg->pose.covariance[32]; //yawz covariance
    pose_.matrix[3][3] = odom_msg->pose.covariance[35]; //yaw  variance
  }
  else
  {
    // in base_link frame
    pose_.coordinates.at(0) = 0.0;
    pose_.coordinates.at(1) = 0.0;
    pose_.coordinates.at(2) = 0.0;
    pose_.coordinates.at(3) = 0.0;

    pose_.matrix[0][0] = odom_msg->pose.covariance[0];
    pose_.matrix[1][1] = odom_msg->pose.covariance[7];
    pose_.matrix[2][2] = odom_msg->pose.covariance[14];
    pose_.matrix[3][3] = odom_msg->pose.covariance[35];
  }

  flag_odom_ = true;

  alg_.unlock();
}
void AckermannControlAlgNode::cb_getGoalMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal_msg)
{
  //std::cout << "AckermannControlAlgNode::cb_getGoalMsg --> goal_msg received!" << std::endl;
  alg_.lock();

  if (control_in_map_frame_)
  {
    goal_.coordinates.at(0) = goal_msg->pose.pose.position.x;
    goal_.coordinates.at(1) = goal_msg->pose.pose.position.y;
    goal_.coordinates.at(2) = goal_msg->pose.pose.position.z;

    double roll, pitch, yaw_rad, yaw_deg;
    tf::Quaternion q_pose(goal_msg->pose.pose.orientation.x, goal_msg->pose.pose.orientation.y,
                          goal_msg->pose.pose.orientation.z, goal_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m_pose(q_pose);
    m_pose.getRPY(roll, pitch, yaw_rad);
    yaw_deg = yaw_rad * 180.0 / M_PI;

    goal_.coordinates.at(3) = yaw_deg;

    goal_.matrix[0][0] = goal_msg->pose.covariance[0]; //x    variance
    goal_.matrix[0][1] = goal_msg->pose.covariance[1]; //xy   covariance
    goal_.matrix[0][2] = goal_msg->pose.covariance[2]; //xz   covariance
    goal_.matrix[0][3] = goal_msg->pose.covariance[5]; //xyaw covariance

    goal_.matrix[1][0] = goal_msg->pose.covariance[6]; //yx   covariance
    goal_.matrix[1][1] = goal_msg->pose.covariance[7]; //y    variance
    goal_.matrix[1][2] = goal_msg->pose.covariance[8]; //yz   covariance
    goal_.matrix[1][3] = goal_msg->pose.covariance[11]; //yyaw covariance

    goal_.matrix[2][0] = goal_msg->pose.covariance[12]; //zx   covariance
    goal_.matrix[2][1] = goal_msg->pose.covariance[13]; //zy   covariance
    goal_.matrix[2][2] = goal_msg->pose.covariance[14]; //z    variance
    goal_.matrix[2][3] = goal_msg->pose.covariance[17]; //zyaw variance

    goal_.matrix[3][0] = goal_msg->pose.covariance[30]; //yawx covariance
    goal_.matrix[3][1] = goal_msg->pose.covariance[31]; //yawy covariance
    goal_.matrix[3][3] = goal_msg->pose.covariance[32]; //yawz covariance
    goal_.matrix[3][3] = goal_msg->pose.covariance[35]; //yaw  variance
  }
  else
  {
    /*
    ///////////////////////////////////////////////////////////
    ///// TRANSFORM TO BASE_LINK FRAME
    geometry_msgs::PointStamped goal_tf;
    geometry_msgs::PointStamped goal_base;
    goal_tf.header.frame_id = frame_id_;
    goal_tf.header.stamp = ros::Time(0); //ros::Time::now();
    goal_tf.point.x = goal_msg->pose.pose.position.x;
    goal_tf.point.y = goal_msg->pose.pose.position.y;
    goal_tf.point.z = goal_msg->pose.pose.position.z;
    try
    {
      listener_.transformPoint("base_link", goal_tf, goal_base);

    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return;
    }

    geometry_msgs::QuaternionStamped orient_tf;
    geometry_msgs::QuaternionStamped orient_base;
    orient_tf.header.frame_id = frame_id_;
    orient_tf.header.stamp = ros::Time(0);
    orient_tf.quaternion.x = goal_msg->pose.pose.orientation.x;
    orient_tf.quaternion.y = goal_msg->pose.pose.orientation.y;
    orient_tf.quaternion.z = goal_msg->pose.pose.orientation.z;
    orient_tf.quaternion.w = goal_msg->pose.pose.orientation.w;
    try
    {
      listener_.transformQuaternion("base_link", orient_tf, orient_base);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return;
    }
    double roll, pitch, yaw;
    tf::Quaternion q_pose(orient_base.quaternion.x, orient_base.quaternion.y, orient_base.quaternion.z,
                          orient_base.quaternion.w);
    tf::Matrix3x3 m_pose(q_pose);
    m_pose.getRPY(roll, pitch, yaw);
    yaw = (yaw * 180.0) / M_PI;
    ///////////////////////////////////////////////////////////

    double r = sqrt(pow(goal_base.point.x, 2) + pow(goal_base.point.y, 2));
    double yaw_to_pose = asin(goal_base.point.y / r);
    yaw_to_pose = (yaw_to_pose * 180.0) / M_PI;
    if (goal_base.point.x < 0.0 && goal_base.point.y <= 0.0)
    {
      yaw_to_pose = -180 - yaw_to_pose;
    }
    else if (goal_base.point.x < 0.0 && goal_base.point.y > 0.0)
    {
      yaw_to_pose = 180 - yaw_to_pose;
    }

    goal_.coordinates.at(0) = goal_base.point.x;
    goal_.coordinates.at(1) = goal_base.point.y;
    goal_.coordinates.at(2) = goal_base.point.z;
    goal_.coordinates.at(3) = yaw_to_pose;
    goal_.matrix[0][0] = goal_msg->pose.covariance[0];
    goal_.matrix[1][1] = goal_msg->pose.covariance[7];
    goal_.matrix[2][2] = goal_msg->pose.covariance[14];
    goal_.matrix[3][3] = goal_msg->pose.covariance[35];
    */
  }

  flag_goal_ = true;

  alg_.unlock();
}

void AckermannControlAlgNode::cb_velodyne(const sensor_msgs::PointCloud2::ConstPtr& velodyne_msg)
{
  //std::cout << "AckermannControlAlgNode::cb_velodyne --> Velodyne msg received!" << std::endl;
  assert(velodyne_msg != NULL && "Null pointer!!! in function cb_velodyne!");

  // We convert the input message to pcl pointcloud
  pcl::PCLPointCloud2 aux;
  pcl_conversions::toPCL(*velodyne_msg, aux);
  pcl::fromPCLPointCloud2(aux, *velodyne_pcl_cloud_ptr_);

  // Then we remove the non-obstacle points
  alg_.naiveNonObstaclePointsRemover(velodyne_pcl_cloud_ptr_, robot_params_, collision_avoidance_params_,
                                     ackermann_prediction_params_, *velodyne_pcl_cloud_ptr_);

  // And pass to the output to visualize the filtering
  toPCLPointCloud2(*velodyne_pcl_cloud_ptr_, aux);
  pcl_conversions::fromPCL(aux, velodyne_ros_cloud_);

  flag_velodyne_ = true;
}

void AckermannControlAlgNode::velodyne_mutex_enter(void)
{
  pthread_mutex_lock(&this->velodyne_mutex_);
}

void AckermannControlAlgNode::velodyne_mutex_exit(void)
{
  pthread_mutex_unlock(&this->velodyne_mutex_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AckermannControlAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_.lock();
  config_ = config;
  alg_.unlock();
}

void AckermannControlAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < AckermannControlAlgNode > (argc, argv, "ackermann_control_alg_node");
}
