#include "ackermann_control_alg.h"

AckermannControlAlgorithm::AckermannControlAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
  const int STOP = 0;
  previous_sense_ = STOP;
  previous_speed_ = 0.0;
}

AckermannControlAlgorithm::~AckermannControlAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AckermannControlAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// AckermannControlAlgorithm Public API

void AckermannControlAlgorithm::naiveNonObstaclePointsRemover(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const RobotParams robot_params,
    const CollisionAvoidanceParams collision_avoidance_params,
    const AckermannPredictionParams ackermann_prediction_params, pcl::PointCloud<pcl::PointXYZI>& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aux(new pcl::PointCloud<pcl::PointXYZI>);

  float minimum_turning_radius = fabs(
      robot_params.wheelbase / tan(robot_params.abs_max_steering_angle_deg * M_PI / 180.0)); // fabs not required but anyway

  float abs_max_x_coordinate = minimum_turning_radius + (robot_params.width / 2.0)
      + collision_avoidance_params.safety_lateral_margin;

  float abs_max_y_coordinate = minimum_turning_radius;

  // Create the filtering object
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1.0 * abs_max_x_coordinate, abs_max_x_coordinate);
  pass.filter(*aux);

  pass.setInputCloud(aux);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0 * abs_max_y_coordinate, abs_max_y_coordinate);
  pass.filter(*aux);

  float min_z_coordinate_for_an_obstacle = robot_params.ground_z_coordinate_in_sensor_frame_origin
      + collision_avoidance_params.min_obstacle_height;

  float max_z_coordinate_for_an_obstacle = collision_avoidance_params.safety_above_margin; //Because we are in velodyne frame and it is on top

  pass.setInputCloud(aux);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z_coordinate_for_an_obstacle, max_z_coordinate_for_an_obstacle);
  pass.filter(output);
}

float AckermannControlAlgorithm::getMaxSpeedAtSteeringAngleInDeg(const float steering_angle,
                                                                 const AckermannControlParams ackermann_control_params,
                                                                 const RobotParams robot_params)
{
  float k_sp = (ackermann_control_params.max_speed_meters_per_second
      - ackermann_control_params.min_speed_meters_per_second) / robot_params.abs_max_steering_angle_deg;

  float max_speed_due_to_steering = ackermann_control_params.max_speed_meters_per_second - fabs(steering_angle) * k_sp;
  return (max_speed_due_to_steering);
}

float AckermannControlAlgorithm::limitSpeedToReachFinalGoal(const float speed, const Pose goal, const Pose pose,
                                                            const AckermannControlParams ackermann_control_params,
                                                            const RobotParams robot_params)
{
  float diff_x = goal.coordinates[0] - pose.coordinates[0];
  float diff_y = goal.coordinates[1] - pose.coordinates[1];
  float distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

  float k_approx = distance / ackermann_control_params.final_goal_approximation_radius;
  if (k_approx > 1.0)
    k_approx = 1.0;

  float limited_speed = speed * k_approx;

  if (limited_speed > 0.0 && limited_speed < robot_params.min_speed_meters_per_second)
    limited_speed = robot_params.min_speed_meters_per_second;

  if (limited_speed < 0.0 && limited_speed > -1.0 * robot_params.min_speed_meters_per_second)
    limited_speed = -1.0 * robot_params.min_speed_meters_per_second;

  return (limited_speed);
}

float AckermannControlAlgorithm::limitAcceleration(const float speed, const int sense, const float max_delta_speed)
{
  std::cout << "Checking if litiming speed is needed!" << std::endl;
  float speed_acceleration_limited = 0.0;

  const int STOP = 0;
  if (sense != STOP)
  {
    if (previous_sense_ != 0 && previous_sense_ != sense) //if there is a sense change we first stop
    {
      std::cout << "Sense change required! --> stopping the robot before to limit acceleration!" << std::endl;
      speed_acceleration_limited = 0.0;
    }
    else
    {
      float speed_with_sign = speed * (float)sense; // 1 means forward, -1 means backwards
      std::cout << "speed_with_sign = " << speed_with_sign << std::endl;
      std::cout << "previous_speed_ = " << previous_speed_ << std::endl;
      std::cout << "previous_sense_ = " << previous_sense_ << std::endl;
      std::cout << "max_delta_speed = " << max_delta_speed << std::endl;

      if (fabs(speed_with_sign - previous_speed_) > max_delta_speed) // if the sense is mantained we limit the speed change
      {
        std::cout << "Limitation needed --> applying delta_speed!" << std::endl;
        if (speed_with_sign - previous_speed_ > 0)
          speed_acceleration_limited = previous_speed_ + max_delta_speed;
        else
          speed_acceleration_limited = previous_speed_ - max_delta_speed;
      }
      else
      {
        std::cout << "Not limitation needed!" << std::endl;
        speed_acceleration_limited = speed_with_sign;
      }
    }
  }
  else
  {
    speed_acceleration_limited = 0.0;
    std::cout << "STOP!!" << std::endl;
  }

  previous_sense_ = sense;
  previous_speed_ = speed_acceleration_limited;

  std::cout << "storing speed_acceleration_limited as previous_speed_ = " << previous_speed_ << std::endl;

  return (speed_acceleration_limited);
}
