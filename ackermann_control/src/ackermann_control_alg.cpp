#include "ackermann_control_alg.h"

AckermannControlAlgorithm::AckermannControlAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
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

void AckermannControlAlgorithm::naiveNonObstaclePointsRemover(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const RobotParams robot_params,
    const CollisionAvoidanceParams collision_avoidance_params,
    const AckermannPredictionParams ackermann_prediction_params, pcl::PointCloud<pcl::PointXYZI>& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aux(new pcl::PointCloud<pcl::PointXYZI>);

  float minimum_turning_radius = fabs(robot_params.wheelbase / tan(robot_params.abs_max_steering_angle_deg * M_PI / 180.0)); // fabs not required but anyway

  float abs_max_x_coordinate = minimum_turning_radius + (robot_params.width / 2.0) + collision_avoidance_params.safety_lateral_margin;

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

// AckermannControlAlgorithm Public API
