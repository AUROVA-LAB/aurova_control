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

  // We use the worst case, if the vehicle were
  // omnidirectional it can advance this much
  // either in x or y coordinates
  float abs_max_coordinate = robot_params.max_speed_meters_per_second * ackermann_prediction_params.temporal_horizon;

  // Create the filtering object
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1 * abs_max_coordinate, abs_max_coordinate);
  pass.filter(*aux);

  pass.setInputCloud(aux);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1 * abs_max_coordinate, abs_max_coordinate);
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
