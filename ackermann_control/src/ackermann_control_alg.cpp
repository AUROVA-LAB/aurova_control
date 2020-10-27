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

void AckermannControlAlgorithm::naiveNonObstaclePointsRemover(const pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                                                              pcl::PointCloud<pcl::PointXYZI>& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aux(new pcl::PointCloud<pcl::PointXYZI>);

  //TODO Extract as parameters
  float abs_max_coordinate = 4.0;   //max_vel * time_horizon; // We use the worst case, if the vehicle were
                                    // omnidirectional it can advance this much
                                    // either in x or y coordinates

  float sensor_height = 1.2;
  float min_obstacle_height = 0.25;
  float safety_margin_above_sensor = 0.3;

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

  float min_z_coordinate_for_an_obstacle = -1.0 * sensor_height + min_obstacle_height;
  float max_z_coordinate_for_an_obstacle = safety_margin_above_sensor;

  pass.setInputCloud(aux);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z_coordinate_for_an_obstacle, max_z_coordinate_for_an_obstacle);
  pass.filter(output);
}

// AckermannControlAlgorithm Public API
