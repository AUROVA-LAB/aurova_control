#include "ackermann_control_alg.h"

AckermannControlAlgorithm::AckermannControlAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

AckermannControlAlgorithm::~AckermannControlAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AckermannControlAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;

  this->unlock();
}

void AckermannControlAlgorithm::naiveNonObstaclePointsRemover(pcl::PointCloud<pcl::PointXYZI>& velodyne_pcl_cloud)
{

}

// AckermannControlAlgorithm Public API
