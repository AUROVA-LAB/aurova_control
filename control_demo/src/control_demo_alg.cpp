#include "control_demo_alg.h"

ControlDemoAlgorithm::ControlDemoAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

ControlDemoAlgorithm::~ControlDemoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ControlDemoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// ControlDemoAlgorithm Public API
