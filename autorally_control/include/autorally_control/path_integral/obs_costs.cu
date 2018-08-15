/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file obs_costs.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date July 6, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief MPPIObsCosts class implementation
 ***********************************************/
#include "gpu_err_chk.h"
#include "debug_kernels.cuh"

#include <stdio.h>
#include <stdlib.h>

namespace autorally_control {

inline MPPIObstacleCosts::MPPIObstacleCosts(int width, int height)
        : MPPICosts(width, height)
{
  obs_width_ = width;
  obs_height_ = height;
  allocateObstacleTexMem();
  //Initialize memory for device obstacle cost param struct
  HANDLE_ERROR( cudaMalloc((void**)&obs_params_d_, sizeof(ObstacleParams)) );

  callback_f_ = boost::bind(&MPPIObstacleCosts::updateParams_dcfg, this, _1, _2);
  server_.setCallback(callback_f_);
}

inline MPPIObstacleCosts::MPPIObstacleCosts(ros::NodeHandle mppi_node)
        : MPPICosts(mppi_node)
{
  //Transform from world coordinates to normalized grid coordinates
  Eigen::Matrix3f R;
  Eigen::Array3f trs;
  //Initialize memory for device obstacle cost param struct
  HANDLE_ERROR( cudaMalloc((void**)&obs_params_d_, sizeof(ObstacleParams)) );

  //Get the obstacle map path
  std::string obstacle_path;
  mppi_node.getParam("obstacle_path", obstacle_path);
  std::vector<float> obstacle_costs = loadObstacleData(obstacle_path.c_str(), R, trs);
  updateObstacleTransform(R, trs);
  updateObstacleParams(mppi_node);
  allocateObstacleTexMem();
  obsToTexture(obstacle_costs.data());

  callback_f_ = boost::bind(&MPPIObstacleCosts::updateParams_dcfg, this, _1, _2);
  server_.setCallback(callback_f_);
}

inline MPPIObstacleCosts::~MPPIObstacleCosts()
{}

inline void MPPIObstacleCosts::allocateObstacleTexMem()
{
  //Allocate memory for the cuda array which is bound the obs_costs_tex_
  obs_channel_desc_ = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
  HANDLE_ERROR(cudaMallocArray(&obs_costs_d_, &obs_channel_desc_, obs_width_, obs_height_));
}

inline void MPPIObstacleCosts::obsToTexture(float* obs_costs)
{
  // Transfer CPU memory to GPU
  HANDLE_ERROR( cudaMemcpyToArray(obs_costs_d_, 0, 0, obs_costs, obs_width_*obs_height_*sizeof(float), cudaMemcpyHostToDevice) );

  // Specify texture
  struct cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = obs_costs_d_;

  // Specify texture object parameters
  struct cudaTextureDesc texDesc;
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.addressMode[0] = cudaAddressModeClamp;
  texDesc.addressMode[1] = cudaAddressModeClamp;
  texDesc.filterMode = cudaFilterModeLinear;
  texDesc.readMode = cudaReadModeElementType;
  texDesc.normalizedCoords = 1;

  // First destroy the current texture object
  HANDLE_ERROR(cudaDestroyTextureObject(obs_costs_tex_));

  // Now create the new texture object.
  HANDLE_ERROR(cudaCreateTextureObject(&obs_costs_tex_, &resDesc, &texDesc, NULL) );
}

inline void MPPIObstacleCosts::updateObstacleTransform(Eigen::MatrixXf m, Eigen::ArrayXf trs)
{
  obs_params_.r_c1.x = m(0,0);
  obs_params_.r_c1.y = m(1,0);
  obs_params_.r_c1.z = m(2,0);
  obs_params_.r_c2.x = m(0,1);
  obs_params_.r_c2.y = m(1,1);
  obs_params_.r_c2.z = m(2,1);
  obs_params_.trs.x = trs(0);
  obs_params_.trs.y = trs(1);
  obs_params_.trs.z = trs(2);
  //Move the updated parameters to gpu memory
  obstacleParamsToDevice();
}

inline void MPPIObstacleCosts::obstacleParamsToDevice()
{
  HANDLE_ERROR( cudaMemcpy(obs_params_d_, &obs_params_, sizeof(ObstacleParams), cudaMemcpyHostToDevice) )
}

inline void MPPIObstacleCosts::updateParams_dcfg(autorally_control::PathIntegralParamsConfig &config, int lvl)
{
  MPPICosts:updateParams_dcfg(config, lvl);
  obs_params_.static_obs_coeff = (float) config.static_obstacle_coefficient;
  obstacleParamsToDevice();
}

inline void MPPIObstacleCosts::updateObstacleParams(ros::NodeHandle mppi_node)
{
  double static_obs_coeff;
  // Read the parameters from the ROS parameter server
  mppi_node.getParam("static_obstacle_coefficient", static_obs_coeff);

  // Transfer to the obstacle params struct
  obs_params_.static_obs_coeff = (float) static_obs_coeff;
  // Move the updated parameters to the GPU memory
  obstacleParamsToDevice();
}

inline std::vector<float> MPPIObstacleCosts::loadObstacleData(const char* obstacle_path, Eigen::Matrix3f &R, Eigen::Array3f &trs)
{
  int i;
  float p;
  FILE *obstacle_data_file;
  char file_path[256];
  file_path[0] = 0;
  strcat(file_path, obstacle_path);
  strcat(file_path, "obstacle_data.txt");
  obstacle_data_file = fopen(file_path, "r");
  if (obstacle_data_file == NULL) {
    ROS_INFO("Error opening obstacle data file: No such file or directory: %s \n", file_path);
    ros::shutdown();
  }
  //Read the parameters from the file
  float x_min, x_max, y_min, y_max, resolution;
  bool success = true;
  success = success && fscanf(obstacle_data_file, "%f", &x_min);
  success = success && fscanf(obstacle_data_file, "%f", &x_max);
  success = success && fscanf(obstacle_data_file, "%f", &y_min);
  success = success && fscanf(obstacle_data_file, "%f", &y_max);
  success = success && fscanf(obstacle_data_file, "%f", &resolution);
  //Save width_ and height_ parameters
  obs_width_ = int((x_max - x_min)*resolution);
  obs_height_ = int((y_max - y_min)*resolution);
  std::vector<float> obstacle_costs(obs_width_*obs_height_);
  //Scan the result of the file to load track parameters
  for (i = 0; i < width_*height_; i++) {
    success = success && fscanf(obstacle_data_file, "%f", &p);
    obstacle_costs[i] = p;
  }
  if (!success){
    ROS_INFO("Warning track parameters not read successfully.");
  }
  //Save the scaling and offset
  R << 1./(x_max - x_min), 0,                  0,
       0,                  1./(y_max - y_min), 0,
       0,                  0,                  1;
  trs << -x_min/(x_max - x_min), -y_min/(y_max - y_min), 1;
  fclose(obstacle_data_file);
  return obstacle_costs;
}

inline void MPPIObstacleCosts::freeCudaMem()
{
  MPPICosts::freeCudaMem();
  HANDLE_ERROR( cudaFree(obs_params_d_) );
}

inline __host__ __device__ void MPPIObstacleCosts::obsCoorTransform(float x, float y, float* u, float* v, float* w)
{
  //Compute a projective transform of (x, y, 0, 1)
  u[0] = obs_params_d_->r_c1.x*x + obs_params_d_->r_c2.x*y + obs_params_d_->trs.x;
  v[0] = obs_params_d_->r_c1.y*x + obs_params_d_->r_c2.y*y + obs_params_d_->trs.y;
  w[0] = obs_params_d_->r_c1.z*x + obs_params_d_->r_c2.z*y + obs_params_d_->trs.z;
}

inline __device__ float MPPIObstacleCosts::getObstacleCost(float* s)
{
  float obstacle_cost = 0;

  //Compute a transformation to get the (x,y) positions of the front and back of the car.
  float x_front = s[0] + FRONT_D*__cosf(s[2]);
  float y_front = s[1] + FRONT_D*__sinf(s[2]);
  float x_back = s[0] + BACK_D*__cosf(s[2]);
  float y_back = s[1] + BACK_D*__sinf(s[2]);

  float u,v,w; //Transformed coordinates

  //Cost of front of the car
  obsCoorTransform(x_front, y_front, &u, &v, &w);
  float obs_cost_front = tex2D<float>(obs_costs_tex_, u/w, v/w);

  //Cost for back of the car
  obsCoorTransform(x_back, y_back, &u, &v, &w);
  float obs_cost_back = tex2D<float>(obs_costs_tex_, u/w, v/w);

  obstacle_cost = (fabs(obs_cost_front) + fabs(obs_cost_back) )/2.0;
  if (fabs(obstacle_cost) < params_d_->track_slop) {
    obstacle_cost = 0;
  }
  else {
    obstacle_cost = obs_params_d_->static_obs_coeff*obstacle_cost;
  }

  return obstacle_cost;
}

inline __device__ float MPPIObstacleCosts::computeCost(float* s, float* u, float* du, float* vars, int* crash, int timestep)
{
  float cost = MPPICosts::computeCost(s, u, du, vars, crash, timestep);
  float obstacle_cost = getObstacleCost(s);
  cost += obstacle_cost;
  if (cost > 1e9 || isnan(cost)) {
    cost = 1e9;
  }
  return cost;
}

}