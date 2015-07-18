/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef GLOBAL_HARD_LAYER_H_
#define GLOBAL_HARD_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

namespace costmap_2d
{
class GlobalHardLayer : public CostmapLayer
{
public:
  GlobalHardLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void slamCb(const nav_msgs::OccupancyGridConstPtr& slamMap);
  void visionHardCb(const nav_msgs::OccupancyGridConstPtr& hardPatch);

  uint8_t interpretValue(int8_t value);

  bool alignWithNewMap(const nav_msgs::OccupancyGridConstPtr& in,
      const nav_msgs::OccupancyGridPtr& out);
  void bufferUpdate(const nav_msgs::OccupancyGridPtr& buffer,
      const nav_msgs::OccupancyGridConstPtr& patch);
  void mapDilation(const nav_msgs::OccupancyGridPtr& in,
      int steps, int coords,
      nav_msgs::OccupancyGridConstPtr checkMap = nav_msgs::OccupancyGridPtr());

protected:
  nav_msgs::OccupancyGridPtr bufferCostmap_;
  int unknown_cost_value_;
  int mitsos_cost_value_;
private:
  ros::Subscriber slam_map_sub_;
  std::string slam_map_topic_;
  bool map_received_;

  ros::Subscriber vision_hard_sub_;
  std::string vision_hard_topic_;
  bool has_updated_data_;

  std::string global_frame_; ///< @brief The global frame for the costmap
};
}
#endif
