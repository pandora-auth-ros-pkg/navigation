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

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  /**
   * @brief
   * @param value
   *
   */
  unsigned char interpretValue(unsigned char value);

  void alignWithNewMap(const nav_msgs::OccupancyGridConstPtr& in,
      const nav_msgs::OccupancyGridPtr& out);

  void mapDilation(const nav_msgs::OccupancyGridPtr& in, int steps, int coords,
              nav_msgs::OccupancyGridConstPtr checkMap = nav_msgs::OccupancyGridPtr());
              
  std::string global_frame_; ///< @brief The global frame for the costmap

  bool map_received_;
  bool has_updated_data_;
  unsigned int x_,y_,width_,height_;
  bool track_unknown_space_;


  ros::Subscriber map_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  mutable boost::recursive_mutex lock_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
