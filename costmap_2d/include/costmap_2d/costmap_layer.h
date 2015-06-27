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
#ifndef COSTMAP_LAYER_H_
#define COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{
class CostmapLayer : public Layer, public Costmap2D
{
public:
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

protected:
  /**
   * Updates the master_grid within the specified bounding box using this layer's values.
   *
   * True Overwrite means every value from this layer is written into the master grid.
   */
  void updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  

  /**
   * Updates the master_grid within the specified bounding box using this layer's values.
   *
   * Overwrite means every valid value from this layer is written into the master grid
   * (does not copy NO_INFORMATION).
   */
  void updateWithOverwrite    (costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  

  /**
   * Updates the master_grid within the specified bounding box using this layer's values.
   *
   * Sets the new value to the maximum of the master's grid value and this layer's value.
   * If the master value is NO_INFORMATION, it is overwritten. If the layer's value is 
   * NO_INFORMATION, the master's value doesn't change.
   */
  void updateWithMax          (costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  

  /**
   * Updates the master_grid within the specified bounding box using this layer's values.
   *
   * Sets the new value to the sum of the master grid's value and this layer's value.
   * If the master value is NO_INFORMATION, it is overwritten with the layer's value.
   * If the layer's value is NO_INFORMATION, then the master value does not change.
   *
   * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE, the master value is set to
   * (INSCRIBED_INFLATED_OBSTACLE - 1)
   * 
   * Not used in any of static, obstacle, coverage, map layers
   */
  void updateWithAddition     (costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);
};
}
#endif

