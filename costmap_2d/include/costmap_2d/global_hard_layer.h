#ifndef GLOBAL_HARD_LAYER_H_
#define GLOBAL_HARD_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>

namespace costmap_2d
{

class GlobalHardLayer : public CostmapLayer
{
public:
  GlobalHardLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void matchSize();

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  void innerCostmapUpdate(const nav_msgs::OccupancyGridConstPtr& new_map);
  //unsigned char interpretValueInd(unsigned char value, unsigned int indx);
  unsigned char interpretValue(unsigned char value);

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  std::string global_frame_; ///< @brief The global frame for the costmap
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_,y_,width_,height_;
  bool trinary_costmap_;
  ros::Subscriber map_sub_;
  unsigned char lethal_threshold_, unknown_cost_value_;

  mutable boost::recursive_mutex lock_;

};
}  // namespace costmap_2d
#endif  // GLOBAL_PATCH_LAYER_H_
