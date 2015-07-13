#include <costmap_2d/global_hard_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::GlobalHardLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

GlobalHardLayer::GlobalHardLayer() : dsrv_(NULL) {}

void GlobalHardLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));


  nh.param("track_unknown_space", track_unknown_space_, true);


  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));


  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &GlobalHardLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GlobalHardLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void GlobalHardLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void GlobalHardLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

/*
* @brief Interpret a cell cost to the corresponding typedef
*
* If tracking unknown space is enabled and the value of the cell grid is equal to the unknown cost we
* set through the "unknown_cost_value" parameter, the function returns NO_INFORMATION.
* If the value of the cell grid is larger or equal to the "lethal_cost_threshold" parameter we set,
* the function returns LETHAL_OBSTACLE.
* If we have set the "trinary_costmap" parameter to true the function returns FREE_SPACE.
* If "trinary_costmap" is false the function returns a scaled cost.
*/
unsigned char GlobalHardLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else
    return FREE_SPACE;

}

/*
 * @ brief Callback to the map subscribed topic
 * @ param new_map The new map to create the static layer from
 *
 * Resize the map layer to correspond to the new map we get and then sets the right
 * values to the grid cells.
 */
void GlobalHardLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_INFO("[Static Layer]Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y){
    matchSize();
  }

  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}


void GlobalHardLayer::activate()
{
    onInitialize();
}

void GlobalHardLayer::deactivate()
{
    map_sub_.shutdown();
}

void GlobalHardLayer::reset()
{
    deactivate();
    activate();
}

void GlobalHardLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  if (!map_received_ || !has_updated_data_)
    return;

  double mx, my;
  // convert from map coordinates to world coordinates
  // x_, y_ are the map coordinates mx, my are the world coordinates
  mapToWorld(x_, y_, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);

  mapToWorld(x_ + width_, y_ + height_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);

  has_updated_data_ = false;

}

void GlobalHardLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void GlobalHardLayer::alignWithNewMap(const nav_msgs::OccupancyGridConstPtr& in,
    const nav_msgs::OccupancyGridPtr& out)
{
  int oldSize = out->data.size();
  int newSize = in->data.size();
  int8_t* oldMap = new int8_t[oldSize];
  nav_msgs::MapMetaData oldMetaData;
  if (oldSize != 0 && oldSize != newSize)
  {
    // Copy old coverage map meta data.
    oldMetaData = out->info;
    // Copy old coverage map.
    memcpy(oldMap, &out->data[0], out->data.size());
  }
  // Reset coveredSpace_->and copy map2dPtr_'s metadata.
  out->header = in->header;
  out->info = in->info;
  if (oldSize != newSize)
  {
    ROS_WARN("[SENSOR_COVERAGE_SPACE_CHECKER %d] Resizing space coverage...", __LINE__);
    out->data.resize(newSize, 0);
    // ROS_ASSERT(newSize == out->data.size());

    if (oldSize != 0)
    {
      double yawDiff = tf::getYaw(out->info.origin.orientation) -
        tf::getYaw(oldMetaData.origin.orientation);
      double xDiff = out->info.origin.position.x -
        oldMetaData.origin.position.x;
      double yDiff = out->info.origin.position.y -
        oldMetaData.origin.position.y;

      double x = 0, y = 0, xn = 0, yn = 0;
      for (unsigned int ii = 0; ii < oldMetaData.width; ++ii)
      {
        for (unsigned int jj = 0; jj < oldMetaData.height; ++jj)
        {
          x = ii * oldMetaData.resolution;
          y = jj * oldMetaData.resolution;
          xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
          yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
          int coords = static_cast<int>(round(xn / out->info.resolution) +
                round(yn * out->info.width / out->info.resolution));
          if ((coords > newSize) || (coords < 0))
          {
            ROS_WARN("Error resizing to: %d\nCoords Xn: %f, Yn: %f\n", newSize, xn, yn);
          }
          else
          {
            uint8_t temp = oldMap[ii + jj * oldMetaData.width];
            out->data[coords] = temp;
            GlobalHardLayer::mapDilation(out, 2, coords, in);
          }
        }
      }
    }
  }
  delete[] oldMap;
}

void GlobalHardLayer::mapDilation(const nav_msgs::OccupancyGridPtr& in, int steps, int coords,
            nav_msgs::OccupancyGridConstPtr checkMap)
{
  if (steps == 0)
    return;
  bool check = checkMap.get() != NULL;

  signed char cell = in->data[coords];

  if (cell != 0)  // That's foreground
  {
    // Check for all adjacent
    if (in->data[coords + in->info.width + 1] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width + 1] < 51)
      {
        in->data[coords + in->info.width + 1] = cell;
        GlobalHardLayer::mapDilation(in, steps - 1, coords + in->info.width + 1);
      }
    }
    if (in->data[coords + in->info.width] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width] < 51)
        in->data[coords + in->info.width] = cell;
    }
    if (in->data[coords + in->info.width - 1] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width - 1] < 51)
      {
        in->data[coords + in->info.width - 1] = cell;
        GlobalHardLayer::mapDilation(in, steps - 1, coords + in->info.width - 1);
      }
    }
    if (in->data[coords + 1] == 0)
    {
      if (!check || checkMap->data[coords + 1] < 51)
        in->data[coords + 1] = cell;
    }
    if (in->data[coords - 1] == 0)
    {
      if (!check || checkMap->data[coords - 1] < 51)
        in->data[coords - 1] = cell;
    }
    if (in->data[coords - in->info.width + 1] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width + 1] < 51)
      {
        in->data[coords - in->info.width + 1] = cell;
        GlobalHardLayer::mapDilation(in, steps - 1, coords - in->info.width + 1);
      }
    }
    if (in->data[coords - in->info.width] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width] < 51)
        in->data[coords - in->info.width] = cell;
    }
    if (in->data[coords - in->info.width - 1] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width - 1] < 51)
      {
        in->data[coords - in->info.width - 1] = cell;
        GlobalHardLayer::mapDilation(in, steps - 1, coords - in->info.width - 1);
      }
    }
  }
}


}
