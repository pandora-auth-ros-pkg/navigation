#include <costmap_2d/global_hard_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::GlobalHardLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

GlobalHardLayer::GlobalHardLayer() {}

void GlobalHardLayer::onInitialize()
{
  // TODO
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string slamMapTopic;
  nh.param("slam_topic", slamMapTopic, std::string("/slam/map"));
  bufferCostmap_.reset( new nav_msgs::OccupancyGrid );

  nh.param("track_unknown_space", track_unknown_space_, true);


  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));


  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  slamMapSub_ = g_nh.subscribe(slamMapTopic, 1, &GlobalHardLayer::slamCB, this);
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
uint8_t GlobalHardLayer::interpretValue(int8_t value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value > unknown_cost_value_)
    return LETHAL_OBSTACLE;
  else
    return FREE_SPACE;
}

void GlobalHardLayer::visionHardCb(const nav_msgs::OccupancyGridConstPtr& hardPatch)
{
  bufferUpdate(bufferCostmap_, hardMap);
}

void GlobalHardLayer::bufferUpdate(const nav_msgs::OccupancyGridPtr& buffer,
    const nav_msgs::OccupancyGridConstPtr& patch)
{
  double yawDiff = tf::getYaw(buffer->info.origin.orientation) -
    tf::getYaw(patch->info.origin.orientation);
  double xDiff = buffer->info.origin.position.x -
    patch->info.origin.position.x;
  double yDiff = buffer->info.origin.position.y -
    patch->info.origin.position.y;

  double x = 0, y = 0, xn = 0, yn = 0;
  for (unsigned int ii = 0; ii < patch->info.width; ++ii)
  {
    for (unsigned int jj = 0; jj < patch->info.height; ++jj)
    {
      x = ii * patch->info.resolution;
      y = jj * patch->info.resolution;
      xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
      yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
      int coords = static_cast<int>(round(xn / buffer->info.resolution) +
            round(yn * buffer->info.width / buffer->info.resolution));
      if ((coords > buffer->data.size()) || (coords < 0))
      {
        ROS_WARN("Error resizing to: %d\nCoords Xn: %f, Yn: %f\n", newSize, xn, yn);
      }
      else
      {
        uint8_t temp = patch->data[ii + jj * patch->info.width];
        buffer->data[coords] = temp;
        mapDilation(buffer, 2, coords);
      }
    }
  }
}

void GlobalHardLayer::slamCb(const nav_msgs::OccupancyGridConstPtr& slamMap)
{
  //TODO
  unsigned int slamWidth = slamMap->info.width;
  unsigned int slamHeight = slamMap->info.height;
  double slamOrgX = slamMap->info.origin.position.x;
  double slamOrgY = slamMap->info.origin.position.y;
  double slamRes = slamMap->info.resolution;

  // For the first time we get slam, so that elevationMap wont write in an
  // empty buffer.
  if (!map_received_)
  {
    ROS_INFO("slamCB first slam");
    bufferCostmap_->info.width = slamWidth;
    bufferCostmap_->info.height = slamHeight;
    bufferCostmap_->info.origin.position.x = slamOrgX;
    bufferCostmap_->info.origin.position.y = slamOrgY;
    bufferCostmap_->info.resolution = slamRes;
    bufferCostmap_->data.resize(slamWidth*slamHeight);
    ROS_INFO("slamCB after copying to buffer slam");
    int it = 0;
    for(int i=0; i<=slamWidth; i++)
    {
      for(int j=0; j<=slamHeight; j++)
      {
        bufferCostmap_->data[it] = 51;  // to param
        it++;
      }
    }
    ROS_INFO("slamCB after copying to before true");

    ROS_INFO("slamCB buffer setted correctly");
  }

  if (bufferCostmap_->info.width != slamWidth ||
      bufferCostmap_->info.height != slamHeight ||
      bufferCostmap_->info.origin.position.x != slamOrgX  ||
      bufferCostmap_->info.origin.position.y != slamOrgY ||
      bufferCostmap_->info.resolution != slamRes
  )
  {
    ROS_INFO("slamCB align slam with buffer");
    // Change MapMetaData and values in bufferCostmap
    alignWithNewMap(slamMap, bufferCostmap_);
  }

  innerCostmapUpdate(bufferCostmap_);
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void GlobalHardLayer::activate()
{
  // TODO
  onInitialize();
}

void GlobalHardLayer::deactivate()
{
  slam_map_sub_.shutdown();
  vision_hard_sub_.shutdown();
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

  // TODO bounds are get from most recent vision update
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
            mapDilation(out, 2, coords);
          }
        }
      }
    }
  }
  delete[] oldMap;
}

void GlobalHardLayer::mapDilation(const nav_msgs::OccupancyGridPtr& in,
    int steps, int coords, nav_msgs::OccupancyGridConstPtr checkMap)
{
  if (steps == 0)
    return;
  bool check = checkMap.get() != NULL;

  signed char cell = in->data[coords];

  if (cell > unknown_cost_value_)  // That's foreground
  {
    // Check for all adjacent
    if (in->data[coords + in->info.width + 1] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width + 1] < 51)
      {
        in->data[coords + in->info.width + 1] = cell;
        mapDilation(in, steps - 1, coords + in->info.width + 1);
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
        mapDilation(in, steps - 1, coords + in->info.width - 1);
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
        mapDilation(in, steps - 1, coords - in->info.width + 1);
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
        mapDilation(in, steps - 1, coords - in->info.width - 1);
      }
    }
  }
}

}
