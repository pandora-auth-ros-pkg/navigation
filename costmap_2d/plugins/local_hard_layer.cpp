#include <costmap_2d/local_hard_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::LocalHardLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

LocalHardLayer::LocalHardLayer() {}

void LocalHardLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  enabled_ = true;

  setDefaultValue(NO_INFORMATION);
  nh.param<int>("unknown_cost_value", unknown_cost_value_, 51);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  if (!nh.getParam("slam_topic", slam_map_topic_))
  {
    ROS_FATAL("[%s] Cound not find slam topic!", name_.c_str());
    ROS_BREAK();
  }
  //slam_map_sub_ = g_nh.subscribe(slam_map_topic_, 1, &LocalHardLayer::slamCb, this);

  if (!nh.getParam("vision_hard_topic", vision_hard_topic_))
  {
    ROS_FATAL("[%s] Cound not find vision hard topic!", name_.c_str());
    ROS_BREAK();
  }
  vision_hard_sub_ = g_nh.subscribe(vision_hard_topic_, 1, &LocalHardLayer::visionHardCb, this);

  bufferCostmap_.reset( new nav_msgs::OccupancyGrid );

  ROS_INFO("Requesting the map...");
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
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
uint8_t LocalHardLayer::interpretValue(int8_t value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value > unknown_cost_value_)
    return LETHAL_OBSTACLE;
  else
    return FREE_SPACE;
}

void LocalHardLayer::visionHardCb(const nav_msgs::OccupancyGridConstPtr& hardPatch)
{
  Costmap2D* master_costmap = layered_costmap_->getCostmap();
  matchSize();
	// Boundaries of the buffer
	double res = bufferCostmap_->info.resolution;
	unsigned int size_x = bufferCostmap_->info.width, size_y = bufferCostmap_->info.height;
  unsigned int map_minX = static_cast<int>(bufferCostmap_->info.origin.position.x / res);
  unsigned int map_minY = static_cast<int>(bufferCostmap_->info.origin.position.y / res);
  unsigned int map_maxX = map_minX + size_x;
  unsigned int map_maxY = map_minY + size_y;

	// Boundaries of the master grid
	unsigned int master_minX = static_cast<int>( master_costmap->getOriginX() / master_costmap->getResolution() );
  unsigned int master_minY = static_cast<int>( master_costmap->getOriginY() / master_costmap->getResolution() );
  unsigned int master_maxX = master_minX + master_costmap->getSizeInCellsX();
  unsigned int master_maxY = master_minY + master_costmap->getSizeInCellsY();

  unsigned int it = 0;
  unsigned int index = 0;
	for(unsigned int i=0; i<=master_costmap->getSizeInCellsX(); i++)
	{
    for(unsigned int j=0; j<=master_costmap->getSizeInCellsY(); j++)
		{
			it = (j+(master_minX - map_minX)) + (i+(master_minY - map_minY))*bufferCostmap_->info.width;
			if (hardPatch->data[it] != unknown_cost_value_)
        costmap_[index] = interpretValue(hardPatch->data[it]);
      index++;
    }
  }

  has_updated_data_ = true;
  map_received_ = true;
}

void LocalHardLayer::bufferUpdate(const nav_msgs::OccupancyGridPtr& buffer,
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
      int coords = static_cast<int>(round(xn / buffer->info.resolution)) +
            static_cast<int>(round(yn / buffer->info.resolution)) * buffer->info.width;
      if ((coords > buffer->data.size()) || (coords < 0))
      {
        ROS_ERROR("Error resizing patch to buffer.");
      }
      else
      {
        uint8_t temp = patch->data[ii + jj * patch->info.width];
        if (temp != unknown_cost_value_)
          buffer->data[coords] = temp;
        mapDilation(buffer, 1, coords);
      }

      // if (ii == 0 && jj == 0)
      // {
      //   x_ = static_cast<int>(round(xn / buffer->info.resolution));
      //   y_ = static_cast<int>(round(yn / buffer->info.resolution));
      // }
    }
  }
  // width_ = static_cast<int>(round(patch->info.width * patch->info.resolution / buffer->info.resolution));
  // height_ = static_cast<int>(round(patch->info.height * patch->info.resolution / buffer->info.resolution));
}

// void LocalHardLayer::slamCb(const nav_msgs::OccupancyGridConstPtr& slamMap)
// {
//   //matchSize();
//
//
//   {
//     boost::unique_lock<boost::shared_mutex> lock(*getLock());
//     alignWithNewMap(slamMap, bufferCostmap_);
//   }
//
//   // if (has_updated_data_ || changed)
//   // {
//   //   boost::unique_lock<boost::shared_mutex> lock(*getLock());
//
//   //   for (int xx = 0; xx < bufferCostmap_->data.size(); ++xx)
//   //   {
//   //     costmap_[xx] = interpretValue(bufferCostmap_->data[xx]);
//   //   }
//   // }
//
//   map_received_ = true;
// }

void LocalHardLayer::activate()
{
  onInitialize();
}

void LocalHardLayer::deactivate()
{
  slam_map_sub_.shutdown();
  vision_hard_sub_.shutdown();
}

void LocalHardLayer::reset()
{
  deactivate();
  activate();
}

void LocalHardLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!map_received_ || !has_updated_data_)
    return;

  double mx, my;
  // convert from map coordinates to world coordinates
  // x_, y_ are the map coordinates mx, my are the world coordinates
  mapToWorld(0, 0, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);

  mapToWorld(size_x_, size_y_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);

  has_updated_data_ = false;
}

void LocalHardLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;
  if (!enabled_)
    return;

	updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

bool LocalHardLayer::alignWithNewMap(const nav_msgs::OccupancyGridConstPtr& in,
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
    out->data.resize(newSize, unknown_cost_value_);
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
          int coords = static_cast<int>(round(xn / out->info.resolution)) +
                static_cast<int>(round(yn / out->info.resolution)) * out->info.width;
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

      unsigned int widthDiff = fabs(floor(xDiff / out->info.resolution));
      unsigned int heightDiff = fabs(floor(yDiff / out->info.resolution));

      for (unsigned int ii = 0; ii < out->info.width; ++ii) {
        for (unsigned int jj = 0; jj < out->info.height; ++jj) {
          if (ii >= oldMetaData.width + widthDiff ||
              jj >= oldMetaData.height + heightDiff)
            out->data[ii + jj * out->info.width] = unknown_cost_value_;
        }
      }
      if (widthDiff != 0 || heightDiff != 0)
      {
        for (unsigned int ii = 0; ii < out->info.width; ++ii) {
          for (unsigned int jj = 0; jj < out->info.height; ++jj) {
            if (ii < widthDiff || jj < heightDiff)
              out->data[ii + jj * out->info.width] = unknown_cost_value_;
          }
        }
      }
    }
  }
  delete[] oldMap;
  return newSize != oldSize;
}

void LocalHardLayer::mapDilation(const nav_msgs::OccupancyGridPtr& in,
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
      if (!check || checkMap->data[coords + in->info.width + 1] < unknown_cost_value_)
      {
        in->data[coords + in->info.width + 1] = cell;
        mapDilation(in, steps - 1, coords + in->info.width + 1);
      }
    }
    if (in->data[coords + in->info.width] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width] < unknown_cost_value_)
        in->data[coords + in->info.width] = cell;
    }
    if (in->data[coords + in->info.width - 1] == 0)
    {
      if (!check || checkMap->data[coords + in->info.width - 1] < unknown_cost_value_)
      {
        in->data[coords + in->info.width - 1] = cell;
        mapDilation(in, steps - 1, coords + in->info.width - 1);
      }
    }
    if (in->data[coords + 1] == 0)
    {
      if (!check || checkMap->data[coords + 1] < unknown_cost_value_)
        in->data[coords + 1] = cell;
    }
    if (in->data[coords - 1] == 0)
    {
      if (!check || checkMap->data[coords - 1] < unknown_cost_value_)
        in->data[coords - 1] = cell;
    }
    if (in->data[coords - in->info.width + 1] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width + 1] < unknown_cost_value_)
      {
        in->data[coords - in->info.width + 1] = cell;
        mapDilation(in, steps - 1, coords - in->info.width + 1);
      }
    }
    if (in->data[coords - in->info.width] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width] < unknown_cost_value_)
        in->data[coords - in->info.width] = cell;
    }
    if (in->data[coords - in->info.width - 1] == 0)
    {
      if (!check || checkMap->data[coords - in->info.width - 1] < unknown_cost_value_)
      {
        in->data[coords - in->info.width - 1] = cell;
        mapDilation(in, steps - 1, coords - in->info.width - 1);
      }
    }
  }
}

}
