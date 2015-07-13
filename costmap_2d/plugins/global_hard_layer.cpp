#include <costmap_2d/global_hard_layer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::GlobalHardLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

GlobalHardLayer::GlobalHardLayer() :dsrv_(NULL) {}

void GlobalHardLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  global_frame_ = layered_costmap_->getGlobalFrameID();

  //std::string map_topic;
  //nh.param("map_topic", map_topic, std::string("map"));
  // My topics
  std::string slamMapTopic;
  std::string elevationMapTopic;
  nh.param("slam_topic", slamMapTopic, "/slam/map");
  nh.param("elevation_map_topic", elevationMapTopic, "/vision/traversability_map");

  slamMapSub = g_nh.subscribe(slamMapTopic, 1, &GlobalHardLayer::slamCB, this);
  elevationMapSub = g_nh.subscribe(elevationMapTopic, 1, &GlobalHardLayer::elevationMapCB, this);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("[HardLayer] Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &GlobalHardLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("[HardLayer] Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GlobalHardLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
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

void GlobalHardLayer::matchSize()
{
  ROS_ERROR("[Hard Layer] Kanw matching");
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
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


// void GlobalHardLayer::mapResizer(const boost::shared_ptr<nav_msgs::OccupancyGrid> input,
// boost::shared_ptr<nav_msgs::OccupancyGrid> output)
// {
//   if(input.get() == NULL)
//   {
//     ROS_ERROR("[HardLayer] Input map not initialized");
//     throw // Todo
//   }
//   output->
// }

/*
* Paragkaaaaaaaaaaaaaaaaaaaaaa
*/
void GlobalHardLayer::innerCostmapUpdate(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  int it = 0;
  for (int j = 0; j < new_map->info.width; j++)
  {
    for (int i = 0; i < new_map->info.height; i++)
    {
      if (costmap_[it] != NO_INFORMATION && costmap_[it] != FREE_SPACE && costmap_[it] != LETHAL_OBSTACLE)
      {
        unsigned char x = costmap_[it];
        ROS_ERROR_THROTTLE(100,"%u",costmap_[it]);
        ROS_ERROR_COND(x == 255, "Shieet 255");
        ROS_ERROR_COND(x == 0, "MPOUTSA 0");
        ROS_ERROR_COND(x == 254, "SKATA 254");
        costmap_[it] = NO_INFORMATION;
      }
      //printf("Incoming %d \n", new_map->data[it]);
      //printf("Costmap before ---------------------  %u \n", costmap_[it]);
      if (new_map->data[it] != 51)
        costmap_[it] = interpretValue(new_map->data[it]);
      //printf("Costmap after ------------------------------------------  %u \n", costmap_[it]);
      it++;
    }

  }
}

void GlobalHardLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  Costmap2D* master = layered_costmap_->getCostmap();

  unsigned int master_size_x = master->getSizeInCellsX();
  unsigned int master_size_y = master->getSizeInCellsY();
  double master_origin_x = master->getOriginX();
  double master_origin_y = master->getOriginY();

  ROS_INFO("[Debug Hard]Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);
  ROS_INFO("size_x of costmap[%d] size_y of costmap[%d]",size_x_,size_y_);

  if(master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_ERROR("[Hard Layer] Resizing Master with the new message data");
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }
  else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y)
  {
    matchSize();
  }
  // resize costmap if size, resolution or origin do not match
  //matchSize();
  unsigned int index = 0;

  //minX = static_cast<>(new_map->info.origin.position.x)/new_map->info.
  innerCostmapUpdate(new_map);
  //initialize the costmap with static data
  // for (unsigned int i = 0; i < size_y_; ++i)
  // {
  //   for (unsigned int j = 0; j < size_x_; ++j)
  //   {
  //     unsigned char value = new_map->data[index];
  //     costmap_[index] = interpretValue(value);
  //     ++index;
  //   }
  // }

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

}

// Callback to slam map
void GlobalHardLayer::slamCB(const nav_msgs::OccupancyGridConstPtr& slamMap)
{
  if (bufferCostmap_.getSizeInCellsX() != slamMap->info.width ||
      bufferCostmap_.getSizeInCellsY() != slamMap->info.height ||
      bufferCostmap_.getOriginX() != slamMap->info.origin.position.x ||
      bufferCostmap_.getOriginY() != slamMap->info.origin.position.y ||
      bufferCostmap_.getResolution() != slamMap->info.resolution
  )
  {
    bu
  }












}

// Callback to vision traversability map
void GlobalHardLayer::elevationMapCB(const nav_msgs::OccupancyGridConstPtr& elevationMap)
{

}

// unsigned char GlobalHardLayer::interpretValueInd(unsigned char value, unsigned int indx)
// {
//   Costmap2D* master = layered_costmap_->getCostmap();
//   //check if the static value is above the unknown or lethal thresholds
//   // if (value > 51)
//   //   return LETHAL_OBSTACLE;
//   // else
//   //   return NO_INFORMATION;
//   if (value == unknown_cost_value_)
//     return *(master->getCharMap()+indx);
//   else if (value >= lethal_threshold_)
//     return LETHAL_OBSTACLE;
//   else
//     return FREE_SPACE;
//   //
//   // double scale = (double) value / lethal_threshold_;
//   // return scale * LETHAL_OBSTACLE;
// }

unsigned char GlobalHardLayer::interpretValue(unsigned char value)
{
  //ROS_ERROR("%u",value);
  if (value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else
    return FREE_SPACE;
}


void GlobalHardLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

} // end namespace
