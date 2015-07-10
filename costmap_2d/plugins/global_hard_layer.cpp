#include <costmap_2d/global_hard_layer.h>
#include <pluginlib/class_list_macros.h>

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

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));

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

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void GlobalHardLayer::matchSize()
{
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


//void GlobalHardLayer::mapResizer(){}

void GlobalHardLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  // Pairnw to map kai to kanw resize
  boost::shared_ptr<nav_msgs::OccupancyGrid> map_copy(new nav_msgs::OccupancyGrid);
  *map_copy = *new_map;

  // The master grid data
  Costmap2D* master = layered_costmap_->getCostmap();
  unsigned int sizeX = master->getSizeInCellsX();  // [cells]
  unsigned int sizeY = master->getSizeInCellsY();  // [cells]
  float res = master->getResolution();  // [m/cell]
  float origX = master->getOriginX();  // [m]
  float origY = master->getOriginY();  // [m]

  // The difference of origins in meters
  float xDiff = new_map->info.origin.position.x - origX;  // [m]
  float yDiff = new_map->info.origin.position.y - origY;  // [m]

  // The differenceof origins in cells
  unsigned int xDiffCells = static_cast<unsigned int>(xDiff / res)+1;
  unsigned int yDiffCells = static_cast<unsigned int>(yDiff / res)+1;

  // Set the MapMetaData of the master grid to the map_copy
  map_copy->info.width = sizeX;
  map_copy->info.height = sizeY;
  map_copy->info.resolution = res;
  map_copy->info.origin.position.x = origX;
  map_copy->info.origin.position.y = origY;

  // A temporary array with the size of the master, containing NO_INFO
  std::vector<signed char> temp_arr(sizeX * sizeY, 51);

  unsigned int it = 0;

  for(unsigned int ii=0; ii<new_map->info.width; ii++)
  {
    for(unsigned int jj=0; jj<new_map->info.height; jj++)
    {
      temp_arr[jj + ii * new_map->info.width] = new_map->data[jj + ii * new_map->info.width];
      //++it;
    }
  }
  map_copy->data = temp_arr;
  ROS_INFO("Map copy Width: %d,  Height: %d", map_copy->info.width, map_copy->info.height);
  //mapResizer(*map_copy, sizeX, sizeY, res, origX, origY);

  unsigned int size_x = map_copy->info.width, size_y = map_copy->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, map_copy->info.resolution);

  // resize costmap if size, resolution or origin do not match
  //Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != map_copy->info.resolution ||
      master->getOriginX() != map_copy->info.origin.position.x ||
      master->getOriginY() != map_copy->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_INFO("[HardLayer]Resizing costmap to %d X %d at %f m/pix", size_x, size_y, map_copy->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, map_copy->info.resolution, map_copy->info.origin.position.x,
                               map_copy->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != map_copy->info.resolution ||
      origin_x_ != map_copy->info.origin.position.x ||
      origin_y_ != map_copy->info.origin.position.y){
    matchSize();
  }

  unsigned int index = 0;

  for (unsigned int i = 0; i < size_y_; ++i)
  {
    for (unsigned int j = 0; j < size_x_; ++j)
    {
	    unsigned char value = map_copy->data[index];
	    // if(interpretValue(value) == NO_INFORMATION)
	    // {
		  //   ++index;  // auto mallon ftaiei gia to mov
	    // }
	    // else
	    // {
      costmap_[index] = interpretValue(value);
      ++index;
	    // }
    }
  }

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}
unsigned char GlobalHardLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (value > 51)
    return LETHAL_OBSTACLE;
  else if (value<51)
    return FREE_SPACE;
  else
    return NO_INFORMATION;
  // if (value == unknown_cost_value_)
  //   return NO_INFORMATION;
  // else if (value >= lethal_threshold_)
  //   return LETHAL_OBSTACLE;
  // else if (trinary_costmap_)
  //   return FREE_SPACE;
  //
  // double scale = (double) value / lethal_threshold_;
  // return scale * LETHAL_OBSTACLE;
}


void GlobalHardLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

} // end namespace
