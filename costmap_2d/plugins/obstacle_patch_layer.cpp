#include <costmap_2d/obstacle_patch_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstaclePatchLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

ObstaclePatchLayer::ObstaclePatchLayer() :dsrv_(NULL) {}

void ObstaclePatchLayer::onInitialize()
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
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &ObstaclePatchLayer::incomingMap, this);
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
      &ObstaclePatchLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}



void ObstaclePatchLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
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

void ObstaclePatchLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void ObstaclePatchLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void ObstaclePatchLayer::activate()
{
    onInitialize();
}

void ObstaclePatchLayer::deactivate()
{
    map_sub_.shutdown();
}

void ObstaclePatchLayer::reset()
{
    deactivate();
    activate();
}


/**
 * @brief This function finds if there is an intersection between the master_grid and an another map.
 *
 * @param minBB_x The bottom left of the bounding box, will be filled with the bottom left of the intersection map
 * @param maxBB_x
 * @param minBB_y
 * @param maxBB_y
 * @return True if there is a section between the master grid and the map that is defined by the bounding
 * box we pass through the params
 *
 * The master_grid is stored inside the class. The params that we pass define the map we want to check if it
 * intersects the master_grid. If there is a section between those two, the arguments we passed by pointer
 * are going to be filled with the bounding box that defines the SECTION between the two maps.
 */
bool ObstaclePatchLayer::mapIntersectsMaster(std::vector<unsigned int>* bounding_box)
{
  Costmap2D* master = layered_costmap_->getCostmap();
  if(master == NULL)
  {
    ROS_ERROR("TSOMPANARAIIIIIIOOIIIIIII");
  }
  // Find the bounding box that is defined by the master grid
  // The bounding box is in cells
  unsigned int master_minX = static_cast<int>( master->getOriginX() / master->getResolution() );
  unsigned int master_minY = static_cast<int>( master->getOriginY() / master->getResolution() );
  unsigned int master_maxX = master_minX + master->getSizeInCellsX();
  unsigned int master_maxY = master_minY + master->getSizeInCellsY();

  // The given bounding box
  unsigned int minX = (*bounding_box)[0];
  unsigned int maxX = (*bounding_box)[1];
  unsigned int minY = (*bounding_box)[2];
  unsigned int maxY = (*bounding_box)[3];

  unsigned int minBB_x, maxBB_x, minBB_y, maxBB_y;

  // Here we check if there is a section. If there is not, we return false
  //if (master_maxX < minX || master_minX > maxX || master_maxY < minY || master_minY > maxY)
  //  return false;


  // Step 2 : Find section X
  if (master_maxX <= maxX)
  {
    minBB_x = std::max(master_minX, minX);
    maxBB_x = master_maxX;
  }

  else if (master_maxX > maxX)
  {
    minBB_x = std::max(master_minX, minX);
    maxBB_x = maxX;
  }

  // Step 3 : Find section _Y
  if (master_maxY <= maxY)
  {
    minBB_y = std::max(master_minY, minY);
    maxBB_y = master_maxY;
  }

  else if (master_maxY > maxY)
  {
    minBB_y = std::max(master_minY,minY);
    maxBB_y = maxY;
  }

  (*bounding_box)[0] = minBB_x;
  (*bounding_box)[1] = maxBB_x;
  (*bounding_box)[2] = minBB_y;
  (*bounding_box)[3] = maxBB_y;
  return true;
}


void ObstaclePatchLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  // A pointer to the master_grid
  Costmap2D* master = layered_costmap_->getCostmap();

  // Check origin matching?????

  // Check if the resolution of the incoming map and the master grid is the same
  //ROS_WARN("This is the master's data: originX[%f] originY[%f] width[%d] height[%d] res[%f]",
    //master->getOriginX(), master->getOriginY(),master->getSizeInCellsX(),master->getSizeInCellsY(),master->getResolution());

  double res  = (ceilf(new_map->info.resolution * 100) / 100);
  if((ceilf(master->getResolution() * 100) / 100) != (ceilf(new_map->info.resolution * 100) / 100))
  {
    ROS_ERROR("Incoming Map in obstacle_cover_layer and master_grid resolutions are different!");
    ROS_ERROR("Master's res: [%f], Incoming map's res: [%f]",(ceilf(master->getResolution() * 100) / 100), (ceilf(new_map->info.resolution * 100) / 100));
    return;
  }

  // Create the new map bounding box in cells and store it to a vector
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  unsigned int map_minX = static_cast<int>(new_map->info.origin.position.x / res);
  unsigned int map_minY = static_cast<int>(new_map->info.origin.position.y / res);
  unsigned int map_maxX = map_minX + size_x;
  unsigned int map_maxY = map_minY + size_y;

  unsigned int master_minX = static_cast<int>( master->getOriginX() / master->getResolution() );
  unsigned int master_minY = static_cast<int>( master->getOriginY() / master->getResolution() );
  unsigned int master_maxX = master_minX + master->getSizeInCellsX();
  unsigned int master_maxY = master_minY + master->getSizeInCellsY();

  std::vector<unsigned int> bounding_box_vect;
  bounding_box_vect.push_back(map_minX);
  bounding_box_vect.push_back(map_maxX);
  bounding_box_vect.push_back(map_minY);
  bounding_box_vect.push_back(map_maxY);

  //ROS_WARN("Created the new map bounding box in cells and store it to a vector");
  // Initially we check if the incoming map has any common points with the master grid
  // If there is an intersection, it is returned in the passed vector
  //bool intersects = mapIntersectsMaster(&bounding_box_vect);

  //ROS_WARN("Checked for intersection [%d]", intersects);
  // if we have no intersection we fill the layer's costmap with NO_INFORMATION
  matchSize();
  unsigned int index = 0;

  unsigned int xBoundLow = master_minX - map_minX;
  //xBoundHigh = master_maxX;

  unsigned int yBoundLow = master_minY - map_minY;

  //ROS_ERROR("xBoundLow: [%d], yBoundLow: [%d]",xBoundLow,yBoundLow );
  //yBoundHigh = master_maxY;
  unsigned char value;
  for (unsigned int i = 0; i < master->getSizeInCellsX(); ++i)
  {
    for (unsigned int j = 0; j < master->getSizeInCellsY(); ++j)
    {
      value = interpretValue(new_map->data[(j+(master_minX - map_minX))+ (i+(master_minY - map_minY))* new_map->info.width]);
      //ROS_ERROR("Set value: [%d]", value);
      costmap_[index] = value;
      ++index;
    }
  }
  /*if(!intersects)
  {

    for (unsigned int i = 0; i < master->getSizeInCellsY(); ++i)
    {
      for (unsigned int j = 0; j < master->getSizeInCellsX(); ++j)
      {
        //ROS_ERROR("Crash??");
        costmap_[index] = NO_INFORMATION;
        ++index;
      }
    }
  }
  // if we have an intersection we update the costmap with the intersected region

  else
  {
    for (unsigned int i = 0; i < master->getSizeInCellsY(); ++i)
    {
      for (unsigned int j = 0; j < master->getSizeInCellsX(); ++j)
      {
        costmap_[index] = new_map->data[(i+yBoundLow) * master->getSizeInCellsX()  + (j+xBoundLow)];
        ++index;
      }
    }
  }*/

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

}

unsigned char ObstaclePatchLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}


void ObstaclePatchLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

} // end namespace
