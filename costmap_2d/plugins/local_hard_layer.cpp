#include <costmap_2d/local_hard_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::LocalHardLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

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
    nh.param<int>("mitsos_cost_value", mitsos_cost_value_, 50);

    global_frame_ = layered_costmap_->getGlobalFrameID();

    if (!nh.getParam("global_hard_topic", global_hard_topic_))
    {
      ROS_FATAL("[%s] Cound not find global hard topic!", name_.c_str());
      ROS_BREAK();
    }
    global_hard_sub_ = g_nh.subscribe(global_hard_topic_, 1, &LocalHardLayer::globalHardCb, this);

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
    else if (value == mitsos_cost_value_)
      return INSCRIBED_INFLATED_OBSTACLE;
    else if (value > unknown_cost_value_)
      return LETHAL_OBSTACLE;
    else
      return FREE_SPACE;
  }

  void LocalHardLayer::globalHardCb(const nav_msgs::OccupancyGridConstPtr& hardPatch)
  {
    Costmap2D* master_costmap = layered_costmap_->getCostmap();
    matchSize();
  	// Boundaries of the buffer
  	double res = hardPatch->info.resolution;
  	unsigned int size_x = hardPatch->info.width, size_y = hardPatch->info.height;
    unsigned int map_minX = static_cast<int>(hardPatch->info.origin.position.x / res);
    unsigned int map_minY = static_cast<int>(hardPatch->info.origin.position.y / res);
    unsigned int map_maxX = map_minX + size_x;
    unsigned int map_maxY = map_minY + size_y;

  	// Boundaries of the master grid
  	unsigned int master_minX = static_cast<int>( master_costmap->getOriginX() / master_costmap->getResolution() );
    unsigned int master_minY = static_cast<int>( master_costmap->getOriginY() / master_costmap->getResolution() );
    unsigned int master_maxX = master_minX + master_costmap->getSizeInCellsX();
    unsigned int master_maxY = master_minY + master_costmap->getSizeInCellsY();
    unsigned int index = 0;
    unsigned char value;
    for (unsigned int i = 0; i < master_costmap->getSizeInCellsX(); ++i)
    {
      for (unsigned int j = 0; j < master_costmap->getSizeInCellsY(); ++j)
      {
        value = interpretValue(hardPatch->data[(j+(master_minX - map_minX))+ (i+(master_minY - map_minY))* hardPatch->info.width]);
        //ROS_ERROR("Set value: [%d]", value);
        costmap_[index] = value;
        ++index;
      }
    }


    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
    has_updated_data_ = true;
    map_received_ = true;
  }

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
    mapToWorld(x_, y_, mx, my);
    *min_x = std::min(mx, *min_x);
    *min_y = std::min(my, *min_y);

    mapToWorld(x_ + width_, y_ + height_, mx, my);
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
}
