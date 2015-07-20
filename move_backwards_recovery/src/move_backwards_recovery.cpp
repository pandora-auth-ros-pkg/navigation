#include <move_backwards_recovery/move_backwards_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/obstacle_layer.h>

PLUGINLIB_DECLARE_CLASS(move_backwards_recovery, MoveBackwardsRecovery, move_backwards_recovery::MoveBackwardsRecovery,
    nav_core::RecoveryBehavior)

namespace move_backwards_recovery
{
  MoveBackwardsRecovery::MoveBackwardsRecovery(): global_costmap_(NULL), local_costmap_(NULL),
    initialized_(false) {}

  MoveBackwardsRecovery::~MoveBackwardsRecovery()
  {
  }

  void MoveBackwardsRecovery::initialize (std::string n, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    if(!initialized_){
      laser_scan_.reset(new sensor_msgs::LaserScan);

      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;

      ros::NodeHandle private_nh_("~/" + n);
      private_nh_.param("linear_escape_vel", linear_escape_vel_, 0.05);
      private_nh_.param("angular_escape_vel", angular_escape_vel_, 0.10);

      std::string planner_namespace;
      private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));
      planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
      planner_dynamic_reconfigure_service_ = planner_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);

      laser_subscriber_ = planner_nh_.subscribe("/laser/scan", 1, &MoveBackwardsRecovery::laserCallback, this);

      initialized_ = true;

      ROS_WARN("[Move_Backwards_Recovery] Initialized move backwards recovery!");
    }
    else{
      ROS_ERROR("[Move_Backwards_Recovery] You should not call initialize twice on this object, doing nothing");
    }
  }

  void MoveBackwardsRecovery::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("[Move_Backwards_Recovery] This object must be initialized before runBehavior is called");
      return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
      ROS_ERROR("[Move_Backwards_Recovery] The costmaps passed to the MoveBackwardsRecovery object cannot be NULL. Doing nothing.");
      return;
    }

    ROS_WARN("[Move_Backwards_Recovery] Move backwards recovery behavior started.");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    double middle_ray = 0;
    if (laser_scan_->ranges.size() != 0)
    {
      // TODO: Check array in real laser
      //double right_ray = laser_scan_->ranges.at(0);
      //double left_ray = laser_scan_->ranges.at(laser_scan_->ranges.size() - 1);
      middle_ray = laser_scan_->ranges.at(laser_scan_->ranges.size() / 2);
    }
    else
      ROS_ERROR("[Move_Backwards_Recovery] Laser scan is empty.");

    geometry_msgs::Twist cmd_vel;

    double middle_threshold = 0.4;
    double side_threshold = 1.6;

    if (middle_ray > middle_threshold)
    {
      cmd_vel.linear.x = 0.11;
    }
    else
    {
      cmd_vel.linear.x = -0.11;
    }

    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

  }

  void MoveBackwardsRecovery::laserCallback(const sensor_msgs::LaserScanPtr& laser_scan)
  {
    laser_scan_ = laser_scan;
  }
};
