#ifndef MOVE_BACKWARDS_RECOVERY_MOVE_BACKWARDS_RECOVERY_H_
#define MOVE_BACKWARDS_RECOVERY_MOVE_BACKWARDS_RECOVERY_H_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Reconfigure.h>
#include <sensor_msgs/LaserScan.h>

namespace move_backwards_recovery
{
  class MoveBackwardsRecovery : public nav_core::RecoveryBehavior
  {
    public:
      MoveBackwardsRecovery();
      ~MoveBackwardsRecovery();

      /// Initialize the parameters of the behavior
      void initialize (std::string n, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* global_costmap,
          costmap_2d::Costmap2DROS* local_costmap);

      /// Run the behavior
      void runBehavior();
      void laserCallback(const sensor_msgs::LaserScanPtr& laser_scan);
    private:
      ros::NodeHandle private_nh_, planner_nh_;
      costmap_2d::Costmap2DROS* global_costmap_;
      costmap_2d::Costmap2DROS* local_costmap_;
      bool initialized_;
      double linear_escape_vel_, angular_escape_vel_;
      ros::ServiceClient planner_dynamic_reconfigure_service_;
      ros::Subscriber laser_subscriber_;
      sensor_msgs::LaserScanPtr laser_scan_;
  };
};

#endif
