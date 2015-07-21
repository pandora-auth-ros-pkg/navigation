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
    footprint_collision_.resize(4, 0);
    if(!initialized_){

      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;

      ros::NodeHandle private_nh_("~/" + n);
      private_nh_.param("linear_escape_vel", linear_escape_vel_, 0.05);
      private_nh_.param("angular_escape_vel", angular_escape_vel_, 0.10);

      std::string planner_namespace;
      private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));
      planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
      planner_dynamic_reconfigure_service_ = planner_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);

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

    /** Recovery **/
    geometry_msgs::Twist cmd_vel;

    std::vector<geometry_msgs::Point> footprint = global_costmap_->getRobotFootprint();
    createFootprintCollision(footprint);

    if(footprint_collision_.size() != 0)
    {
      // if -1 collision, if 0 not
      int back = footprint_collision_.at(0);
      int left = footprint_collision_.at(1);
      int front = footprint_collision_.at(2);
      int right = footprint_collision_.at(3);
      if(front == -1)
      {
        ROS_WARN("[MoveBackRecovery] Front Footprint Line in collision, moving back");
        cmd_vel.linear.x = -0.1;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
      }
      else if()
      {
        if(front == -1)
        {
          ROS_WARN("[MoveBackRecovery] Back Footprint Line in collision, moving front");
          cmd_vel.linear.x = 0.1;
          cmd_vel.linear.y = 0;
          cmd_vel.angular.z = 0;
        }
      }
    }
    else
      ROS_ERROR("[MoveBackRecovery] Collision vector empty, sending zero velocities!");

    /****/

    vel_pub.publish(cmd_vel);
  }

  /*
  Checks which lines of the footprint are in collision(-1) and returns a vector
  that corresponds to those lines (back, left, front, right) needs a 2D costmap
  to be stored
  */
  std::vector<int> MoveBackwardsRecovery::createFootprintCollision(
    std::vector<geometry_msgs::Point>& footprint)
  {
    if(footprint.size() == 4)
    {
      // check back line for collision
      int back_line = lineInCollision(footprint.at(0).x, footprint.at(1).x, footprint.at(0).y, footprint.at(1).y);
      footprint_collision_.at(0) = back_line;

      // check left line for collision
      int left_line = lineInCollision(footprint.at(1).x, footprint.at(2).x, footprint.at(1).y, footprint.at(2).y);
      footprint_collision_.at(1) = left_line;

      // check front line for collision
      int front_line = lineInCollision(footprint.at(2).x, footprint.at(3).x, footprint.at(2).y, footprint.at(3).y);
      footprint_collision_.at(2) = front_line;

      // check right line for collision
      int right_line = lineInCollision(footprint.at(3).x, footprint.at(0).x, footprint.at(3).y, footprint.at(0).y);
      footprint_collision_.at(3) = right_line;

    }
    else
    {
      ROS_ERROR("[Move Back Recovery] Footprint not a rectangle!");
    }
    return footprint_collision_;
  }
  /* Helpers */
  // if point is in collision return -1, else 0
  int MoveBackwardsRecovery::pointInCollision(int x, int y)
  {
    unsigned char cost = global_costmap_->getCostmap()->getCost(x,y);
    if(cost == costmap_2d::LETHAL_OBSTACLE)
      return -1;
    else
      return 0;
  }

  /* Iterates the points of a line, if any of the line points is in collision
  then the line is considered in collision
  return -1 if in collision
  */
  int MoveBackwardsRecovery::lineInCollision(int x0, int x1, int y0, int y1){

    int line_cost = 0;
    int point_cost = 0;

    for( base_local_planner::LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
      point_cost = pointInCollision( line.getX(), line.getY() ); //Score the current point

      if(point_cost == -1)
        return -1;
    }

    return line_cost;
  }
};
