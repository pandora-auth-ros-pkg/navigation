#include <collision_recovery/collision_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_2d.h>


PLUGINLIB_DECLARE_CLASS(collsion_recovery, CollisionRecovery, collision_recovery::CollisionRecovery,
    nav_core::RecoveryBehavior)

namespace collision_recovery
{
  CollisionRecovery::CollisionRecovery(): global_costmap_(NULL), local_costmap_(NULL),
    initialized_(false) {}

  CollisionRecovery::~CollisionRecovery()
  {
  }

  void CollisionRecovery::initialize (std::string n, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    footprint_collision_.resize(4, 0);
    if(!initialized_){

      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;
      lines_ = false;
      yolo = false;
      ros::NodeHandle private_nh_("~/" + n);
      private_nh_.param("linear_escape_vel", linear_escape_vel_, 0.3);
      private_nh_.param("angular_escape_vel", angular_escape_vel_, 0.2);
      linear_escape_vel_ = 0.3;
      angular_escape_vel_ = 0.2;
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

  void CollisionRecovery::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("[Move_Backwards_Recovery] This object must be initialized before runBehavior is called");
      return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
      ROS_ERROR("[Move_Backwards_Recovery] The costmaps passed to the CollisionRecovery object cannot be NULL. Doing nothing.");
      return;
    }

    ROS_WARN("[Move_Backwards_Recovery] Move backwards recovery behavior started.");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    /** Recovery **/
    geometry_msgs::Twist cmd_vel;

    std::cout<<"Footprint Collision Vector Before"<<footprint_collision_[0]<<footprint_collision_[1]
    <<footprint_collision_[2]<<footprint_collision_[3]<<std::endl;


    if(footprint_collision_.size() != 0)
    {
      if(lines_ == true)
      {
        // if -1 collision, if 0 not
        int back = footprint_collision_.at(0);
        int left = footprint_collision_.at(1);
        int front = footprint_collision_.at(2);
        int right = footprint_collision_.at(3);

        if(front == -1 && left == -1 && back == -1 && right == -1)
        {
          ROS_WARN("[CollisionCheckingRecovery] You really messed this up dude! Going back #YOLO vel_x[%f]",cmd_vel.linear.x);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }
        if(front == -1 && left == -1 && back == 0 && right == 0)
        {
          ROS_WARN("[CollisionCheckingRecovery] Front Left in collision, turning back right vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = angular_escape_vel_;
        }
        if(front == -1 && left == 0 && back == 0 && right == -1)
        {
          ROS_WARN("[CollisionCheckingRecovery] Front Right in collision, turning back left vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = -angular_escape_vel_;
        }

        if(front == 0 && left == -1 && back == -1 && right == 0)
        {
          ROS_WARN("[CollisionCheckingRecovery] Left back in collision, going forward right vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = -angular_escape_vel_;
        }

        if(front == 0 && left == 0 && back == -1 && right == -1)
        {
          ROS_WARN("[CollisionCheckingRecovery] Right back in collision, going forward left vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = angular_escape_vel_;
        }

        if(front == 0 && left == -1 && back == -1 && right == -1)
        {
          ROS_WARN("[CollisionCheckingRecovery] Only front free, going forward vel_x[%f]", cmd_vel.linear.x);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }

        if(front == 0 && left == 0 && back == -1 && right == -1)
        {
          ROS_WARN("[CollisionCheckingRecovery] Only back free, going back vel_x[%f]", cmd_vel.linear.x);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }

        if(front == 0 && left == 0 && back == 0 && right == 0)
        {
          ROS_WARN("[CollisionCheckingRecovery] Ta triatafila einai kokkina oi toulipes einai ple, tsifsa rop ki ola kople, tpt den xtypaei");
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }
      }
      else
      {
        int index = 0;
        while(index<=3)
        {
          std::vector<geometry_msgs::Point> footprint;
          global_costmap_->getOrientedFootprint(footprint);
          std::cout<<footprint[0]<<std::endl;
          std::cout<<footprint[1]<<std::endl;
          std::cout<<footprint[2]<<std::endl;
          std::cout<<footprint[3]<<std::endl;
          std::cout<<"Before create footprint Collision"<<std::endl;
          createFootprintCollision(footprint);
          std::cout<<"Footprint Collision Vector After"<<footprint_collision_[0]
          <<footprint_collision_[1]<<footprint_collision_[2]<<footprint_collision_[3]<<std::endl;

          // if -1 collision, if 0 not
          int back_right = footprint_collision_.at(0);
          int back_left = footprint_collision_.at(1);
          int front_left = footprint_collision_.at(2);
          int front_right = footprint_collision_.at(3);

          if(back_right == -1 && back_left == -1 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[CollisionCheckingRecovery] You really messed this up dude! Going back #YOLO vel_x[%f]",cmd_vel.linear.x);
          }
          if(back_right == -1 && back_left == -1 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[CollisionCheckingRecovery] Back Left and Back Right in collision going forward vel_x[%f]", cmd_vel.linear.x);
          }
          if(back_right == -1 && back_left == 0 && front_left == 0 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_escape_vel_;
            ROS_WARN("[CollisionCheckingRecovery] Back Right Front Right in collision, turning back left vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }

          if(back_right == 0 && back_left == -1 && front_left == -1 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -angular_escape_vel_;
            ROS_WARN("[CollisionCheckingRecovery] Back Left Front left in collision, turning back right vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }


          if(back_right == 0 && back_left == -1 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -angular_escape_vel_;
            ROS_WARN("[CollisionCheckingRecovery] Only back_right free, going back right vel_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }

          if(back_right == 0 && back_left == 0 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[CollisionCheckingRecovery] Front left and right in collision, going back vel_x[%f]", cmd_vel.linear.x);
          }

          if(back_right == 0 && back_left == 0 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[CollisionCheckingRecovery] Ta triatafila einai kokkina oi toulipes einai ple, tsifsa rop ki ola kople, tpt den xtypaei");
            break;
          }
          index++;
          vel_pub.publish(cmd_vel);
        }

      }


    }
    else
      ROS_ERROR("[CollisionCheckingRecovery] Collision vector empty, sending zero velocities!");

    /****/
  }

  /*
  Checks which lines of the footprint are in collision(-1) and returns a vector
  that corresponds to those lines (back, left, front, right) needs a 2D costmap
  to be stored
  */
  std::vector<int> CollisionRecovery::createFootprintCollision(
    std::vector<geometry_msgs::Point>& footprint)
  {
    std::cout<<"Size of footprint collision vector:"<<footprint.size()<<std::endl;
    if(footprint.size() == 4)
    {
      if(lines_ == true)
      {
        // check back line for collision
        ROS_ERROR("Checking Back Line");
        footprint_collision_.at(0) = lineInCollision(footprint.at(0), footprint.at(1));

        // check left line for collision
        ROS_ERROR("Checking Left Line");
        footprint_collision_.at(1) = lineInCollision(footprint.at(1), footprint.at(2));

        // check front line for collision
        ROS_ERROR("Checking Front Line");
        footprint_collision_.at(2) = lineInCollision(footprint.at(2), footprint.at(3));

        // check right line for collision
        ROS_ERROR("Checking Right Line");
        footprint_collision_.at(3) = lineInCollision(footprint.at(3), footprint.at(0));

      }
      else
      {
        // check back right point for collision
        ROS_ERROR("Checking Back Right point");
        footprint_collision_.at(0) = pointInCollision(footprint.at(0).x, footprint.at(0).y);

        // check back left point for collision
        ROS_ERROR("Checking Back Left point");
        footprint_collision_.at(1) = pointInCollision(footprint.at(1).x, footprint.at(1).y);

        // check front line for collision
        ROS_ERROR("Checking Front Left point");
        footprint_collision_.at(2) = pointInCollision(footprint.at(2).x, footprint.at(2).y);

        // check right line for collision
        ROS_ERROR("Checking Front Right point");
        footprint_collision_.at(3) = pointInCollision(footprint.at(3).x, footprint.at(3).y);
      }


    }
    else
    {
      ROS_ERROR("[Move Back Recovery] Footprint not a rectangle!");
    }
    return footprint_collision_;
  }
  /* Helpers */
  // if point is in collision return -1, else 0
  int CollisionRecovery::pointInCollision(double wx, double wy)
  {
    int mx, my;
    global_costmap_->getCostmap()->worldToMapEnforceBounds(wx, wy, mx, my);
    unsigned char cost = global_costmap_->getCostmap()->getCost(mx, my);
    if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      ROS_INFO("Point in collision");
      return -1;
    }
    else
      return 0;
  }
  /**
   * @param p0 start point of line
   * @param p1 end point of line
   */
  std::vector<geometry_msgs::Point>linePointVectorCreator(geometry_msgs::Point& p0, geometry_msgs::Point& p1)
  {
    // Initialization
    std::vector<geometry_msgs::Point>line_point_vector;
    double step = 0.0, x_r = 0.0, y_r = 0.0;
    double resolution = 0.02;
    // yaw of the line that is defined by the two points
    double yaw = atan2((p1.y - p0.y), (p1.x - p0.x));
    // Euclidean distance between the two points
    double distance = sqrt( (p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y) );

    int num_steps = static_cast<int>(distance / resolution);
  	for(int int_step=1; int_step< num_steps; int_step++)
  	{
  		geometry_msgs::Point line_point;
      step = int_step * resolution;
  		x_r = p0.x + cos(yaw) * step;
  		y_r = p0.y + sin(yaw) * step;
      line_point.x = x_r;
      line_point.y = y_r;
      line_point_vector.push_back(line_point);
    }
    return line_point_vector;
  }

  /* Iterates the points of a line, if any of the line points is in collision
  then the line is considered in collision
  return -1 if in collision
  */
  int CollisionRecovery::lineInCollision(geometry_msgs::Point& p0, geometry_msgs::Point& p1)
  {
    std::vector<geometry_msgs::Point> line_point_vector = linePointVectorCreator(p0, p1);

    for(int i = 0; i < line_point_vector.size(); i++)
    {
      // When you find a not valid point you return
      if(pointInCollision(line_point_vector[i].x, line_point_vector[i].x) == -1)
        return -1;
    }

    return 0;
  }
}
