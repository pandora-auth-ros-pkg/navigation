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

  void CollisionRecovery::initialize (std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    footprint_collision_.resize(4, 0);
    if(!initialized_){

      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;
      lines_ = false;
      yolo = false;
      ros::NodeHandle private_nh_("~/" + name);
      private_nh_.param("linear_escape_vel", linear_escape_vel_, 0.05);
      private_nh_.param("angular_escape_vel", angular_escape_vel_, 0.1);
      std::string planner_namespace;
      private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));
      planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
      planner_dynamic_reconfigure_service_ = planner_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);

      initialized_ = true;

      ROS_WARN("[collision_recovery] Loaded %s!", name.c_str());
    }
    else{
      ROS_ERROR("[collision_recovery] You should not call initialize twice on this object, doing nothing");
    }
  }

  void CollisionRecovery::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("[collision_recovery] This object must be initialized before runBehavior is called");
      return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
      ROS_ERROR("[collision_recovery] The costmaps passed to the CollisionRecovery object cannot be NULL. Doing nothing.");
      return;
    }

    ROS_WARN("[collision_recovery] Move backwards recovery behavior started.");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::Twist cmd_vel;

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
          ROS_WARN("[collision_recovery] You really messed this up dude! Going back #YOLO lin_x[%f]",cmd_vel.linear.x);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }
        if(front == -1 && left == -1 && back == 0 && right == 0)
        {
          ROS_WARN("[collision_recovery] Front Left in collision, turning back right lin_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = angular_escape_vel_;
        }
        if(front == -1 && left == 0 && back == 0 && right == -1)
        {
          ROS_WARN("[collision_recovery] Front Right in collision, turning back left lin_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = -angular_escape_vel_;
        }

        if(front == 0 && left == -1 && back == -1 && right == 0)
        {
          ROS_WARN("[collision_recovery] Left back in collision, going forward right lin_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = -angular_escape_vel_;
        }

        if(front == 0 && left == 0 && back == -1 && right == -1)
        {
          ROS_WARN("[collision_recovery] Right back in collision, going forward left lin_x[%f], ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = angular_escape_vel_;
        }

        if(front == 0 && left == -1 && back == -1 && right == -1)
        {
          ROS_WARN("[collision_recovery] Only front free, going forward lin_x[%f]", cmd_vel.linear.x);
          cmd_vel.linear.x = linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }

        if(front == 0 && left == 0 && back == -1 && right == -1)
        {
          ROS_WARN("[collision_recovery] Only back free, going back lin_x[%f]", cmd_vel.linear.x);
          cmd_vel.linear.x = -linear_escape_vel_;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }

        if(front == 0 && left == 0 && back == 0 && right == 0)
        {
          ROS_WARN("[collision_recovery] Ta triatafila einai kokkina oi toulipes einai ple, tsifsa rop ki ola kople, tpt den xtypaei");
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
        }
      }
      else
      {
        ros::NodeHandle n;
        ros::Rate r(1.0);

        int index = 0;
        while (n.ok() && index < 2)
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;

          std::vector<geometry_msgs::Point> footprint;
          global_costmap_->getOrientedFootprint(footprint);
          createFootprintCollision(footprint);

          // if -1 collision, if 0 not
          int back_right = footprint_collision_.at(0);
          int back_left = footprint_collision_.at(1);
          int front_left = footprint_collision_.at(2);
          int front_right = footprint_collision_.at(3);

          // No collision
          if(back_right == 0 && back_left == 0 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] Unstuck!");
            break;
          }

          // One point collision
          else if(back_right == -1 && back_left == 0 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] back_right in collision, going forward with lin_x[%f]", cmd_vel.linear.x);
          }
          else if(back_right == 0 && back_left == -1 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] back_left in collision, going forward with lin_x[%f]", cmd_vel.linear.x);
          }
          else if(back_right == 0 && back_left == 0 && front_left == -1 && front_right == 0)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] front_left in collision, going back with lin_x[%f]", cmd_vel.linear.x);
          }
          else if(back_right == 0 && back_left == 0 && front_left == 0 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] front_right in collision, going back with lin_x[%f]", cmd_vel.linear.x);
          }

          // Two point collision
          else if(back_right == -1 && back_left == -1 && front_left == 0 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] back_left and back_right in collision, going forward with lin_x[%f]", cmd_vel.linear.x);
          }
          else if(back_right == -1 && back_left == 0 && front_left == 0 && front_right == -1)
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_escape_vel_;
            ROS_WARN("[collision_recovery] back_right and front_right in collision, turning left with ang_z[%f]", cmd_vel.angular.z);
          }
          else if(back_right == 0 && back_left == -1 && front_left == -1 && front_right == 0)
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -angular_escape_vel_;
            ROS_WARN("[collision_recovery] back_left and front_left in collision, turning right with ang_z[%f]", cmd_vel.angular.z);
          }
          else if(back_right == 0 && back_left == 0 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_WARN("[collision_recovery] front_left and front_right in collision, going back with lin_x[%f]", cmd_vel.linear.x);
          }

          // Three point collision
          else if(back_right == 0 && back_left == -1 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -angular_escape_vel_;
            ROS_WARN("[collision_recovery] Only back_right free, going back and right with lin_x[%f] and ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }
          else if(back_right == -1 && back_left == 0 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_escape_vel_;
            ROS_WARN("[collision_recovery] Only back_left free, going back and left with lin_x[%f] and ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }
          else if(back_right == -1 && back_left == -1 && front_left == 0 && front_right == -1)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_escape_vel_;
            ROS_WARN("[collision_recovery] Only front_left free, going front and left with lin_x[%f] and ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }
          else if(back_right == -1 && back_left == -1 && front_left == -1 && front_right == 0)
          {
            cmd_vel.linear.x = linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -angular_escape_vel_;
            ROS_WARN("[collision_recovery] Only front_right free, going front and right with lin_x[%f] and ang_z[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
          }

          // Four point collision
          else if(back_right == -1 && back_left == -1 && front_left == -1 && front_right == -1)
          {
            cmd_vel.linear.x = -linear_escape_vel_;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_ERROR("[collision_recovery] You really messed this up dude! #YOLO, going back with lin_x[%f]",cmd_vel.linear.x);
          }

          else
            ROS_ERROR("[collision_recovery] The program should never reach here, please report this bug.");

          vel_pub.publish(cmd_vel);
          r.sleep();

          index++;
        }

      }


    }
    else
      ROS_ERROR("[collision_recovery] Collision vector empty, sending zero velocities!");
  }

  /*
  Checks which lines of the footprint are in collision(-1) and returns a vector
  that corresponds to those lines (back, left, front, right) needs a 2D costmap
  to be stored
  */
  std::vector<int> CollisionRecovery::createFootprintCollision(
    std::vector<geometry_msgs::Point>& footprint)
  {
    if(footprint.size() == 4)
    {
      if(lines_ == true)
      {
        // check back line for collision
        footprint_collision_.at(0) = lineInCollision(footprint.at(0), footprint.at(1));

        // check left line for collision
        footprint_collision_.at(1) = lineInCollision(footprint.at(1), footprint.at(2));

        // check front line for collision
        footprint_collision_.at(2) = lineInCollision(footprint.at(2), footprint.at(3));

        // check right line for collision
        footprint_collision_.at(3) = lineInCollision(footprint.at(3), footprint.at(0));

      }
      else
      {
        // check back right point for collision
        footprint_collision_.at(0) = pointInCollision(footprint.at(0).x, footprint.at(0).y);

        // check back left point for collision
        footprint_collision_.at(1) = pointInCollision(footprint.at(1).x, footprint.at(1).y);

        // check front line for collision
        footprint_collision_.at(2) = pointInCollision(footprint.at(2).x, footprint.at(2).y);

        // check right line for collision
        footprint_collision_.at(3) = pointInCollision(footprint.at(3).x, footprint.at(3).y);
      }

    }
    else
    {
      ROS_ERROR("[collision_recovery] Footprint not a rectangle!");
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
