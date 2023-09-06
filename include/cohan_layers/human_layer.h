#ifndef HUMAN_LAYER_H
#define HUMAN_LAYER_H

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace cohan_layers
{

class HumanLayer : public nav2_costmap_2d::Layer
{
  public: 
  HumanLayer();
  
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();
  
  virtual bool isClearable() {return false;}

  void agentsCB(const geometry_msgs::msg::PoseArray& agents);

protected:
  struct AgentPoseVel{
    std_msgs::msg::Header header;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
  };

  std::vector<AgentPoseVel> transformed_agents_;

  bool first_time_, /*reset,*/ shutdown_;
  rclcpp::Time last_time;
  geometry_msgs::msg::PoseArray agents_;
  double radius_, amplitude_, covar_, cutoff_;
  double robot_radius = 0.46, human_radius=0.31;

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recalculation_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr agent_sub;

  int GRADIENT_SIZE = 20;
  int GRADIENT_FACTOR = 10;
};
} // namespace cohan_layers

#endif
