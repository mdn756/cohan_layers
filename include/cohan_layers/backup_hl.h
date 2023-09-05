#ifndef HUMAN_LAYER_H
#define HUMAN_LAYER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
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
  ~HumanLayer();
  
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

  virtual bool isClearable() {return false;}

  void agentsCB(const geometry_msgs::msg::PoseArray& agents);

protected:
  struct AgentPoseVel{
    std_msgs::msg::Header header;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
  };  
  

  //bool shutdownCB(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse &res);

  double Guassian1D(double x, double x0, double A, double varx){
    double dx = x-x0;
    return A*exp(-pow(dx,2.0)/(2.0*varx));
  }

  double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary)
  {
    double dx = x - x0, dy = y - y0; 
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta), Y = d*sin(theta);
    return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang)
  {
    double dx = x - x0, dy = y - y0; 
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta-skew_ang), Y = d*sin(theta-skew_ang);
    return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double getRadius(double cutoff, double A, double var)
  {
    return sqrt(-2 * var * log(cutoff / A));
  }

  //cohan_msgs::msg::TrackedAgents agents_;
  std::vector<AgentPoseVel> transformed_agents_;

  bool first_time_, /*reset,*/ shutdown_;
  rclcpp::Time last_time;
  geometry_msgs::msg::PoseArray agents_;
  double radius_, amplitude_, covar_, cutoff_;
  double robot_radius = 0.46, human_radius=0.31;
private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recalculation_;
};

} // namespace cohan_layers

#endif