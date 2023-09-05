#ifndef HUMAN_LAYER_H
#define HUMAN_LAYER_H

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

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

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recalculation_;
  int GRADIENT_SIZE = 20;
  int GRADIENT_FACTOR = 10;
};
} // namespace cohan_layers

#endif
