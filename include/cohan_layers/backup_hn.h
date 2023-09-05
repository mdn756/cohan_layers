#ifndef HUMAN_NODE_H
#define HUMAN_NODE_H

#include "cohan_layers/human_layer.h"

namespace cohan_layers
{

class HumanNode : public rclcpp::Node
{
  public:
  HumanNode();

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr agents_sub_ ;
  void agentsCB_test(const geometry_msgs::msg::PoseArray& agents);

  private: 
    HumanLayer human_layer_;//=nullptr;
};


} //namespace cohan_layers

#endif
