#include "cohan_layers/human_node.h"
#include "cohan_layers/human_layer.h"

#define TRACKED_AGENT_SUB "/tracked_agents"

namespace cohan_layers
{
HumanNode::HumanNode()
: Node("human_layer_node")
{

  //human_layer_ = new HumanLayer();
  agents_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(TRACKED_AGENT_SUB, 1, std::bind(&HumanNode::agentsCB_test, this, std::placeholders::_1)); 
}

void HumanNode::agentsCB_test(const geometry_msgs::msg::PoseArray& agents){
  human_layer_.agentsCB(agents);
}

} //namespace cohan_layers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cohan_layers::HumanNode>());
    rclcpp::shutdown();
    return 0;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cohan_layers::HumanNode, rclcpp::Node)

