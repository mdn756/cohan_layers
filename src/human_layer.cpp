#include "cohan_layers/human_layer.h"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
// what are these includes?

#define TRACKED_AGENT_SUB "/tracked_agents"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cohan_layers
{

HumanLayer::HumanLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
  layered_costmap_ = NULL;
}
void HumanLayer::onInitialize()
{
  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  
  std::string topics_string; //topics that we subscribe to. 

  declareParameter("enabled", rclcpp::ParameterValue(true));
  //declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  node->get_parameter(name_ + "." + "enabled", enabled_); //get and set
  //node->get_parameter(name_ + "." + "observation_sources", topics_string);

  //std::string topic;

  //declareParameter(topics_string + "." + "topic", rclcpp::ParameterValue(topics_string));
  //node->get_parameter(name_ + "." + topics_string + "." + "topic", topic);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  //custom_qos_profile.depth = 50; //what is this
  
  //auto sub = std::make_shared<rclcpp::Subscription<geometry_msgs::msg::PoseArray>>(node, topic, custom_qos_profile, sub_opt);
  //sub->unsubscribe();
  //sub->registerCallback(std::bind(&HumanLayer::agentsCB, this, std::placeholders::_1));
  //observation_subscribers_.push_back(sub);

  agent_sub = node->create_subscription<geometry_msgs::msg::PoseArray>("/agents", 10, std::bind(&HumanLayer::agentsCB, this, std::placeholders::_1), sub_opt); //change 10

  need_recalculation_ = false;
  current_ = true;
  //RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4); this DEBUG thing makes the whole costmap fail. Why?????
  
}

void HumanLayer::agentsCB(const geometry_msgs::msg::PoseArray& agents)
{
  //boost::recursive_mutex::scoped_lock lock(lock_);
  agents_ = agents;
}

void HumanLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  //std::string global_frame = layered_costmap_->getGlobalFrameID();

  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void HumanLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "HumanLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void HumanLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  
  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution(); //size of a pixel basically
  if (!agents_.poses.empty()) //agent message has been passed
  {
    double agent_x = agents_.poses[0].position.x;
    double agent_y = agents_.poses[0].position.y;
    int map_x, map_y;
    double radius_ = 1.5;
    double amplitude_ = 150.0;
    double ox = agent_x - radius_, oy = agent_y - radius_;
    costmap->worldToMapNoBounds(ox, oy, map_x, map_y); //gets pixel location of xy pos methinks 

    unsigned int width = std::max(1,static_cast<int>((2*radius_) / res)); 
    unsigned int height = std::max(1,static_cast<int>((2*radius_) / res)); 

    
    int start_x = 0, start_y = 0, end_x = width, end_y = height; 
    if (map_x < 0)
      start_x = -map_x;
    else if (map_x + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - map_x);
    if (static_cast<int>(start_x + map_x) < min_i)
      start_x = min_i - map_x;
    if (static_cast<int>(end_x + map_x) > max_i)
      end_x = max_i - map_x;  
    if (map_y < 0)
      start_y = -map_y;
    else if (map_y + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - map_y);
    if (static_cast<int>(start_y + map_y) < min_j)
      start_y = min_j - map_y;
    if (static_cast<int>(end_y + map_y) > max_j)
      end_y = max_j - map_y;

    double bx = ox + res / 2,
           by = oy + res / 2; //i think taking center of origin pixel. so back to real units

    double var = radius_;
    

    
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + map_x, j + map_y);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION)
          continue;
 
        double x = bx + i * res, y = by + j * res;
        double val;
        val = Gaussian2D(x, y, agent_x, agent_y, amplitude_, var, var);
        double rad = sqrt(-2*var*log(val/amplitude_));

        if (rad > radius_)
          continue;
        unsigned char cvalue = (unsigned char) val;
        costmap->setCost(i + map_x, j + map_y, std::max(cvalue, old_cost));
      }
    }
    
  }
  else{
  
    /*int center_x = (max_i - min_i)/2 + min_i;
    int center_y = (max_j - min_j)/2 + min_j;
    double radius = (max_i - min_i)/3;
    for (int j = min_j; j < max_j; j++) {
      // Reset gradient_index each time when reaching the end of re-calculated window
      // by OY axis.
      for (int i = min_i; i < max_i; i++) {
        //factor 0-1 and based on distance, else 0
        int dx = (i-center_x);
        int dy = (j-center_y);
        double distance = sqrt(dx*dx + dy*dy);

        int index = master_grid.getIndex(i, j);
      
        //unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
        unsigned char cost = 0;
        if (distance < radius)
        {
          int factor = (LETHAL_OBSTACLE*(1-distance/radius));
          cost = factor%255;
        }
        else 
        {
          cost = 0;
        }
        master_array[index] = cost;
      }
    }
  
    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        //factor 0-1 and based on distance, else 0
        int dx = (i-center_x);
        int dy = (j-center_y);
        double distance = sqrt(dx*dx + dy*dy);

        int index = master_grid.getIndex(i, j);
      
        unsigned char cost = 0;
        if (!agents_.poses.empty())
        {
          int factor = (LETHAL_OBSTACLE*(1-distance/radius));
          cost = factor%255;
          cost = 150;
        }
        else 
        {
          cost = 40;
        }
        master_array[index] = cost;
      }
    }*/
  }
  }

} // namespace cohan_layers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cohan_layers::HumanLayer, nav2_costmap_2d::Layer)
