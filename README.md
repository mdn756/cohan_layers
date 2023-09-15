USE: cohan_layers adds another layer to the costmap- a circle around static humans, or a skewed ellipse in the direction of velocity for dynamic humans. 

STEPS: If the package builds, make sure the costmap plugin is being used in the yaml file. Then, runthe sim, and open rviz. Then, send human pose and velocity data to the corresponding topic. For example, in terminal you can type 

ros2 topic pub --once agents cohan_msgs/msg/DynamicAgents "{header: {frame_id: 'random_frame'}, poses: [{position: {x: -2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 1.0}}], twists: [{linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 1.0, y: 0.0, z: 0.0}}]}"

Then, the costmap on rviz should be modified. 

tunable parameters: 
variance of the gaussians
radius of the gaussian
skew_factor (how much the gaussian is skewed in the direction of velocity)
amplitude (of the costmap values)

TODO: test stream of agent position data
TODO: global costmap or local? both?
