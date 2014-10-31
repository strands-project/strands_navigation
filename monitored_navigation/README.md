# The monitored navigation package
           

This package provides an actionlib server that executes, monitors and recovers from failure (possibly by asking for human help) a continuous navigation actionlib server, e.g., `move_base`. 
It receives a goal in the form:
    
```
    string action_server
    geometry_msgs/PoseStamped target_pose
```
    
where `action_server` is an actionlib server that receives a  `geometry_msgs/PoseStamped goal` (as `move_base`).

To launch the monitored_navigation with the default STRANDS recovery behaviours do:

```
roslaunch monitored_navigation monitored_nav.launch config_file:="path to strands_navigation"/monitored_navigation/config/strands.yaml
```