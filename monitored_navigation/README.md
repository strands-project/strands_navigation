# The monitored navigation package
           

This package provides an actionlib server that executes, monitors and recovers from failure (possibly by asking for human help) a continuous navigation actionlib server, e.g., `move_base`. It receives a goal in the form:
    

    
    string action_server
    geometry_msgs/PoseStamped target_pose

    
where `action_server` is an actionlib server that receives a  `geometry_msgs/PoseStamped goal` (as `move_base`). It then executes it, with the same monitor and recovery behaviours as the old long-term patroller system. To test the monitored navigation with `move_base`:
        
    
1. launch the scitos_bringup or morse:
1. launch the STRANDS `move_base` action server:
    
    ```
    roslaunch scitos_2d_navigation scitos_2d_nav.launch map:="your map"
   ```
   
1. launch the monitored navigation action server:
    
    `roslaunch monitored_navigation monitored_nav.launch`
    
1. launch an actionlib client gui:
    
    ```
    rosrun actionlib axclient.py /monitored_navigation
    ```
    
1. fill the goal (in this case `action_server='move_base'`)