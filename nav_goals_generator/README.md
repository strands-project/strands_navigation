nav_goals_generator
===================

ROS Service to generate 2D navigation goals with orientation in a specifed
region of interest (ROI). The service takes the number of navigation goals
(*n*), an *inflation radius* which ressembles the robot's footprint, and a ROI
described by a polygon as arguments and returns a list of goal poses.


# Usage

First make sure that an occupancy grid map (nav_msgs/OccupancyGrid) is
published on a topic, e.g. `/map`. Second, launch the service as follows:

```
roslaunch nav_goal_generation nav_goal_generation.launch
```
The default map is `/map`. It can be overwritten by passing an argument as follows:
```
roslaunch nav_goal_generation nav_goal_generation.launch map:=/projected_map
```
If the map argument is a costmap, you should also set the flag `is_costmap` to `true`. Then the inflation radius in the service call is ignored (a costmap is already inflated)
```
roslaunch nav_goal_generation nav_goal_generation.launch map:=/move_base/global_costmap/costmap  is_costmap:=true
```

You can send a service request as follows:

```
rosservice call /nav_goals '{n: 100, inflation_radius: 0.5, roi: {points: [[0,0,0],[2,-2,0],[-2,-2,0]]}}'
```

whereby the first argument is the number of goal loactions to be generated
(here 100) and and the second argument is the inflation radius of the robot's
footprint (here 0.5), and the third argument is a ROI specified as a list of
points (at least three).  The result of the pose generation is additionally
published on the topic `/nav_goals` in order to visualize the result in RVIZ.

If the service is called with an empty ROI, the full map is considered as ROI
by default. 

```
rosservice call /nav_goals '{n: 100, inflation_radius: 0.5, roi: {}}'
```

If a specified ROI includes a point that is outside the map, its *conflicting*
coordinates are automatically adjusted to the map's bounding box.

# Known Issues

The service fails if there is no map topic available or no message has been
published on this topic after the service has been started. As a workaround,
the map server could be started after this service.
 





