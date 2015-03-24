## Topological loggind based on a white list of nodes

Due to privacy protection we might not be allowed to record certain data in specific
areas. Like the east wing of AAF for example. This package contains a node, that
uses the topologocal map information to trigger logging of sensitive data.

### Usage

The logging is based on a so-called white list of nodes, e.g. this AAF example:

```
nodes:
    - "Eingang"
    - "Rezeption"
    - "WayPoint2"
    - "WayPoint17"
    - "WayPoint35"
```

This file is in YAML format. The node has to be started with `_white_list_file:=<file_name>`.

It monitores the topics for the current edge and current node. Whenever the robot
is at a node that is in the white list, the script publishes `true`, `false` otherwise.
In addition, it uses the look-up service of topological navigation, to get the
edges between these nodes. When the robot is on that edge, it also publishes
`true` and `false` otherwise. Publishing rate is 30hz, can be changed with the `_publishing_rate`
parameter.

To allow logging on all nodes and edges use:

```
nodes:
    - "ALL"
```

which you can find in the `conf` dir.

Published are a bool and a "stamped bool" which is the custom LoggingManager.msg
containing a header and a bool. The stamped bool is useful in combination with
the ApproximateTimeSynchronizer. Let's say you want to log camera images:

```
import rospy
import message_filters
from topological_logging_manager.msg import LoggingManager
from sensor_msgs.msg import Image
...

class SaveImages():
    def __init__(self):
        ...

        subs = [
            message_filters.Subscriber(rospy.get_param("~rgb", "/head_xtion/rgb/image_rect_color"), Image),
            message_filters.Subscriber(rospy.get_param("~logging_manager", "/logging_manager/log_stamped"), LoggingManager),
        ]

        ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.1)
        ts.registerCallback(self.cb)
...

    def cb(self, img, log):
        if log:
            ...
...
```


If the message is not published, the callback will not be triggered, if it publishes,
you can check if recording is true or false.

The normal bool can be used as a flag for everything else.

### Parameters

* `white_list_file`: the yaml file containing the node names
* `current_node_topic` _default="/current_node_: The topic where the current node is published
* `closest_node_topic` _default="/closest_node_: The topic where the closest node is published
* `use_closest_node` _default="true"_: If true uses the closest node to determine if true or false. If false, only being in the influence zone of the node will give true.
* `edge_topic` _default="/current_edge_: The topic where the current edge is published
* `check_topic_rate` _default="1"_: The rate at which the edge and node topics are checked if they are still alive.
* `publishing_rate` _default="30"_: Rate at which the result is published. 30 to match camera output.
* `bool_publisher_topic` _default="/logging_manager/log: The topic n which the std_msgs/Bool is published
* `bool_stamped_publisher_topic` _default="/logging_manager/log_stamped_: The topic n which the topological_logging_manager/LoggingManager is published


### Security

To ensure that this works as reliable as possible, the default should always be not to record.
As mentioned above, using the "stamped bool" acheives this quite easily. To ensure
that the node always publishes the correct values, it has topic monitores for
the two used topics that check if they are still alive, every second. The results
are published in a different thread because the edges and nodes are only published when they change.
The monitores are therefore used to detect if the navigation died.
