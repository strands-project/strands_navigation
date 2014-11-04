# The monitored navigation package
           

This package provides an actionlib server that executes, monitors and recovers from failure (possibly by asking for human help) a continuous navigation actionlib server, e.g., `move_base`. 
It receives a goal in the form:
    
```
    string action_server
```

```
    geometry_msgs/PoseStamped target_pose
```
    
where `action_server` is an actionlib server that receives a  `geometry_msgs/PoseStamped goal` (as `move_base`).

To launch the monitored_navigation with the default STRANDS recovery behaviours do:

```
roslaunch monitored_navigation strands_monitored_nav.launch
```

## Configuring monitors and recovery behaviours

The ``monitored_navigation`` package provides a skeleton state machine where one can add smach implementations of monitor and recovery behaviours. These monitors and recoveries should be subclasses  of the 
[``MonitorState``](src/monitored_navigation/monitor_state.py) and [``RecoverStateMachine``](src/monitored_navigation/recover_state_machine.py) classes respectively. A monitor runs in parallel with the execution of 
the continuous navigation action. When it outputs 'invalid', the continuous navigation, and any other monitors that are running, are preempted, and the state machine jumps to the corresponding recovery behaviour. Examples of STRANDS specific behaviours can be found
 [here](https://github.com/strands-project/strands_recovery_behaviours/tree/hydro-devel/strands_recovery_behaviours/src/strands_recovery_behaviours).

### At initialization

A yaml file that defines the monitor and recovery behaviours that should be added to the state machine at startup can be provided by doing

```
rosrun monitored_navigation monitored_nav.py path_to_yaml_file
```

The yaml file specifies the ``nav_recovery`` mechanism and the ``monitor_recovery_pairs``. 

* The ``nav_recovery`` is specified by:
    * ``package:`` The package where the recovery behaviour can be found
    * ``recovery_file:`` The file where the ``RecoverStateMachine`` subclass implementation can be found (without the .py extenstion)
    * ``recovery_class:`` The name of the recovery class. This should be a subclass of ``RecoverStateMachine``
* The ``monitor_recovery_pairs`` is a list where each element is specified by:
    * ``name:`` The name to be used as an indentifier of the monitor/recovery pair
    * ``package:`` The package where the monitor and recovery behaviour can be found
    * ``monitor_file:`` The file where the ``MonitorState`` subclass implementation can be found (without the .py extenstion)
    * ``monitor_class:`` The name of the monitor class. This should be a subclass of ``MonitorState``
    * ``recovery_file:`` The file where the ``RecoverStateMachine`` subclass implementation can be found (without the .py extenstion)
    * ``recovery_class:`` The name of the recovery class. This should be a subclass of ``RecoverStateMachine``

An example yaml file for the STRANDS-specific configuration can be found [here](config/strands.yaml).

### At run-time

Monitors and recovery behaviours can also be added and removed at runtime (only if the monitored navigation action server is *not* running). To do this, the ```monitored_navigation``` node provides the 
following services. The meaning of the request fields in the service definitios are the same as described above for the yaml file:

* ``/monitored_navigation/add_monitor_recovery_pair``. Add a monitor/recovery pair. Service definition is [here](../strands_navigation_msgs/srv/AddMonitorRecoveryPair.srv)
* `` /monitored_navigation/del_monitor_recovery_pair``. Remove a monitor/recovery pair, by name. Service definition is [here](../strands_navigation_msgs/srv/DelMonitorRecoveryPair.srv)
* ``/monitored_navigation/set_monitor_recovery_pairs``. Set a list of monitor/recovery pairs. The ones currently being used are removed. Service definition is 
[here](../strands_navigation_msgs/srv/SetMonitorRecoveryPairs.srv)
* ``/monitored_navigation/set_nav_recovery``. Set the recovery state machine for navigation. The current recovery for navigation is replaced by the new one. Service definition is [here](../strands_navigation_msgs/srv/SetNavRecovery.srv)