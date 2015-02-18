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

To run the bare-bone monitored navigation state machine you can do

```
rosrun monitored_navigation monitored_nav.py
```

## Running monitored_navigation within the STRANDS system


To launch the monitored_navigation with the default STRANDS recovery behaviours and human help interfaces, you need to have [```strands_recovery_behaviours```](https://github.com/strands-project/strands_recovery_behaviours) installed and do:

```
roslaunch strands_recovery_behaviours strands_monitored_nav.launch
```

## Configuring monitors and recovery behaviours, and interfaces for asking help from humans

The ``monitored_navigation`` package provides a skeleton state machine where one can add smach implementations of monitor and recovery behaviours. Furthermore, it allows the robot to ask for human help 
via pre-defined interfaces.

* The monitors and recoveries should be subclasses  of the 
[``MonitorState``](src/monitored_navigation/monitor_state.py) and [``RecoverStateMachine``](src/monitored_navigation/recover_state_machine.py) classes respectively. A monitor runs in parallel with the execution of 
the continuous navigation action. When it outputs ``'invalid'``, the continuous navigation, and any other monitors that are running, are preempted, and the state machine jumps to
the corresponding recovery behaviour. Examples of STRANDS specific behaviours can be found
 [here](https://github.com/strands-project/strands_recovery_behaviours/tree/hydro-devel/strands_monitored_nav_states/src/strands_monitored_nav_states).
* The interfaces for asking help should be subclasses of the [``UIHelper``](src/monitored_navigation/ui_helper.py) class. The ``ask_help``, ``being_helped``, ``help_finished`` and ``help_failed`` methods need
to be defined for the specific ui being used. Help is asked by a [``RecoverStateMachine``](src/monitored_navigation/recover_state_machine.py) via a service
call to ```'/monitored_navigation/human_help/'```. Service definition [here](../strands_navigation_msgs/srv/AskHelp.srv). The  ``ask_help``, ``being_helped``, ``help_finished`` and ``help_failed`` methods
receive datafrom the service request, and do something appropriate, depending on the ui they are implementing (say something, send email, ...). Examples of STRANDS specific helpers can be found
 [here](https://github.com/strands-project/strands_recovery_behaviours/tree/hydro-devel/strands_human_help/src/strands_human_help).


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
* The ``human_help`` is a list where each element is specified by:
    * ``package:`` The package where the helper implementation can be found
    * ``helper_file:`` The file where the ``UIHelper`` subclass implementation can be found (without the .py extenstion)
    * ``recovery_class:`` The name of the rhelper class. This should be a subclass of ``UIHelper``

An example yaml file for the STRANDS-specific configuration can be found [here](config/strands.yaml).

### At run-time

Monitors and recovery behaviours can also be added and removed at runtime (only if the monitored navigation action server is *not* running). To do this, the ```monitored_navigation``` node provides the 
following services, based on the  [DynClassLoaderDef](../strands_navigation_msgs/msg/DynClassLoaderDef.msg) message. The meaning of the message fields is the same as described above for the yaml file:

* ``/monitored_navigation/add_monitor_recovery_pair``. Add a monitor/recovery pair. Service definition is [here](../strands_navigation_msgs/srv/AddMonitorRecoveryPair.srv)
* `` /monitored_navigation/del_monitor_recovery_pair``. Remove a monitor/recovery pair, by name. Service definition is [here](../strands_navigation_msgs/srv/DelMonitorRecoveryPair.srv)
* ``/monitored_navigation/set_monitor_recovery_pairs``. Set a list of monitor/recovery pairs. The ones currently being used are removed. Service definition is 
[here](../strands_navigation_msgs/srv/SetMonitorRecoveryPairs.srv)
* ``/monitored_navigation/get_monitor_recovery_pairs``. Get current monitor/recovery pairs. Service definition is 
[here](../strands_navigation_msgs/srv/GetMonitorRecoveryPairs.srv)
* ``/monitored_navigation/set_nav_recovery``. Set the recovery state machine for navigation. The current recovery for navigation is replaced by the new one. Service definition is [here](../strands_navigation_msgs/srv/SetNavRecovery.srv)
* ``/monitored_navigation/get_nav_recovery``. Get the current recovery state machine for navigation. Service definition is [here](../strands_navigation_msgs/srv/GetNavRecovery.srv)
* ``/monitored_navigation/add_helper``. Add a human helper interface. Service definition is [here](../strands_navigation_msgs/srv/AddHelper.srv)
* `` /monitored_navigation/del_helper``. Remove a human helper interface, by name. Service definition is [here](../strands_navigation_msgs/srv/DelHelper.srv)
* ``/monitored_navigation/set_helpers``. Set a list of human helper interfaces. The ones currently being used are removed. Service definition is 
[here](../strands_navigation_msgs/srv/SetHelpers.srv)
* ``/monitored_navigation/get_helpers``. Gett  list of current human helper interfaces. Service definition is 
[here](../strands_navigation_msgs/srv/GetHelpers.srv)
