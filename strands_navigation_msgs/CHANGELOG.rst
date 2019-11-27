^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_navigation_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2019-11-27)
------------------

1.0.8 (2019-06-04)
------------------
* Merge pull request `#374 <https://github.com/strands-project/strands_navigation/issues/374>`_ from Jailander/edge-reconf
  Move base parameters being reconfigured at edges
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
* Moving reconf server to strands
* Merge pull request `#369 <https://github.com/strands-project/strands_navigation/issues/369>`_ from strands-project/ori-indigo-devel
  Support for multi-robot and different global planners
* Merge remote-tracking branch 'ori/indigo-devel' into indigo-devel
  Bringing in changes from ORI for multi-robot and different base planners.
* correct feedback publishing from topo nav
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Nick Hawes

1.0.7 (2018-10-26)
------------------

1.0.6 (2018-07-17)
------------------

1.0.5 (2018-04-17)
------------------
* Merge pull request `#351 <https://github.com/strands-project/strands_navigation/issues/351>`_ from heuristicus/indigo-devel
  Can now place nodes with RMB to stop automatic edge creation
* Can now place nodes with RMB to stop automatic edge creation
  Fix deletion dialogue, edges and tags were swapped
* Contributors: Michal Staniaszek, Nick Hawes

1.0.4 (2017-06-23)
------------------
* Modifications to topological map tools to accommodate topological map editor (`#345 <https://github.com/strands-project/strands_navigation/issues/345>`_)
  * fix weird space-colon
  * Easier translational movement of waypoints, generic node field updater
  Moving the waypoints that are displayed in the topological map in rviz is now
  easier - just uses 2D planar motion as opposed to multiple handles for the x and
  y dimensions.
  Added a function which calls into the database to update any property of a node.
  * Fixed not loading map after update, correctly updates edges on node rename
  This should really not be the file being used - it seems like the one in util is
  used to change things and as such is more up to date.
  * remove unnecessary if
  * update function for edge action and top_vel
  * add deprecation warnings to topological_map.py - should use manager.py instead
  * start on work to make manager services more useful for modifying map
  * add callback for getting tags for a specific node
  * partial switch to the using manager, updating and adding tags
  * fix message fields and add messages to generation
  * small script to insert empty map into a database
  * add edge removal service
  * change callbacks so that functions can be called without service
* Contributors: Michal Staniaszek

1.0.3 (2017-01-11)
------------------

1.0.2 (2016-10-31)
------------------

1.0.1 (2016-06-21)
------------------

1.0.0 (2016-06-09)
------------------

0.0.45 (2016-06-06)
-------------------

0.0.44 (2016-05-30)
-------------------

0.0.43 (2016-05-25)
-------------------
* 0.0.42
* updated changelogs
* 0.0.41
* updated changelogs
* Adding localise_pose service which returns the node and closest node for a pose.
  And fixing conflicts
* Contributors: Jaime Pulido Fentanes, Jenkins

0.0.42 (2016-03-21)
-------------------

0.0.41 (2016-03-03)
-------------------

0.0.40 (2016-02-07)
-------------------

0.0.39 (2016-01-28)
-------------------
* Impossible tests now require the navigation to fail on its own accord
  Currently, the impossible tests, i.e., blocking the way or the final node, require that the graceful death attempt is successful, meaning that the robot is able to navigate back to start after the navigation to end failed. With this PR, a new field for the service is added, giving feedback if the navigation timed out or if it failed on its own accord. Impossible tests are therefore only passed, if the navigation failed without timing out and if graceful death was successful.
* Contributors: Christian Dondrup

0.0.38 (2015-11-17)
-------------------
* Adding graceful_fail results to run service
* Adding load an run service for topo nav testing
* Revert "Adding first version of topological test scenarios"
* Adding graceful_fail results to run service
* Adding load an run service for topo nav testing
* adding simple policy generation based on A*
* creating move base testing branch
* Contributors: Christian Dondrup, Jaime Pulido Fentanes

0.0.37 (2015-08-26)
-------------------
* Added window range to action message. If this is left blank in the goal the behaviour is as before
* Contributors: Nick Hawes

0.0.36 (2015-05-17)
-------------------

0.0.35 (2015-05-10)
-------------------

0.0.34 (2015-05-05)
-------------------

0.0.32 (2015-04-12)
-------------------

0.0.31 (2015-04-10)
-------------------
* change mon nav events to allow for more failures to be logged
* Adding Services:
  * /topological_map_manager/add_content_to_node: adds content to a node
  * /topological_map_manager/get_tags: get a list of tags to in the current topological map
  * /topological_map_manager/rm_tag_from_node: removes a tag from a list of nodes
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes

0.0.29 (2015-03-23)
-------------------

0.0.28 (2015-03-20)
-------------------

0.0.27 (2015-03-19)
-------------------
* Adding topological map editor launch file,
  replacing map_publisher with map manager,
  adding add_node service
* commiting migrate script plus typo fix
* redefining msgs to include localise by topic and inflation radius and simplifying recovery behaviours definition by setting them in one config string
* bug fixes
* Navigation and policy_executor working with new defs
* turning Message back to one single pose as agreed it will be sorted using the carrot planner
* added services to get current human helpers and recoveries from mon nav
* adding message def to dynamically load python objects to the monitored_nav state machine
* bug fixes
* committing map creation script
* Adding Recovery behaviours to edges full definition
* Adding recovery behaviours to edges
* Merge branch 'topological-testing' of https://github.com/strands-project/strands_navigation into topological-testing
* added services to get current human helpers and recoveries from mon nav
* adding message def to dynamically load python objects to the monitored_nav state machine
* adding Get Topological Map service
* adding changes to messages and creating topological testing branch locally
* removing traversabilty from edge message
* commiting latest versions of messages
* switching NavRoute to new def
* renaming new definitions to old message types
* new branch created
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes

0.0.26 (2015-03-18)
-------------------

0.0.25 (2015-03-18)
-------------------

0.0.24 (2015-03-17)
-------------------
* adding sensor_msgs to package.xml
* adding costmaps to the monitored nav event logging
* Contributors: Bruno Lacerda

0.0.23 (2014-12-17)
-------------------

0.0.22 (2014-11-26)
-------------------

0.0.21 (2014-11-23)
-------------------

0.0.20 (2014-11-21)
-------------------
* fixing typo
* Contributors: Bruno Lacerda

0.0.19 (2014-11-21)
-------------------

0.0.18 (2014-11-21)
-------------------

0.0.17 (2014-11-21)
-------------------

0.0.16 (2014-11-21)
-------------------

0.0.15 (2014-11-19)
-------------------

0.0.14 (2014-11-19)
-------------------

0.0.12 (2014-11-17)
-------------------

0.0.11 (2014-11-14)
-------------------

0.0.10 (2014-11-14)
-------------------
* Adding retries to topological navigation and current edge publisher
* Contributors: Jaime Pulido Fentanes

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-11)
------------------

0.0.6 (2014-11-06)
------------------

0.0.5 (2014-11-05)
------------------
* Adding licences and bug fix
* Removed topological_utils dependency.
* Moved Vertex and Edge into strands_navigation_msgs.
  Basic test for travel_time_tester passes.
* Merge topological_navigation and topological_map_manager packages.
  Added the EstimateTravelTime service to provide a clean way of getting travel times of the topological map.
* added service definitions for adding and removing monitor and help states to the overall monitored nav state machine
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Nick Hawes

0.0.4 (2014-10-30)
------------------

0.0.3 (2014-10-29)
------------------
* moving human_help_manager service definition to human_help_manager package
* Contributors: Bruno Lacerda

0.0.2 (2014-10-29)
------------------
* 0.0.1
* added changelogs
* Removing TopologicalMap.msg as it may create conflicts with other pull requests
* Adding Missing TopologicalMap.msg and changing maintainer emails, names and Licences for Packages
* Moving and renaming Execute Policy Action
* taking out distinction between local and global plan failure
  first steps to make monitored_nav scitos independent
  always oututs after help
  new action definition
  less management of new goals arriving during execution, as it was buggy
* Fixes bugs created by name changes of mongodb_store and moving packages between repositories
* moving strands_navigation_msgs to strands_navigation
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Marc Hanheide
