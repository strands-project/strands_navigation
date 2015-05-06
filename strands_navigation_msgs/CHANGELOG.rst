^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_navigation_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
