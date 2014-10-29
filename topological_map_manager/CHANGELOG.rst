^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_map_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adding install targets
* including visualization_msgs in package xml to sort `#83 <https://github.com/strands-project/strands_navigation/issues/83>`_
* Adding Missing TopologicalMap.msg and changing maintainer emails, names and Licences for Packages
* Fixes bugs created by name changes of mongodb_store and moving packages between repositories
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
  Conflicts:
  topological_navigation/scripts/localisation.py
  topological_navigation/scripts/navigation.py
* scitos_apps_msgs has been removed.
  All the imports were unused anyway.
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to be merged first.
* bug fix
* Adding add Node controller
* Bug fix
* Adapting Interactive Markers on Topological Map Manager to use the topological Map Publisher
  and bug fixes.
  *WARNING: Still requires a lot of testing*
* republishing map when modified
* Merge branch 'hydro-devel' of https://github.com/Jailander/strands_navigation into hydro-devel
  Conflicts:
  topological_map_manager/src/topological_map_manager/policies.py
* adding topological map publisher and adapting localisation node to use it
* adding scripts to topological utils
* making arrows blacker
* Making Arrow less thick
* fix fix fix
* Fixing super stupid error
* tests
* test
* trying to fix bug
* bug fix test
* adding policies markers
* making bigest colour bigger than maxval
* Making sure no markers are published whilst updating
* bug fix
* Changing Nodes to Sphenes and normalization
* normalizing values
* Changing Colours
* increasing length of arrows
* moving arrows to the waypoint
* bug fix
* trying bug fix
* Commiting edge array for statistics
* Adding Topological_map_manager
* Contributors: Bruno Lacerda, Christian Dondrup, Jaime Pulido Fentanes, Nick Hawes
