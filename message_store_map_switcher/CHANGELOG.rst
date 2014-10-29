^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package message_store_map_switcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2014-10-29)
------------------
* deleting comment
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
  Conflicts:
  message_store_map_switcher/CMakeLists.txt
* removing more mongodb_store_cpp_client references
* removing message_store_cpp_client from message_store_map_switcher
* adding mongodb_store to message_store_map_switcher dependencies
* removing mongodb_store_cpp_client from message_store_map_switcher
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to be merged first.
* Added definition for new cxx features.
* swapping  target link libraries to correct compilation error
* Fixed to use correct query options. Damn untyped language.
* Now using the new updateNamed method to ensure we don't get lots of maps in the database.
* More docs
* Adding basic usage docs.
* Added switching service with return for success/fail. Tested and working on two maps in rviz.
* Added server node.
* Added saving to db.
* Added loading of map from file based on map_server code.
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Nick Hawes
