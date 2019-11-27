^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package monitored_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2019-11-27)
------------------

1.0.8 (2019-06-04)
------------------
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
* Merge pull request `#369 <https://github.com/strands-project/strands_navigation/issues/369>`_ from strands-project/ori-indigo-devel
  Support for multi-robot and different global planners
* Merge remote-tracking branch 'ori/indigo-devel' into indigo-devel
  Bringing in changes from ORI for multi-robot and different base planners.
* update of absolute/relative topic names for multi-robot setup
* Contributors: Bruno Lacerda, Nick Hawes

1.0.7 (2018-10-26)
------------------

1.0.6 (2018-07-17)
------------------

1.0.5 (2018-04-17)
------------------

1.0.4 (2017-06-23)
------------------

1.0.3 (2017-01-11)
------------------
* use threads to make sure all helpers run at the same time
* Contributors: Bruno Lacerda

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
* allowing for userdata between monitor and recover states
* 0.0.42
* updated changelogs
* 0.0.41
* updated changelogs
* allowing setting of max bumper recoveries param at startup
* Contributors: Bruno Lacerda, Jenkins

0.0.42 (2016-03-21)
-------------------

0.0.41 (2016-03-03)
-------------------

0.0.40 (2016-02-07)
-------------------

0.0.39 (2016-01-28)
-------------------

0.0.38 (2015-11-17)
-------------------
* Got too carried away with the dependencies... removing the faulty one.
* Adding missing dependencies to monitored navigation package.xml
* Revert "Adding first version of topological test scenarios"
* Got too carried away with the dependencies... removing the faulty one.
* Adding missing dependencies to monitored navigation package.xml
* Contributors: Christian Dondrup, Jaime Pulido Fentanes

0.0.37 (2015-08-26)
-------------------

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
* set n_tries to 1 when it doesnt come from userdata
* RecoverState class to allow enable/disable of certain recoveries on the fly
* extra try/catch statements in the logging
* stop instantiating new classes all the time - making sure services with the same name arent created
* first stuff to allow deactivating recover states
* Contributors: Bruno Lacerda

0.0.29 (2015-03-23)
-------------------

0.0.28 (2015-03-20)
-------------------

0.0.27 (2015-03-19)
-------------------
* Update README.md
* Update README.md
* Update README.md
* edit mon nav read me
* added services to get current human helpers and recoveries from mon nav
* adding message def to dynamically load python objects to the monitored_nav state machine
* Update README.md
* Update README.md
* Update README.md
* edit mon nav read me
* added services to get current human helpers and recoveries from mon nav
* adding message def to dynamically load python objects to the monitored_nav state machine
* Contributors: Bruno Lacerda

0.0.26 (2015-03-18)
-------------------

0.0.25 (2015-03-18)
-------------------

0.0.24 (2015-03-17)
-------------------
* adding launch and config dirs to install targets
* explicit queue size for pub
* Contributors: Bruno Lacerda

0.0.23 (2014-12-17)
-------------------
* publishers now created with an explicit queue_size (indigo doesnt like it otherwise)
* Contributors: Bruno Lacerda

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
* changing logging default back to true (top nav wont log nav recoveries after all)
* making sure everything preempts
* only looking at filtered argv
* Contributors: Bruno Lacerda

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
* replanning when failing
* checking correctness of monitored_navigation argument
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes

0.0.9 (2014-11-12)
------------------
* adding monitored_nav to topological_navigation.launch. default is monitored_nav without recovery behaviours
* Contributors: Bruno Lacerda

0.0.8 (2014-11-11)
------------------

0.0.6 (2014-11-06)
------------------
* add backtrack action server launch to monitored navigation launch
* update strands config to add a monitored nav pause monitor
* Contributors: Bruno Lacerda

0.0.5 (2014-11-05)
------------------
* Adding licences and bug fix
* edited readme
* code cleaning
* created strands-specific launch file
* monitors and recoveries can only be added when action server is not running
  Signed-off-by: Bruno Lacerda <b.lacerda@cs.bham.ac.uk>
* edit readme (to be extended later)
* added service definitions for adding and removing monitor and help states to the overall monitored nav state machine
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
* added strands specific config yaml
* monitor and recovery states are now defined via a config yaml file.
* Merge branch 'target' into hydro-devel
  Conflicts:
  monitored_navigation/CMakeLists.txt
* adding monitored nav launch to targets
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes

0.0.4 (2014-10-30)
------------------

0.0.3 (2014-10-29)
------------------
* adding installation of monitored nav launch file
* edited launch file for new launch structure of the ui's
* Contributors: Bruno Lacerda

0.0.2 (2014-10-29)
------------------
* 0.0.1
* added changelogs
* correcting help manager include
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
  Conflicts:
  message_store_map_switcher/CMakeLists.txt
* making monitored_navigation a general smach based repo that allows user to add specific instantiations of smach monitors and smach recovery behaviours
* Adding Missing TopologicalMap.msg and changing maintainer emails, names and Licences for Packages
* changing param defauls;
  aborting when continuous nav action server does not exist
* taking out distinction between local and global plan failure
  first steps to make monitored_nav scitos independent
  always oututs after help
  new action definition
  less management of new goals arriving during execution, as it was buggy
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to be merged first.
* changing server nave for instrospection
* adding introspection seerver to monitored_navigation
* alternative preemption
* stopped preempting monitored_nav action when help is being offered by human. more edits for proper preemption of continuous nav action
* waiting more time to timeout previous action
* missing logging component
* add logging and making preemption work after recovery
* improving preemption mechanism
* bug fix
* goals are only replaced when the new goal has the same action server name
* sovling time/duration comparisons bug
* disabling backtrack for now
* Adding machine tags to launch files
* Merge branch 'hydro-devel' of https://github.com/BFALacerda/strands_navigation into hydro-devel
* small bug fixes
* monitored navigation now does not cancel move_base when new goal arrives
* use ptu action from scitos_ptu
* Checking for preemption and added a few dependencies for recover states
* monitored navigation now does not ask for help when NavFn fails, as it usually means that the goal pose is blocked by an obstacle
* Added backwards driving behaviour
* adding state to be filled with moving backwards recovery
* - ability to preempt bumper recovery
  - send interaction_service without the prefix
* removed scitos_2d_nav of monitored_nav.launch
* added monitored navigation gui
* code cleaning
* getting preemption to work properly
* making the continuous navigation action server an input to the monitored navigation
* code cleaning
* making human help optional
* adding manager node for human help interfaces - first version
* first version of monitored navigation
* Contributors: BFALacerda, Bob, Bruno Lacerda, Chris Burbridge, Jaime Pulido Fentanes, Lars Kunze, Marc Hanheide, Nick Hawes, Nils Bore, strands
