^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.1 (2015-02-27)
------------------

2.3.0 (2014-12-01)
------------------
* Update contact manager for Gazebo 2.0
* add catkin_ignore due to the compile error. remove this when it is ready go
* -Fix instantaneous odom getting computed, but not published (would alway publish 0 before)
* Fix compilation on OS X (at least 10.9)
  - without this, linking fails.
* replace deprecated shared_dynamic_cast (fixes `#9 <https://github.com/turtlebot/turtlebot_create_desktop/issues/9>`_)
* fixes gazebo header paths (refs `#7 <https://github.com/turtlebot/turtlebot_create_desktop/issues/7>`_)
* Contributors: Jihoon Lee, Marcus Liebhardt, Nikolaus Demmel, Samir Benmendil, Stefan Kohlbrecher, trainman419

2.2.0 (2013-08-30)
------------------
* publish odom and joint_states with absolute paths so they don't go under /turtlebot_node/ namespace.
* adds bugtracker and repo info to package.xml

2.1.0 (2013-07-18)
------------------
* Fully catkinized
* Hydro beta release
