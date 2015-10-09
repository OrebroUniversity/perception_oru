^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ndt_visualisation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.30 (2015-10-09)
-------------------

1.0.29 (2015-10-08)
-------------------
* Fixed transform error in the orbit camera.
* Added check to avoid duplicate drawing objects.
* Added new functionalities to the visualization class.
*removed dependency on mrpt
* Contributors: Henrik Andreasson, Todor Stoyanov

1.0.28 (2014-12-05)
-------------------
* added opengl and image_transport as deps
* Contributors: Todor Stoyanov

1.0.27 (2014-12-05)
-------------------
* fixes to package.xml files
* Contributors: Todor Stoyanov

1.0.26 (2014-12-03)
-------------------
* small fix to package.xmls and an update of visualization
* Contributors: Todor Stoyanov

1.0.25 (2014-12-01)
-------------------

1.0.24 (2014-11-25)
-------------------

1.0.23 (2014-11-25)
-------------------

1.0.20 (2014-11-21)
-------------------
* removing mrpt dependency
* Added NDTViz into a thread.
* Added the glut NDT visualizer.
* Contributors: Henrik Andreasson, Martin Magnusson, RVC, Todor Stoyanov, Tomasz Kucner, Tomasz Kuncer

1.0.18 (2014-04-09)
-------------------

1.0.15 (2014-01-09)
-------------------
* fixes to makefiles wrt pcl
* Contributors: Todor Stoyanov

1.0.13 (2014-01-09)
-------------------
* fixing mess with pcl compatibility
* Contributors: Todor Stoyanov

1.0.12 (2013-12-03)
-------------------

1.0.10 (2013-12-03)
-------------------
* backward compatible CMyellipsoid
* Contributors: Todor Stoyanov

1.0.9 (2013-12-02)
------------------
* removed deprecated dependancy on mrpt-graphslam
* hydro release
* Contributors: Todor Stoyanov

1.0.8 (2013-12-02)
------------------
* "1.0.8"
* changelogs updated
* Removed legacy dependency to mrpt-graphslam
* Contributors: Todor Stoyanov

1.0.7 (2013-11-28)
------------------
* "1.0.7"
* changelogs update
* Added release flags to all CMake files
* Re-organization of include files to follow ros convention, lots of changes
* Contributors: Todor Stoyanov

1.0.6 (2013-11-27 20:13)
------------------------
* "1.0.6"
* Contributors: Todor Stoyanov

1.0.5 (2013-11-27 19:52)
------------------------
* "1.0.5"
* now glut xmu issue, this will never end
* Contributors: Todor Stoyanov

1.0.4 (2013-11-27 19:40)
------------------------
* "1.0.4"
* trying to add glut as well, situation is not good
* Contributors: Todor Stoyanov

1.0.3 (2013-11-27 19:26)
------------------------
* "1.0.3"
* prepairing for second release candidate
* rosdeps are now handled through catkin, hopefully mrpt as well
* CMake files fixed to output in the correct place
* Contributors: Todor Stoyanov

1.0.2 (2013-11-27 13:58)
------------------------
* "1.0.2"
* Contributors: Todor Stoyanov

1.0.1 (2013-11-27 12:33)
------------------------
* "1.0.1"
* added changelog files to stream
* removed message gen that was not needed and generating scary warnings
* remove depreciated dependencies
* the removal of rosbuild remains
* compiled packages ndt_fuser  ndt_map  ndt_map_builder  ndt_mcl  ndt_registration  ndt_visualisation  perception_oru  pointcloud_vrml
* Contributors: Todor Stoyanov, Tomasz Kuncer
