^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.1 (2021-04-05)
------------------
* [exotica] Update version to 6.1.1 for documentation
* Contributors: Wolfgang Merkt

6.1.0 (2021-03-15)
------------------
* [exotica] Adjust version for documentation
* [exotica] Update installation.rst instructions
* Upgrade clang format from v3.9 to v6.0 (`#730 <https://github.com/ipab-slmc/exotica/issues/730>`_)
* Contributors: Wolfgang Merkt

6.0.2 (2020-11-23)
------------------

6.0.1 (2020-11-17)
------------------
* Minor updates to documentation
* Contributors: Wolfgang Merkt

6.0.0 (2020-11-08)
------------------
* Remove usage of exotica_collision_scene_fcl
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* ROS Noetic/Python3 compatibility
* Make CollisionScene Instantiable
  - Makes CollisionScene instantiable with an Initializer
  - Moves CollisionScene related properties from SceneInitializer to
  CollisionSceneInitializer
  - Adds ability to _not\_ load any default CollisionScene. E.g., when not
  considering collision avoidance. This makes exotica_core
  runtime-independent of exotica_collision-scene_fcl and also allows for
  faster start.
  - Adds property of a robot link replacement config to CollisionScene
* Move ROS Indigo (14.04) instructions to separate file
* Added visualisation docs
* Contributors: Vladimir Ivan, Wolfgang Merkt

5.1.3 (2020-02-13)
------------------

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* Update documentation
* Contributors: Chris Mower, Christian Rauch, Vladimir Ivan, Wolfgang Merkt, the-raspberry-pi-guy
