^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica_collision_scene_fcl_latest
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.1 (2021-04-05)
------------------

6.1.0 (2021-03-15)
------------------

6.0.2 (2020-11-23)
------------------

6.0.1 (2020-11-17)
------------------

6.0.0 (2020-11-08)
------------------
* Remove usage of exotica_collision_scene_fcl; add backwards compatibility using exotica_collision_scene_fcl_latest (`#726 <https://github.com/ipab-slmc/exotica/issues/726>`_)
* Fix transform bug fix, remove usage of tf_conversions (`#723 <https://github.com/ipab-slmc/exotica/issues/723>`_)
* Merge remote-tracking branch 'origin/master'
* Return after first ACM entry is found
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* Move to Initializer initialisation of collision scenes
* Check whether transform contains NaNs (translation)
* CollisionScene: Store whether update of objects is required
* CollisionScene: Add setter/getter for replace_cylinder_with_capsule, do not store duplicate company of world links to be excluded from collision scene
* Contributors: Vladimir Ivan, Wolfgang Merkt

5.1.3 (2020-02-13)
------------------
* Refactor CollisionScene, add faster distance checks, speedup SmoothCollisionDistance (`#688 <https://github.com/ipab-slmc/exotica/issues/688>`_)
* Contributors: Wolfgang Merkt

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* Address -Wall warnings (`#679 <https://github.com/ipab-slmc/exotica/issues/679>`_)
* Update from research branch (`#660 <https://github.com/ipab-slmc/exotica/issues/660>`_)
* Address catkin lint (`#549 <https://github.com/ipab-slmc/exotica/issues/549>`_)
* Add world-links-to-exclude (`#525 <https://github.com/ipab-slmc/exotica/issues/525>`_)
* Check finiteness of contact_pos in DEBUG mode (`#514 <https://github.com/ipab-slmc/exotica/issues/514>`_)
* Contributors: Christian Rauch, Vladimir Ivan, Wolfgang Merkt
