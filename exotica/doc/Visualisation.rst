*************
Visualisation
*************
You can visualise the robot states, trajectories, or even intermediate frames and debugging aids using EXOTica. We provide several ways how to achieve this.

Meshcat Visualiser
==================
Meshcat is a robot visualisation tool that uses a web browser and WebGL for rendering. The recommended way to use Meshcat with EXOTica is to use the `meshcat-python <https://github.com/rdeits/meshcat-python>`_ ZMQ server and the `meshcat-ros-fileserver <https://github.com/VladimirIvan/meshcat_ros_fileserver>`_ file server. Install both of these python packages from source (e.g. by running ``python setup.py install --user``)

Then open two terminals and in the first one start the ZMQ server:

.. code-block:: bash

    meshcat-server

In the secont terminal start the file server:

.. code-block:: bash

    meshcat-ros-fileserver -f /

Once the servers are running, you can connect to it from exotica:

.. code-block:: python

    vis = exo.VisualizationMeshcat(scene, 'tcp://127.0.0.1:6000')
    vis.delete() # Clear existing scene
    vis.display_scene() # Display this scene

You can now open a web browser at ``http://127.0.0.1:7000/static/`` (don't forget the ``/static/`` at the end). You can get this URL by running ``vis.get_web_url()``. Alternatively, if you are using jupyter, you can display the visualisation directly in your nodebook by running:

.. code-block:: python

    exo.jupyter_meshcat.show(vis.get_web_url())

You can now display robot states, trajectories and change the scene settings:

.. code-block:: python

    # Show state at time t
    vis.display_state(state, t)
    # Display a trajectory
    vis.display_trajectory(trajectory, dt)

    # Change scene settings
    vis.set_property('/Lights/DirectionalLight','visible', False)
    vis.set_property('/Lights/FillLight','visible', False)
    vis.set_property('/Lights/SpotLight','visible', True)
    vis.set_property('/Lights/SpotLight/<object>','castShadow', True)
    vis.set_property('/Grid','visible', False)
    vis.set_property('/Axes','visible', False)

If you change the underlying scene the visualiser is using (e.g., by loading scene meshes using ``scene.load_scene_file()``), you'll have to refresh the visualiser by running:

.. code-block:: python

    vis.display_scene()
    
MoveIt Visualiser
==================

The MoveIt visualiser publishes a trajectory in the MoveIt trajectory message format for display in RViz.

To set up the visualiser run:

.. code-block:: python

    vis = exo.VisualizationMoveIt(scene)

To visualise the trajectory run:

.. code-block:: python

    vis = exo.VisualizationMoveIt(scene)
    vis.display_trajectory(trajectory)

The message will be published at ``<scene_name>/Trajectory`` and the trajectory can be visualised in RViz using the trajectory display plugin.

Debugging using RViz
====================
You may sometimes need to display the state of the robot during planning iterations, collision shapes, debugging markers or just frames defined in exotica.
The scene and task maps can be switched into debug mode in which they publish several useful visual aids to RViz. To enable the debug mode on the scene or on a task maps add the ``Debug="1"`` argument to its tag.

When in debug mode, the scene will automatically publish all defined frames as TF frames with the ``exotica`` prefix. This will happen every time the scene updates. You can use these in RViz to: display the robot model, visualise TFs, attch marker to exotica frames. All TF transforms are attached to the EXOTica wolrd frame (as defined/named in the SRDF). There is no hierarchy among the TFs to allow displaying the kinematics computed in EXOTica.

Publishing all frames at at every update may slow down your solvers. Instead, you can publish the TFs manually by running:

.. code-block:: python

    scene.get_kinematic_tree().publish_frames()

Some of the task maps also publish Marker and MarkerArray messages with useful visual aids for debugging. All of these attach the marker to a TF published by the scene (including the ``exotica`` prefix) and set the marker to update its position automatically when the TF moves. This means that the task map markers will also update only when the scene publishes the TF transforms. If the marker shape/type doesn't need changing, the overhead of displaying it is on RViz, therefore the only overhead is the publishing of TFs by the scene.
