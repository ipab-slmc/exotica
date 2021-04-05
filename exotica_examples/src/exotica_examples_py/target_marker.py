#!/usr/bin/env python
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Pose, Point, Quaternion
import PyKDL as kdl
import pyexotica as exo

__all__ = ['TargetMarker']

def list_to_pose(p):
    if len(p) == 3:
        return Pose(Point(x = p[0], y = p[1], z = p[2]), Quaternion(x = 0., y = 0., z = 0., w = 1.))
    elif len(p) == 7:
        return Pose(Point(x = p[0], y = p[1], z = p[2]), Quaternion(x = p[3], y = p[4], z = p[5], w = p[6]))
    else:
        raise RuntimeError('Invalid transform!')


class TargetMarker:
    def __init__(self, pose=[0, 0, 0, 0, 0, 0, 1],
                 description='Target',
                 server_name='target_marker',
                 frame_id='exotica/world_frame',
                 marker_shape=Marker.ARROW,
                 marker_size=[0.2, 0.05, 0.05],
                 marker_color=[0.0, 0.5, 0.5, 1.0],
                 controls=[],
                 controls_rotate=True,
                 controls_translate=True):
        self.position = kdl.Frame()
        self.position_exo = exo.KDLFrame()
        self.server = InteractiveMarkerServer(server_name)
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = frame_id
        self.int_marker.name = server_name
        self.int_marker.description = description
        self.int_marker.pose = list_to_pose(pose)
        self.position_exo = exo.KDLFrame(pose)
        self.position = kdl.Frame(
            kdl.Rotation.Quaternion(self.int_marker.pose.orientation.x, self.int_marker.pose.orientation.y, self.int_marker.pose.orientation.z, self.int_marker.pose.orientation.w),
            kdl.Vector(self.int_marker.pose.position.x, self.int_marker.pose.position.y, self.int_marker.pose.position.z))

        # create a grey box marker
        visual_marker = Marker()
        visual_marker.type = marker_shape
        if len(marker_size) != 3:
            raise RuntimeError("Length of marker_size needs to be 3.")
        visual_marker.scale.x = marker_size[0]
        visual_marker.scale.y = marker_size[1]
        visual_marker.scale.z = marker_size[2]
        if len(marker_color) != 4:
            raise RuntimeError("Length of marker_color needs to be 4.")
        visual_marker.color.r = marker_color[0]
        visual_marker.color.g = marker_color[1]
        visual_marker.color.b = marker_color[2]
        visual_marker.color.a = marker_color[3]

        # create a non-interactive control which contains the box
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append( visual_marker )

        # add the control to the interactive marker
        self.int_marker.controls.append( visual_control )

        if controls_rotate:
            self.addControl([1, 0, 0], InteractiveMarkerControl.ROTATE_AXIS)
            self.addControl([0, 1, 0], InteractiveMarkerControl.ROTATE_AXIS)
            self.addControl([0, 0, 1], InteractiveMarkerControl.ROTATE_AXIS)
        if controls_translate:
            self.addControl([1, 0, 0], InteractiveMarkerControl.MOVE_AXIS)
            self.addControl([0, 1, 0], InteractiveMarkerControl.MOVE_AXIS)
            self.addControl([0, 0, 1], InteractiveMarkerControl.MOVE_AXIS)

        for control in controls:
            self.int_marker.controls.append(control)

        self.server.insert(self.int_marker, self.process_feedback)
        self.server.applyChanges()

    def addControl(self, direction, control_type):
        control = InteractiveMarkerControl()
        quat = exo.KDLFrame([0, 0, 0] + direction + [1]).get_quaternion()
        control.orientation.x = quat[0]
        control.orientation.y = quat[1]
        control.orientation.z = quat[2]
        control.orientation.w = quat[3]
        control.interaction_mode = control_type
        self.int_marker.controls.append(control)

    def process_feedback(self, feedback):
        self.position = kdl.Frame(
            kdl.Rotation.Quaternion(feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w),
            kdl.Vector(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z))
        self.position_exo = exo.KDLFrame([feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z,
            feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w])
