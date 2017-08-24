from __future__ import division, print_function, absolute_import

import rospy

from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers import interactive_marker_server


def make_interactive_marker(name, pose, frame='base', color=(0.5, 0.5, 0.5)):
    rospy.loginfo("Creating imarker...")
    int_marker = interactive_marker_server.InteractiveMarker()
    int_marker.header.frame_id = frame
    int_marker.header.stamp = rospy.Time.now()

    int_marker.scale = 0.25
    int_marker.name = name
    int_marker.description = "interactive marker"

    cylinder_marker = Marker()
    cylinder_marker.type = Marker.CYLINDER
    cylinder_marker.scale.x = 0.25
    cylinder_marker.scale.y = 0.25
    cylinder_marker.scale.z = 0.25
    cylinder_marker.color.r = color[0]
    cylinder_marker.color.g = color[1]
    cylinder_marker.color.b = color[2]
    cylinder_marker.color.a = 0.5

    cylinder_control = InteractiveMarkerControl()
    cylinder_control.always_visible = True
    cylinder_control.markers.append(cylinder_marker)
    int_marker.controls.append(cylinder_control)

    for axis, orientation in zip(['x', 'y', 'z'],
                                 [(1, 1, 0, 0), (1, 0, 1, 0), (1, 0, 0, 1)]):
        control = InteractiveMarkerControl()
        for attr, val in zip(['w', 'x', 'y', 'z'],
                             orientation):
            setattr(control.orientation, attr, val)
        control.name = "rotate_{}".format(axis)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control.name = "move_{}".format(axis)
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

    int_marker.pose = pose
    rospy.loginfo("Imarker created")
    return int_marker
