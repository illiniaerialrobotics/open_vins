#!/usr/bin/env python3
"""Publish RViz line markers for a race layout YAML."""

from __future__ import annotations

from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def point(values) -> Point:
    p = Point()
    p.x = float(values[0])
    p.y = float(values[1])
    p.z = float(values[2])
    return p


def color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


def base_marker(frame_id: str, namespace: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(0)
    return marker


def append_segment(marker: Marker, start, end) -> None:
    marker.points.append(point(start))
    marker.points.append(point(end))


def append_rectangle(marker: Marker, corners) -> None:
    for start, end in zip(corners, corners[1:] + corners[:1]):
        append_segment(marker, start, end)


def make_gate_boundaries(frame_id: str, gate_index: int, gate: dict, spec: dict, rgba: ColorRGBA) -> Marker:
    """Draw outer gate boundary and inner opening boundary as line segments."""
    left_bottom = gate["bottom_corners"]["left"]
    right_bottom = gate["bottom_corners"]["right"]

    x = float(left_bottom[0])
    y_left = float(left_bottom[1])
    y_right = float(right_bottom[1])
    z_bottom = float(left_bottom[2])

    border = float(spec["border_height_m"])
    opening_height = float(spec["opening_height_m"])
    outer_height = opening_height + 2.0 * border
    y_sign = 1.0 if y_left >= y_right else -1.0

    inner_y_left = y_left - y_sign * border
    inner_y_right = y_right + y_sign * border
    inner_z_bottom = z_bottom + border
    inner_z_top = inner_z_bottom + opening_height

    outer = [
        [x, y_left, z_bottom],
        [x, y_right, z_bottom],
        [x, y_right, z_bottom + outer_height],
        [x, y_left, z_bottom + outer_height],
    ]
    inner = [
        [x, inner_y_left, inner_z_bottom],
        [x, inner_y_right, inner_z_bottom],
        [x, inner_y_right, inner_z_top],
        [x, inner_y_left, inner_z_top],
    ]

    marker = base_marker(frame_id, "race1_gate_boundaries", gate_index, Marker.LINE_LIST)
    marker.scale.x = 0.05
    marker.color = rgba
    append_rectangle(marker, outer)
    append_rectangle(marker, inner)
    return marker


def build_markers(layout: dict, frame_id: str) -> MarkerArray:
    markers = MarkerArray()
    spec = layout["gate_spec"]

    for index, gate in enumerate(layout.get("sequence", []), start=1):
        gate_color = (
            color(0.2, 0.85, 1.0, 0.9)
            if gate.get("type") == "single"
            else color(0.75, 0.5, 1.0, 0.9)
        )
        markers.markers.append(make_gate_boundaries(frame_id, index, gate, spec, gate_color))

    return markers


def main() -> None:
    rospy.init_node("race1_layout_rviz")

    default_layout = str(Path(__file__).with_name("race1_layout.yaml"))
    layout_yaml = rospy.get_param("~layout_yaml", default_layout)
    frame_id = rospy.get_param("~frame_id", "global")
    topic = rospy.get_param("~marker_topic", "/race1_layout_markers")
    publish_hz = float(rospy.get_param("~publish_hz", 1.0))

    with open(layout_yaml, "r", encoding="utf-8") as file:
        layout = yaml.safe_load(file)

    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(publish_hz)

    rospy.loginfo("Publishing race1 gates from %s on %s in frame %s", layout_yaml, topic, frame_id)
    while not rospy.is_shutdown():
        publisher.publish(build_markers(layout, frame_id))
        rate.sleep()


if __name__ == "__main__":
    main()
