import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from visualization_msgs.msg import MarkerArray, Marker

def pose_graph_to_marker_array(pose_graph, color_map):
    """Converts a PoseGraph message to a MarkerArray message"""
    marker_array = MarkerArray()

    # Nodes (poses)
    marker = Marker()
    marker.header.frame_id = "map" # TODO: make this the base TF frame of the robot
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = "r" + str(pose_graph.robot_id)
    marker.id = "poses"
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = color_map[pose_graph.robot_id][0]
    marker.color.g = color_map[pose_graph.robot_id][1]
    marker.color.b = color_map[pose_graph.robot_id][2]
    marker.color.a = 1.0
    marker.frame_locked = True
    for node in pose_graph.values:
        marker.points.append(node.pose.position)
    marker_array.markers.append(marker)

    # Edges (constraints)
    marker = Marker()
    marker.header.frame_id = "map" # TODO: make this the base TF frame of the robot
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = "r" + str(pose_graph.robot_id)
    marker.id = "edges"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.r = color_map[pose_graph.robot_id][0]
    marker.color.g = color_map[pose_graph.robot_id][1]
    marker.color.b = color_map[pose_graph.robot_id][2]
    marker.color.a = 1.0
    marker.frame_locked = True
    for edge in pose_graph.edges:
        if edge.key_from.robot_id == pose_graph.robot_id and edge.key_to.robot_id == pose_graph.robot_id:
            marker.points.append(pose_graph.values[edge.key_from].pose.position)
            marker.points.append(pose_graph.values[edge.key_to].pose.position)
        elif edge.key_from.robot_id == pose_graph.robot_id:
            marker.points.append(pose_graph.values[edge.key_from].pose.position)
            position_to = pose_graph.values[edge.key_from].pose.position
            position_to.x += edge.measurement.position.x
            position_to.y += edge.measurement.position.y
            position_to.z += edge.measurement.position.z
            marker.points.append(pose_graph.values[edge.key_to].pose.position) 
        elif edge.key_to.robot_id == pose_graph.robot_id:
            position_from = pose_graph.values[edge.key_to].pose.position
            position_from.x -= edge.measurement.position.x
            position_from.y -= edge.measurement.position.y
            position_from.z -= edge.measurement.position.z
            marker.points.append(position_from)
            marker.points.append(pose_graph.values[edge.key_to].pose.position) 
    marker_array.markers.append(marker)
    
    return marker_array

