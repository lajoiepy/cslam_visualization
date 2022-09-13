import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
import cslam_visualization.visualization_utils as utils
from visualization_msgs.msg import MarkerArray, Marker

class Visualization(Node):
    """ Visualization node """

    def __init__(self):
        """Initialization and parameter parsing"""
        super().__init__('visualization')

        self.declare_parameters(
            namespace='',
            parameters=[('nb_robots', None)])
        self.params = {}
        self.params['nb_robots'] = self.get_parameter('nb_robots').value

        self.receive_pose_graph_subscribers = {}
        for i in range(self.params['nb_robots']):
            self.receive_pose_graph_subscribers[i] = self.create_subscription(
                PoseGraph,
                '/r' + str(i) + '/viz/pose_graph',
                self.receive_pose_graph_callback,
                10)
            self.get_logger().info('Subscribed to /r' + str(i) + '/viz/pose_graph')
        
        self.pose_graph_markers_publisher = self.node.create_publisher(
            MarkerArray, "/viz/pose_graph_markers", 10)


    def receive_pose_graph_callback(self, msg):
        """Callback for receiving pose graph"""
        self.get_logger().info('Received pose graph from robot ' + str(msg.robot_id))
        marker_array = utils.pose_graph_to_marker_array(msg)
        self.pose_graph_markers_publisher.publish(marker_array)

if __name__ == '__main__':

    rclpy.init(args=None)
    viz = Visualization()
    viz.get_logger().info('Initialization done.')
    rclpy.spin(viz)
    rclpy.shutdown()
