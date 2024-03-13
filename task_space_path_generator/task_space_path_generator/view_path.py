import random
import sys
import argparse

import rclpy
from rclpy.clock import Clock
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.exceptions import ParameterUninitializedException
import rosbag2_py

from tf2_ros import Buffer, TransformListener

import sensor_msgs.msg
import std_msgs.msg
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from builtin_interfaces.msg import Duration, Time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Vector3, PoseArray, Quaternion, TransformStamped

import numpy as np

from task_space_path_generator.path_generators import SineOnePeriodPathGenerator, SinePathGenerator, PlanarPathGenerator, PathParameter, Transform
from task_space_path_generator.point_factory import PointFactory

class PathViewer(rclpy.node.Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('path_viewer', parameter_overrides=parameter_overrides)
        self.get_logger().info("Path Viewer initialization started")
        # ROS2 parameters
        self.declare_parameter('path', None, ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('rate', 5, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('marker_topic', 'visualization_marker', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        try:
            self.path
        except ParameterUninitializedException:
            self.get_logger().error("'path' parameter is uninitialized. Aborting...")
            raise ValueError("'path' parameter cannot be uninitialized.")
        
        # Print parameters
        self.get_logger().info("Publishing Rviz markers on topic '%s' with rate '%d'" % (self.marker_topic, self.rate))
        
        # Delete the attributes
        self.msg_templates = {}
        self._base_frame = None
        self._path = None
        self.import_path()
        # Initialize message templates
        self._init_msg_templates()
        
        # Publishers, subscribers and timers
        self.pub_rviz_marker = self.create_publisher(Marker, self.marker_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._timer_callback)
        
    @property
    def marker_topic(self):
        return self.get_parameter('marker_topic').get_parameter_value().string_value

    @property
    def rate(self):
        return self.get_parameter('rate').get_parameter_value().integer_value
    
    @property
    def path(self):
        return self.get_parameter('path').get_parameter_value().string_value
    
    def import_path(self):
        """
        This method imports the path from a bag file.
        """
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=self.path, storage_id='sqlite3'), rosbag2_py._storage.ConverterOptions('', ''))
        topic, msg, timestamp = reader.read_next()
        
        if topic != '/cartesian_path':
            self.get_logger().error("Can't import the path: the bag file does not contain a path")
            raise ValueError("'Can't import the path: the bag file does not contain a path")
        
        self.get_logger().info("Importing path from bag file '%s'" % self.path)
        deserialized_msg = deserialize_message(msg, PoseArray)
        self._base_frame = deserialized_msg.header.frame_id
        self._path = []
        for pose in deserialized_msg.poses:
            self._path.append(pose.position)
        self.get_logger().info("Plane orientation: '%s'" % str(deserialized_msg.poses[0].orientation))

    def _timer_callback(self):
        # pose_msg = self.msg_templates['pose']
        # pose_msg.position.x = random.uniform(-1.0, 1.0)
        # pose_msg.position.y = random.uniform(-1.0, 1.0)
        # pose_msg.position.z = random.uniform(-1.0, 1.0)
        # scale = self.msg_templates['vector3']
        # scale.x = random.uniform(0.1, 1.0)
        # scale.y = random.uniform(0.1, 1.0)
        # scale.z = random.uniform(0.1, 1.0)
        
        color = self.msg_templates['color']
        color.a = 1.0
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        
        lifetime = self.msg_templates['duration']

        if self._path is not None:
            self._publish_path(path=self._path, width=0.01, color=color, lifetime=lifetime, override_last=True)
        
    
    def _init_msg_templates(self):
        """
        This method initializes the message templates used by the node. 
        The message templates are used to avoid the creation of new message objects at each iteration.
        """
        # Vector3 msg
        self.msg_templates['vector3'] = Vector3()
        
        # Pose msg
        self.msg_templates['pose'] = Pose()
        
        # Color msg
        self.msg_templates['color'] = std_msgs.msg.ColorRGBA()
        self.msg_templates['color'].a = 1.0
        
        # Duration msg
        self.msg_templates['duration'] = Duration()
        self.msg_templates['duration'].sec = 1
        self.msg_templates['duration'].nanosec = 0
        
        # Path Marker msg
        path_marker = Marker()
        path_marker.id = hash("view_path") % 2147483648
        path_marker.header.frame_id = self._base_frame
        path_marker.ns = "Path" # unique ID
        path_marker.action = Marker.ADD
        path_marker.type = Marker.LINE_LIST
        path_marker.lifetime = self.msg_templates['duration']
        self.msg_templates['path'] = path_marker
        
    def _publish_path(self, path, width, color, lifetime=None, override_last=False):
        """
        Publish a path on the rviz marker topic. The path must be represented as a list of N Point messages.
        """
        if type(path) != list and type(path) != np.ndarray and type(path) != np.matrix:
            self.get_logger().error("Invalid type '%s' for path argument" % type(path).__name__)
            return
        
        path_marker = self.msg_templates['path']
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.header.frame_id = self._base_frame
        if not override_last:
            path_marker.id += 1
        path_marker.scale.x = width 
        path_marker.scale.y = width 
        path_marker.scale.z = width 
        path_marker.color = color
        
    
        if lifetime is None:
            duration = Duration()
            duration.sec = 0
            duration.nanosec = 0
            path_marker.lifetime = duration
        else:
            path_marker.lifetime = lifetime
        
        # Set the path points and color
        path_marker.points[:] = [] # clear
        path_marker.colors[:] = []
        
        if type(path) == list: # we already have a list of Point messages
            if len(path) == 0:
                return
            if type(path[0]) != Point:
                self.get_logger().error("Invalid type '%s' for path list" % type(path[0]).__name__)
                return
            points_list = path
        else: # numpy.ndarray or numpy.matrix
            if path.shape[0] != 3:
                self.get_logger().error("Invalid shape '%s' for path argument" % str(path.shape))
                return
            points_list = [path[:, i] for i in range(path.shape[1])] # Convert the numpy.ndarray to a list of numpy.ndarray (one for each column)
        # Now points_list is a list of points
        path_marker.points.append(PointFactory.build_point(points_list[0]))
        path_marker.colors.append(color)
        for i in range(1,len(points_list)-1):
            path_marker.points.append(PointFactory.build_point(points_list[i]))
            path_marker.colors.append(color)
            path_marker.points.append(PointFactory.build_point(points_list[i]))
            path_marker.colors.append(color)
        path_marker.points.append(PointFactory.build_point(points_list[len(points_list)-1])) 
        path_marker.colors.append(color)
        self.pub_rviz_marker.publish(path_marker)
        
    def _delete_all_markers(self):
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = self.draw_base_frame
        delete_all_marker.header.stamp = self.get_clock().now().to_msg()
        delete_all_marker.action = Marker.DELETEALL
        self.pub_rviz_marker.publish(delete_all_marker)
    
def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('path', help='The bagfile containing the path', type=str)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])
    path_viewer_node = PathViewer(parameter_overrides=[Parameter('path', Parameter.Type.STRING, parsed_args.path)])
    
    try:
        rclpy.spin(path_viewer_node)
    except KeyboardInterrupt:
        pass

    path_viewer_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()