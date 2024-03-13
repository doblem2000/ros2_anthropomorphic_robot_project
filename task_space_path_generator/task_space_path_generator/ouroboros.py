"""
This module contains the Ouroboros class, which is a ROS2 node for generating and exporting paths in a robotics project.
"""

import random

import rclpy
from rclpy.clock import Clock
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message

import rosbag2_py
import transforms3d as tf3d
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

import sensor_msgs.msg
import std_msgs.msg
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from builtin_interfaces.msg import Duration, Time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Vector3, PoseArray, Quaternion, TransformStamped

import numpy as np

from task_space_path_generator.path_generators import SineOnePeriodPathGenerator, SinePathGenerator, PlanarPathGenerator, PathParameter, Transform
from task_space_path_generator.point_factory import PointFactory
from task_space_path_generator.ouroboros_parameters import ouroboros_parameters

SUPPORTED_PATH_GENERATORS = {
    'Sine Wave': SinePathGenerator(upper_bounds={'amplitude': 10.0, 'frequency': 5.0, 'propagation_length': 30.0}),
    'Sine Wave (One Period)': SineOnePeriodPathGenerator(upper_bounds={'amplitude': 10.0, 'wavelength': 50.0})
}
CARTESIAN_PATH_FRAME_NAME = "cartesian_path_frame"

class Ouroboros(rclpy.node.Node):
    """
    Ouroboros class is a ROS2 node for generating and exporting paths in a robotics project.
    """

    def __init__(self, parameter_overrides=None):
        """@brief Initialize the Ouroboros node.

        @param parameter_overrides (dict): Dictionary of parameter overrides.
        """
        super().__init__('ouroboros', parameter_overrides=parameter_overrides)
        self.get_logger().info("Ouroboros initialization started")
        # ROS2 parameters
        self.param_listener = ouroboros_parameters.ParamListener(self)
        
        # Print parameters
        self.get_logger().info("Publishing Rviz markers on topic '%s' with rate '%d'" % (self.marker_topic, self.path_publish_rate))
        
        # Delete the attributes
        self.msg_templates = {}
        self.generator = None
        self.tf_buffer = Buffer()
        
        # Initialize message templates
        self._init_msg_templates()
        
        # Publishers, subscribers and timers
        self.pub_rviz_marker = self.create_publisher(Marker, self.marker_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)
        self.timer_publish_path = self.create_timer(1.0 / self.path_publish_rate, self._publish_path_callback)
        self.timer_publish_tf = self.create_timer(1.0 / self.tf_publish_rate, self._publish_tf_callback)
        
    @property
    def marker_topic(self):
        """
        @brief Gets the value of the 'marker_topic' parameter.
        @return (str): The topic name for the marker.
        """
        return self.get_parameter('marker_topic').get_parameter_value().string_value
    
    @property
    def draw_base_frame(self):
        """
        @brief Gets the value of the 'draw_base_frame' parameter.
        @return (str): The draw base frame.
        """
        return self.get_parameter('draw_base_frame').get_parameter_value().string_value
    
    @draw_base_frame.setter
    def draw_base_frame(self, frame):
        """
        @brief Set the draw base frame.
        @param frame (str): The draw base frame.
        """
        self.set_parameters_atomically([Parameter('draw_base_frame', Parameter.Type.STRING, frame)])
    
    @property
    def export_base_frame(self):
        """
        @brief Returns the base frame for exporting the path.
        @return (str): The base frame for exporting.
        """
        return self.get_parameter('export_base_frame').get_parameter_value().string_value

    @property
    def path_publish_rate(self):
        """
        @brief Returns the path publish rate.
        @return (str): The path publish rate.
        """
        return self.get_parameter('path_publish_rate').get_parameter_value().integer_value
    
    @property
    def tf_publish_rate(self):
        """
        @brief Returns the tf publish rate.
        @return (str): The tf publish rate.
        """
        return self.get_parameter('tf_publish_rate').get_parameter_value().integer_value
    
    @property
    def n_samples_draw(self):
        """
        @brief Get the number of samples to draw.

        @return (int): The number of samples to draw.
        """
        return self.get_parameter('n_samples_draw').get_parameter_value().integer_value
    
    @property
    def n_samples_export(self):
        """
        @brief Get the number of samples to export.

        @return (int): The number of samples to export.
        """
        return self.get_parameter('n_samples_export').get_parameter_value().integer_value
    
    @property
    def all_frames_yaml(self):
        return self.tf_buffer.all_frames_as_yaml()
    
    @property
    def path_generator(self):
        return self.generator
    
    @property
    def path_generator_name(self):
        """
        @brief Get the name of the path generator.
        
        @return The name of the path generator, or None if the generator is not set.
        """
        if self.generator is None:
            return None
        for k, v in SUPPORTED_PATH_GENERATORS.items():
            if v == self.generator:
                return k
        return None
    
    @path_generator_name.setter
    def path_generator_name(self, generator):
        """
        Set the path generator name.

        @brief This method sets the name of the path generator to be used.
        @param generator: The name of the path generator.
        """
        if generator not in SUPPORTED_PATH_GENERATORS.keys():
            raise ValueError("Unsupported path generator '%s'" % generator)
        self.generator = SUPPORTED_PATH_GENERATORS[generator]
    
    def _publish_tf_callback(self):
            """
            @brief Publishes the transform message for the task space path generator.
            """
            if self.generator is None:
                return
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = self.draw_base_frame #The parent frame is the draw frame
            transform_msg.child_frame_id = CARTESIAN_PATH_FRAME_NAME
            transform = Transform()
            plane_orientation = self.generator._rotation_quaternion
            orientation = Quaternion()
            orientation.x = plane_orientation[0]
            orientation.y = plane_orientation[1]
            orientation.z = plane_orientation[2]
            orientation.w = plane_orientation[3]
            path_offset = self.generator._offset
            transform._translation.x = path_offset[0]
            transform._translation.y = path_offset[1]
            transform._translation.z = path_offset[2]
            transform._rotation = orientation
            transform_msg.transform = transform
            self.tf_broadcaster.sendTransform(transform_msg)
    
    def export_path(self, file_path):
        """
        @brief This method exports the path to a bag file.
        @param file_path (str): The path to the bag file.
        """ 
        if self.generator is None:
            self.get_logger().error("Can't export the path: no path generator set")
            return
        if not issubclass(type(self.generator), PlanarPathGenerator):
            raise NotImplementedError("Exporting paths is only supported for planar paths")
        
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=file_path, storage_id='sqlite3'), rosbag2_py._storage.ConverterOptions('', ''))
        topic_info = rosbag2_py.TopicMetadata(
            name='/cartesian_path',
            type='geometry_msgs/PoseArray',
            serialization_format='cdr')
        writer.create_topic(topic_info)
        time_stamp = self.get_clock().now()

        path_msg = PoseArray()
        path_msg.header.stamp = time_stamp.to_msg()
        path_msg.header.frame_id = self.export_base_frame

        points = self.generator.generate_path(n_samples=self.n_samples_export)
        points = PointFactory.build_point_list(points)
        
        # Get the orientation of the plane with respect to the base frame
        #self.update_tf() # This is a trick to have a recent transform in the buffer
        time = Time()
        time.sec = 0
        time.nanosec = 0
        #base_to_path_tf = self.tf_buffer.lookup_transform_full(CARTESIAN_PATH_FRAME_NAME, time, self.export_base_frame, time, self.export_base_frame)
        base_to_path_tf = self.tf_buffer.lookup_transform(self.export_base_frame, CARTESIAN_PATH_FRAME_NAME, time)
        base_to_draw_tf = self.tf_buffer.lookup_transform(self.export_base_frame, self.draw_base_frame, time)
        if base_to_path_tf is None:
            self.get_logger().error("Can't export the path: no transform found between '%s' and '%s'" % self.export_base_frame, CARTESIAN_PATH_FRAME_NAME)
            return
        if base_to_draw_tf is None:
            self.get_logger().error("Can't export the path: no transform found between '%s' and '%s'" % self.export_base_frame, self.draw_base_frame)
            return
        self.get_logger().info("Base to path tf: %s" % str(base_to_path_tf))
        self.get_logger().info("Base to draw tf: %s" % str(base_to_draw_tf))
        # Get the orientation of the plane with respect to the export base frame
        plane_orientation = base_to_path_tf._transform.rotation
        for point in points:
            # Transform the point coordinates from the draw frame to the export base frame before exporting
            rotation_matrix = tf3d.quaternions.quat2mat(np.array([base_to_draw_tf._transform._rotation.w, base_to_draw_tf._transform._rotation.x, base_to_draw_tf._transform._rotation.y, base_to_draw_tf._transform._rotation.z]))
            point_vector = np.array([point.x, point.y, point.z])
            point_vector = np.dot(rotation_matrix, point_vector)
            point_vector += np.array([base_to_draw_tf._transform._translation.x, base_to_draw_tf._transform._translation.y, base_to_draw_tf._transform._translation.z])
            point.x = point_vector[0]
            point.y = point_vector[1]
            point.z = point_vector[2]
            # Create the pose message
            pose = Pose()
            pose.position = point
            pose.orientation = plane_orientation
            path_msg.poses.append(pose)
        
        writer.write(
            '/cartesian_path',
            serialize_message(path_msg),
            time_stamp.nanoseconds
        )
        


    def _publish_path_callback(self):
        """
        @brief Publishes a path generated by the generator.

        This method generates a path using the generator object and publishes it with the specified color, width, and lifetime.
        """
        color = self.msg_templates['color']
        color.a = 1.0
        color.r = 0.0
        color.g = 0.0
        color.b = 1.0

        lifetime = self.msg_templates['duration']

        if self.generator is not None:
            self._publish_path(path=self.generator.generate_path(n_samples=self.n_samples_draw), width=0.01, color=color, lifetime=lifetime, override_last=True)
        
    
    def _init_msg_templates(self):
        """
        @brief This method initializes the message templates used by the node. 
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
        self.msg_templates['duration'].sec = 0
        self.msg_templates['duration'].nanosec = 0
        
        # Sphere Marker msg
        sphere_marker = Marker()
        sphere_marker.id = 0
        sphere_marker.header.frame_id = self.draw_base_frame
        sphere_marker.ns = "Sphere" # unique ID
        sphere_marker.action = Marker.ADD
        sphere_marker.type = Marker.SPHERE
        sphere_marker.lifetime = self.msg_templates['duration']
        self.msg_templates['sphere'] = sphere_marker
        
        # Path Marker msg
        path_marker = Marker()
        path_marker.id = 0
        path_marker.header.frame_id = self.draw_base_frame
        path_marker.ns = "Path" # unique ID
        path_marker.action = Marker.ADD
        path_marker.type = Marker.LINE_LIST
        path_marker.lifetime = self.msg_templates['duration']
        self.msg_templates['path'] = path_marker
        
        # self.sphere_marker.pose.position.x = 0.0
        # self.sphere_marker.pose.position.y = 0.0
        # self.sphere_marker.pose.position.z = 0.0
        # self.sphere_marker.pose.orientation.x = 0.0
        # self.sphere_marker.pose.orientation.y = 0.0
        # self.sphere_marker.pose.orientation.z = 0.0
        # self.sphere_marker.pose.orientation.w = 1.0
        # self.sphere_marker.scale.x = 1.0
        # self.sphere_marker.scale.y = 1.0
        # self.sphere_marker.scale.z = 1.0
        # self.sphere_marker.color.a = 1.0
        # self.sphere_marker.color.r = 0.0
        # self.sphere_marker.color.g = 1.0
        # self.sphere_marker.color.b = 0.0
        # self.sphere_marker.header.stamp = self.get_clock().now().to_msg()
    
    def _publish_sphere(self, pose, scale, color, lifetime=None, override_last=False):
        """
        @brief This method publishes a sphere marker on the rviz marker topic.
        It has been used for testing purposes.
        """
        sphere_marker = self.msg_templates['sphere']
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.header.frame_id = self.draw_base_frame
        if not override_last:
            sphere_marker.id += 1
        sphere_marker.pose = pose
        sphere_marker.color = color
        sphere_marker.scale = scale
    
        if lifetime is None:
            duration = Duration()
            duration.sec = 0
            duration.nanosec = 0
            sphere_marker.lifetime = duration
        else:
            sphere_marker.lifetime = lifetime
        
        self.pub_rviz_marker.publish(sphere_marker)
        
    def _publish_path(self, path, width, color, lifetime=None, override_last=False):
        """
        @brief Publish a path on the rviz marker topic.
        The path can either be represented as a list of N Point messages or as a 3xN numpy.ndarray.
        @param path (list or numpy.ndarray): The path to be published.
        @param width (float): The width of the path.
        @param color (std_msgs.msg.ColorRGBA): The color of the path.
        @param lifetime (builtin_interfaces.msg.Duration): The lifetime of the path.
        @param override_last (bool): If True, the last path is overridden.
        """
        if type(path) != list and type(path) != np.ndarray and type(path) != np.matrix:
            self.get_logger().error("Invalid type '%s' for path argument" % type(path).__name__)
            return
        
        path_marker = self.msg_templates['path']
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.header.frame_id = self.draw_base_frame
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
        """
        @brief Deletes all markers in the RViz visualization.
        """
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = self.draw_base_frame
        delete_all_marker.header.stamp = self.get_clock().now().to_msg()
        delete_all_marker.action = Marker.DELETEALL
        self.pub_rviz_marker.publish(delete_all_marker)
        
    @staticmethod
    def get_supported_generators():
        return SUPPORTED_PATH_GENERATORS.keys()
    
def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    ouroboros = Ouroboros()
    ouroboros.set_path_generator(SineOnePeriodPathGenerator())
    
    try:
        rclpy.spin(ouroboros)
    except KeyboardInterrupt:
        pass

    ouroboros.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()