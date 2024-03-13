"""
This module contains the classes for the path generators in a three dimensional task space.
"""
from enum import Enum
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import numpy as np

import math
import rosbag2_py
from copy import deepcopy
from typing import List, Optional, TypeVar, Generic, get_args
import matplotlib.pyplot as plt
import transforms3d as tf3d

from geometry_msgs.msg import Transform, Quaternion

class PathType(Enum):
    SINE = 1
    SINE_ONE_PERIOD = 2

T = TypeVar('T')
class PathParameter(Generic[T]):
    """
    @brief This class represents a parameter of a path generator.
    """
    def __init__(self, name: str, value: T, upper_bound: T | None = None, lower_bound: T | None = None) -> None:
        self._name = name
        self._value = value
        self._upper_bound = upper_bound
        self._lower_bound = lower_bound
        self._type = type(value)
    
    @property
    def name(self) -> str:
        """
        @brief Name of the parameter
        """
        return self._name

    @property
    def value(self) -> T:
        """
        @brief Value of the parameter
        """
        return self._value
    
    @property
    def upper_bound(self) -> T | None:
        """
        @brief Upper bound of the parameter
        """
        return self._upper_bound

    @upper_bound.setter
    def upper_bound(self, value: T) -> None:
        """
        @brief Setter for the upper bound of the parameter
        """
        if self._lower_bound is not None and value < self._lower_bound:
            raise ValueError(f'Upper bound {value} is less than lower bound {self._lower_bound}')
        self._upper_bound = value

    @property
    def lower_bound(self) -> T | None:
        """
        @brief Lower bound of the parameter
        """
        return self._lower_bound

    @lower_bound.setter
    def lower_bound(self, value: T) -> None:
        """
        @brief Setter for the lower bound of the parameter
        """
        if self._upper_bound is not None and value > self._upper_bound:
            raise ValueError(f'Lower bound {value} is greater than upper bound {self._upper_bound}')
        self._lower_bound = value

    @property
    def type(self) -> type:
        """
        @brief Type of the parameter
        """
        return self._type

    @value.setter
    def value(self, value: T) -> None:
        """
        @brief Setter for the value of the parameter
        """
        if self._upper_bound is not None and value > self._upper_bound:
            raise ValueError(f'Value {value} exceeds upper bound {self._upper_bound}')
        if self._lower_bound is not None and value < self._lower_bound:
            raise ValueError(f'Value {value} exceeds lower bound {self._lower_bound}')
        self._value = value

"""
Path parameters for each path type.
"""
PATH_PARAMETERS = {
    PathType.SINE: {
        'amplitude': PathParameter[float](name='amplitude', value=0.5, lower_bound=0.0),
        'frequency': PathParameter[float](name='frequency', value=1.0, lower_bound=0.0),
        'propagation_length': PathParameter[float](name='propagation_length', value=1.0, lower_bound=0.0),
    },
    PathType.SINE_ONE_PERIOD: {
        'amplitude': PathParameter[float](name='amplitude', value=0.5, lower_bound=0.0),
        'wavelength': PathParameter[float](name='wavelength', value=1.0, lower_bound=0.0),
    }
}

class PathGenerator:
    """
    @brief This class is an abstract class for path generators in a three dimensional task space.
    """
    def __init__(self, path_type, path_parameters) -> None:
        self._path_type = path_type
        self._path_parameters = path_parameters
    
    def generate_path(self, n_samples: int) -> np.ndarray:
        """
        @brief Abstract function that generates a path in a three dimensional task space and returns n_samples waypoints sampled from the path.
        @param n_samples (int): number of samples to generate for each period
        """
        raise NotImplementedError
    
    @property
    def path_type(self) -> PathType:
        """
        @brief Type of the path
        """
        return self._path_type
    
    @property
    def path_parameters(self) -> dict[str, PathParameter]:
        """
        @brief Parameters of the path
        """
        return self._path_parameters

class PlanarPathGenerator(PathGenerator):
    """@brief This class is an abstract class for planar path generators in a three dimensional task space."""
    def __init__(self, path_type, path_parameters, offset = np.zeros((3,)), rotation_matrix = np.eye(3)) -> None:
        super().__init__(path_type, path_parameters)
        #print(path_parameters)
        path_parameters.update(
            {'propagation_axis_angle': PathParameter[float](name='propagation_axis_angle',
                                                             value=np.pi/2,
                                                               lower_bound=0.0,
                                                                 upper_bound=2*np.pi)
             })
        self._transform = Transform()
        #self._offset = offset
        #self._rotation_matrix = rotation_matrix

    @property
    def _offset(self) -> np.ndarray:
        """
        @brief Getter for the offset of the path generator.
        @return offset (np.ndarray): offset vector (3x1, expressed in meters)
        """
        return np.array([self._transform.translation.x, self._transform.translation.y, self._transform.translation.z])

    @_offset.setter
    def _offset(self, offset: np.ndarray) -> None:
        """
        @brief Setter for the offset of the path generator.
        @param offset (np.ndarray): offset vector (3x1, expressed in meters)
        """
        self._transform.translation.x = offset[0]
        self._transform.translation.y = offset[1]
        self._transform.translation.z = offset[2]
    
    @property
    def _rotation_matrix(self) -> np.ndarray:
        """
        @brief Getter for the rotation of the path generator expressed in a rotation matrix.
        @return rotation_matrix (np.ndarray): rotation matrix (3x3)
        """
        return tf3d.quaternions.quat2mat(np.array([self._transform._rotation.w, self._transform._rotation.x, self._transform._rotation.y, self._transform._rotation.z]))

    @property
    def _rotation_quaternion(self) -> np.ndarray:
        """
        @brief Getter for the rotation of the path generator expressed in quaternions.
        @return quaternion (np.ndarray): quaternion (4x1)
        """
        return np.array([self._transform._rotation.x, self._transform._rotation.y, self._transform._rotation.z, self._transform._rotation.w])
    
    @property
    def _rotation_rpy(self) -> np.ndarray:
        """
        @brief Getter for the rotation of the path generator expressed in roll, pitch and yaw angles.
        @return rpy (np.ndarray): roll, pitch and yaw angles (3x1)
        """
        return tf3d.euler.quat2euler(np.array([self._transform._rotation.w, self._transform._rotation.x, self._transform._rotation.y, self._transform._rotation.z]))

    @_rotation_rpy.setter
    def _rotation_rpy(self, rpy: np.ndarray) -> None:
        """
        @brief Setter for the rotation of the path generator expressed in roll, pitch and yaw angles.
        @param rpy (np.ndarray): roll, pitch and yaw angles (3x1)
        """
        quaternion_tf3d = tf3d.euler.euler2quat(rpy[0], rpy[1], rpy[2])
        quaternion_ros = Quaternion()
        quaternion_ros.x = quaternion_tf3d[1]
        quaternion_ros.y = quaternion_tf3d[2]
        quaternion_ros.z = quaternion_tf3d[3]
        quaternion_ros.w = quaternion_tf3d[0]
        self._transform._rotation = quaternion_ros

    def _generate_xy_path(self, n_samples: int) -> np.ndarray:
        """
        @brief This function generates a path in a two dimensional task space and returns n_samples waypoints sampled from the path.
        """
        raise NotImplementedError

    def generate_path(self, n_samples: int) -> np.ndarray:
        """
        @brief This function generates a path in a three dimensional task space and returns n_samples waypoints sampled from the path.
        """
        xy_path = self._generate_xy_path(n_samples)
        z_path = np.zeros((1, n_samples))
        path = np.concatenate((xy_path, z_path), axis=0)
        # Apply the rotation in the XY plane
        xy_rotation_matrix = tf3d.euler.euler2mat(0.0, 0.0, self.path_parameters['propagation_axis_angle'].value, axes='sxyz')
        path = xy_rotation_matrix @ path
        # Apply the transform between the base frame and the path frame
        rotated_path = self._rotation_matrix @ path
        # print(rotated_path[:,1].shape)
        # print(self._offset.shape)
        for j in range(n_samples):
            rotated_path[:,j] += self._offset
        return rotated_path
    
    @property
    def offset(self) -> np.ndarray:
        """
        @brief Getter for the offset of the path generator.
        @return offset (np.ndarray): offset vector (3x1, expressed in meters)
        """
        return self._offset
    
    @offset.setter
    def offset(self, offset: np.ndarray) -> None:
        """
        @brief Setter for the offset of the path generator.
        @param offset (np.ndarray): offset vector (3x1, expressed in meters)
        """
        self._offset = offset

    @property
    def rotation_matrix(self) -> np.ndarray:
        """
        @brief Getter for the rotation of the path generator expressed in a rotation matrix.
        @return rotation_matrix (np.ndarray): rotation matrix (3x3)
        """
        return self._rotation_matrix
    
    @rotation_matrix.setter
    def rotation_matrix(self, rotation_matrix: np.ndarray) -> None:
        """
        @brief Setter for the rotation of the path generator expressed in a rotation matrix.
        @param rotation_matrix (np.ndarray): rotation matrix (3x3)
        """
        self._rotation_matrix = rotation_matrix

class SinePathGenerator(PlanarPathGenerator):
    """
    @brief This class generates a sine path in a three dimensional task space.
    """
    def __init__(self, upper_bounds = None) -> None:
        default_parameters = deepcopy(PATH_PARAMETERS[PathType.SINE])
        if upper_bounds is not None:
            for parameter, upper_bound in upper_bounds.items():
                default_parameters[parameter].upper_bound = upper_bound
        super().__init__(path_type=PathType.SINE, path_parameters=default_parameters)

    def _generate_xy_path(self, n_samples: int) -> np.ndarray:
        """
        @brief This functions generates a sine path in a two dimensional task space and returns n_samples waypoints sampled from the path.
        The sine function is defined the by the amplitude, frequency and lenght on the propagation axis.
        @param n_samples (int): number of samples to generate for each period
        """
        # We interpret n_samples as the number of samples per period
        #n_samples_per_period = n_samples

        # Calculate the angular frequency of the sine wave
        omega = 2 * np.pi * self._path_parameters['frequency'].value

        # Calculate the number of periods
        n_periods = self._path_parameters['propagation_length'].value * self._path_parameters['frequency'].value

        # The number of samples is equal to the number of periods times the number of samples per period
        #n_samples = int(math.ceil(n_periods * n_samples_per_period))

        # Generate a sine in the XY plane. 
        x = np.linspace(0, self._path_parameters['propagation_length'].value, num=n_samples)
        y = self._path_parameters['amplitude'].value * np.sin(omega * x)
        
        points = np.stack((x, y), axis=0) # points is a 2xn_samples matrix

        return points
    
class SineOnePeriodPathGenerator(PlanarPathGenerator):
    """
    @brief This class generates a single period of a sine path in a three dimensional task space.
    """
    def __init__(self, upper_bounds = None) -> None:
        default_parameters = deepcopy(PATH_PARAMETERS[PathType.SINE_ONE_PERIOD])
        if upper_bounds is not None:
            for parameter, upper_bound in upper_bounds.items():
                default_parameters[parameter].upper_bound = upper_bound
        super().__init__(path_type=PathType.SINE_ONE_PERIOD, path_parameters=default_parameters)

    def _generate_xy_path(self, n_samples: int) -> np.ndarray:
        """
        @brief This functions generates a single period of a sine path in a two dimensional task space and returns n_samples waypoints sampled from the path.
        The sine function is defined the by the amplitude and wavelength.
        @param n_samples (int): number of samples to generate for each period
        """
        # We interpret n_samples as the number of samples per period
        #n_samples_per_period = n_samples

        # Calculate the angular frequency of the sine wave
        if self._path_parameters['wavelength'].value != 0:
            omega = 2 * np.pi / self._path_parameters['wavelength'].value
        else:
            omega = 0
        # Generate a sine in the XY plane. 
        x = np.linspace(0, self._path_parameters['wavelength'].value, num=n_samples)
        y = self._path_parameters['amplitude'].value * np.sin(omega * x)
        
        points = np.stack((x, y), axis=0)
        return points

# TODO: BELOW THERE ARE SOME FUNCTIONS THAT ARE NOT USED ANYMORE
def generate_sine_path(amplitude: float, frequency: float, propagation_length: float, offset: np.ndarray, rotation_matrix: np.ndarray, n_samples_per_period: int=20) -> np.ndarray:
    """@brief This functions generates waypoints in a three dimensional task space. 
    The waypoints are sampled from a sine function in the task space.
    The sine function is first defined on the XY plane by the amplitude, frequency and lenght on the propagation axis.
    Then the sine function is rotated by the rotation matrix and translated by the offset vector.
       
    @param amplitude (float): amplitude of the sine function (expressed in meters)
    @param frequency (float): frequency of the sine function (expressed in oscillations/meter)
    @param propagation_length (float): length of the propagation axis (expressed in meters)
    @param offset (np.ndarray): offset vector (3x1, expressed in meters)
    @param rotation_matrix (np.ndarray): rotation matrix (3x3)
    @param n_samples_per_period (int): number of samples to generate for each period (default: 20)

    @return (np.ndarray): a 3xn_samples matrix containing the generated waypoints on the task space
    """
    # Calculate the angular frequency of the sine wave
    omega = 2 * np.pi * frequency

    # Calculate the number of periods
    n_periods = propagation_length * frequency

    # The number of samples is equal to the number of periods times the number of samples per period
    n_samples = int(math.ceil(n_periods * n_samples_per_period))

    # Generate a sine in the XY plane. 
    x = np.linspace(0, propagation_length, num=n_samples)
    y = amplitude * np.sin(omega * x)
    z = np.zeros_like(x) # z is always zero because the sine is defined in the XY plane
    
    points = np.stack((x, y, z), axis=0) # points is a 3xn_samples matrix
    # Rotate the points by the rotation matrix
    rotated_points = rotation_matrix @ points # rotated_points is a 3xn_samples matrix
    
    # Translate the points by the offset vector
    for j in range(n_samples):
        rotated_points[:,j] += offset
    
    #pose = Pose()
    #pose._position = (x_rot, y_rot, z_rot)
    
    return rotated_points


def generate_sine_one_period_path(amplitude, wavelength, offset, rotation_matrix, n_samples=20) -> np.ndarray:
    """
    @brief This functions generates waypoints in a three dimensional task space.
    The waypoints are sampled from a single period of sine function in the task space.
    The sine function is first defined on the XY plane by the amplitude and wavelength.
    Then the sine function is rotated by the rotation matrix and translated by the offset vector.
    @return (np.ndarray): a 3xn_samples matrix containing the generated waypoints on the task space
    """
    return generate_sine_path(amplitude, 1/wavelength, wavelength, offset, rotation_matrix, n_samples)

def main():
    # #Generate a sine path
    # A = 0.5  # Amplitude
    # wavelength = 1.0  # Wavelength in meters
    # rotation_matrix = np.eye(3)
    # sine_waypoints = generate_sine_path(A, 1, 1, 15.0, rotation_matrix, 20)
    # # Plot the generated path
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot(sine_waypoints[0,:], sine_waypoints[1,:], sine_waypoints[2,:])
    # plt.show()

    #Generate a sine path
    A = 0.5  # Amplitude
    wavelength = 1.0  # Wavelength in meters
    rotation_matrix = np.eye(3)
    sine_path_generator = SineOnePeriodPathGenerator()
    sine_path_generator.path_parameters['amplitude'].value = 0.5
    sine_path_generator.path_parameters['wavelength'].value = 6.5678
    sine_path_generator.path_parameters['propagation_axis_angle'].value = np.pi/2
    #sine_path_generator.path_parameters['propagation_length'].value = 15.0
    sine_waypoints = sine_path_generator.generate_path(900)
    # Plot the generated path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(sine_waypoints[0,:], sine_waypoints[1,:], sine_waypoints[2,:])
    plt.show()




if __name__ == '__main__':
    main()