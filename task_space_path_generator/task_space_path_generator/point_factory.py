"""
This module provides a PointFactory class for creating ROS Point messages from different types of arguments.
"""
import numpy as np
from geometry_msgs.msg import Point, Pose

class PointFactory:
    """@brief This class is used to create (list of) ROS Point messages from different types of arguments."""
    @staticmethod
    def build_point(*args):
        """
        @brief This method is a wrapper for the ROS Point message. It is used to provide a more intuitive interface for the user.
        It supports the following parameter types:
            - PointFactory()
            - PointFactory(Point)
            - PointFactory(Pose)
            - PointFactory(numpy.ndarray)
            - PointFactory(float, float, float)
        """
        point = Point()
        if len(args) == 0:
            point.x = 0.0
            point.y = 0.0
            point.z = 0.0
        
        elif len(args) == 1:
            if type(args[0]) == Point:
                point.x = args[0].x
                point.y = args[0].y
                point.z = args[0].z
            elif type(args[0]) == Pose:
                point.x = args[0].position.x
                point.y = args[0].position.y
                point.z = args[0].position.z
            elif type(args[0]) == np.ndarray or type(args[0]) == np.matrix:
                # print("DEBUG: ndarray")
                if args[0].shape != (3,1) and args[0].shape != (3,) and args[0].shape != (1,3):
                    raise TypeError("Invalid shape '%s' for build_point()" % str(args[0].shape))
                if args[0].dtype != np.float32 and args[0].dtype != np.float64:
                    raise TypeError("Invalid dtype '%s' for build_point()" % str(args[0].dtype))
                # print("DEBUG: checks ok, shape '%s', dtype '%s'" % (str(args[0].shape), str(args[0].dtype)))
                if args[0].shape == (3,1):
                    # print("DEBUG: shape (3,1)")
                    point.x = args[0][0,0].item()
                    point.y = args[0][1,0].item()
                    point.z = args[0][2,0].item()
                elif args[0].shape == (1,3):
                    # print("DEBUG: shape (1,3)")
                    # print(type(args[0][0,0]), args[0][0,1], args[0][0,2])
                    point.x = args[0][0,0].item()
                    point.y = args[0][0,1].item()
                    point.z = args[0][0,2].item()
                else:
                    # print("DEBUG: shape (3,)")
                    point.x = args[0][0].item()
                    point.y = args[0][1].item()
                    point.z = args[0][2].item()
            else:
                raise TypeError("Unsupported type '%s' for build_point()" % type(args[0]).__name__)
            
        elif len(args) == 3 and type(args[0]) == float and type(args[1]) == float and type(args[2]) == float:
            point.x = args[0]
            point.y = args[1]
            point.z = args[2]
            
        else:
            raise TypeError("Invalid number of arguments for build_point()")
        
        return point
    
    @staticmethod
    def build_point_list(*args):
        """
        @brief This method is a builder for a list of ROS Point messages. It is used to provide a more intuitive interface for the user.
        """
        points_list = []
        if len(args)==0:
            return points_list
        
        elif len(args)==1:
            if type(args[0]) == np.ndarray or type(args[0]) == np.matrix:
                if args[0].shape[0] != 3:
                    raise TypeError("Invalid shape '%s' for build_point_list(). First dimension must be 3. " % str(args[0].shape))
                if args[0].dtype != np.float32 and args[0].dtype != np.float64:
                    raise TypeError("Invalid dtype '%s' for build_point_list()" % str(args[0].dtype))
                # print("DEBUG: ", args[0].shape[1])
                points_list = np.apply_along_axis(PointFactory.build_point, 0, args[0])
                if type(points_list) != list:
                    points_list = points_list.tolist()
                # print("DEBUG: ", points_list)
                return points_list
            else:
                raise TypeError("Unsupported type '%s' for build_point_list(). '%s'" % type(args[0]).__name__)
        else:
            raise TypeError("Invalid number of arguments for build_point_list()")