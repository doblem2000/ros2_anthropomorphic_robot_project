# This module contains the OuroborosGui class, which is a GUI for the Ouroboros node.
# This GUI was heavily inspired by the joint_state_publisher_gui.py file from the official ros2 humble's 'joint_state_publisher_gui' package.
# For reference, the original file can be found here:
# https://github.com/ros/joint_state_publisher/blob/ros2/joint_state_publisher_gui/joint_state_publisher_gui/joint_state_publisher_gui.py

import argparse
import random
import signal
import sys
import threading
import yaml

import rclpy
from rclpy.parameter import Parameter

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtCore import pyqtSignal as Signal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QFileDialog

import numpy as np

from task_space_path_generator.flow_layout import FlowLayout

from task_space_path_generator.path_generators import PathType, SinePathGenerator, SineOnePeriodPathGenerator, PATH_PARAMETERS, PathGenerator, PathParameter

from task_space_path_generator.ouroboros import Ouroboros

RANGE = 10000 # < The range of the sliders
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # < Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11 
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SLIDER_HEIGHT = 64  # < The combination of default heights in Slider

# Calculate default minimums for window sizing
MIN_WIDTH = SLIDER_WIDTH + DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2

TRANSFORM_SLIDERS_CONSTRAINTS = {
    'x': {'lower_bound': -2, 'upper_bound': 2},
    'y': {'lower_bound': -2, 'upper_bound': 2},
    'z': {'lower_bound': -2, 'upper_bound': 2},
    'roll': {'lower_bound': -3.14, 'upper_bound': 3.14},
    'pitch': {'lower_bound': -3.14, 'upper_bound': 3.14},
    'yaw': {'lower_bound': -3.14, 'upper_bound': 3.14}
}

# The following class is the same class from the joint_state_publisher_gui.py file
class Slider(QWidget):
    """
    @brief Class for creating a slider widget
    """
    def __init__(self, name):
        """
        @brief Create a new slider widget
        @param name: The name of the joint
        """
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, RANGE)
        self.slider.setValue(int(RANGE / 2))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)

        self.setLayout(self.joint_layout)

    def remove(self):
        """
        @brief Remove the slider widget from the layout
        """
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)


class SliderGroup(QScrollArea):
    """
    @brief Class for creating a group of sliders
    """
    def __init__(self):
        """
        @brief Create a new slider group widget
        """
        super().__init__()
        
        self._sliders = {}
        
        # Scroll area widget contents
        self.slider_group_widget = QWidget()
        self.slider_group_widget.setLayout(FlowLayout())

        # Scroll area for sliders
        self.setWidgetResizable(True)
        self.setWidget(self.slider_group_widget)

    def remove(self):
        """
        @brief Remove the slider group widget from the layout
        """
        self.removeAllSliders()
        self.slider_group_widget.setParent(None)
        self.slider_group_widget = None
        self.setParent(None)
    
    def addSlider(self, slider):
        """
        @brief Add a slider to the group
        @param slider: The slider to add
        """
        self._sliders[slider] = None
        self.slider_group_widget.layout().addWidget(slider)
        
    def removeSlider(self, slider):
        """
        @brief Remove a slider from the group
        @param slider: The slider to remove
        """
        self.slider_group_widget.layout().removeWidget(slider)
        slider.setParent(None)
        slider.remove()
        self._sliders.pop(slider)
        
    def removeAllSliders(self):
        """
        @brief Remove all sliders from the group
        """
        for slider in self._sliders:
            self.slider_group_widget.layout().removeWidget(slider)
            slider.setParent(None)
            slider.remove()
        self._sliders = {}

class OuroborosGui(QMainWindow):
    """
    @brief Class for creating the Ouroboros GUI
    """
    sliderUpdateTrigger = Signal()
    #initialize = Signal()

    def __init__(self, title, node):
        """
        @brief Create a new Ouroboros GUI
        """
        super(OuroborosGui, self).__init__()
        self.setWindowTitle(title)
        
        # The ouroboros_node is the reference to the Ouroboros node.
        self.ouroboros_node = node
        # The path_generator is the object that is used to generate the path.
        self.path_generator: PathGenerator = None
        # The path_parameters_map is used to keep track of the sliders of each path parameter
        self.path_parameters_map = {}
        # The transform_sliders_map is used to keep track of the sliders of each transform parameter
        self.transform_parameters_map = {}
        

        # Create combo box for selecting the desired path type
        self.combo_box_path_type = QComboBox()
        self.combo_box_path_type.addItems(Ouroboros.get_supported_generators())
        self.combo_box_path_type.currentTextChanged.connect(self.onPathTypeChanged)

        # Create combo box for selecting the frame according to which the path is generated
        self.combo_box_base_frame = QComboBox()
        self._avaiable_frames = {}
        self.updateAvailableFrames()
        self.update_avaialable_frames_timer = QTimer(self)
        self.update_avaialable_frames_timer.timeout.connect(self.updateAvailableFrames)
        self.update_avaialable_frames_timer.start(500)
        self.combo_box_base_frame.currentTextChanged.connect(self.onBaseFrameChanged)
        
        # Button for exporting the path to a bagfile
        self.export_button = QPushButton('Export path...', self)
        self.export_button.clicked.connect(self.exportEvent)

        # Button for randomizing the sliders
        #self.rand_button = QPushButton('Randomize', self)
        #self.rand_button.clicked.connect(self.randomizeEvent)

        # Button for centering the sliders
        #self.ctr_button = QPushButton('Center', self)
        #self.ctr_button.clicked.connect(self.centerEvent)

        # The following code is meant to configure the Qt5 GUI. The hierarchy of the GUI is as follows:
        # central_widget -> scroll_area -> scroll_widget -> scroll_layout -> sliders

        self.path_parameters_sliders = SliderGroup()
        self.transform_sliders = SliderGroup()
        
        for transform_parameter in TRANSFORM_SLIDERS_CONSTRAINTS.keys():
            slider = Slider(transform_parameter)
            parameter = PathParameter(transform_parameter, value=0.0, lower_bound=TRANSFORM_SLIDERS_CONSTRAINTS[transform_parameter]['lower_bound'], upper_bound=TRANSFORM_SLIDERS_CONSTRAINTS[transform_parameter]['upper_bound'])
            self.transform_parameters_map[transform_parameter] = {
                'display': slider.display,
                'slider': slider.slider,
                'parameter': parameter
            }
            self.transform_sliders.addSlider(slider)
            if transform_parameter in ['x', 'y', 'z']:
                slider.slider.valueChanged.connect(lambda event, name = parameter.name: self.onTransformOffsetValueChanged(name))
            else:
                slider.slider.valueChanged.connect(lambda event, name = parameter.name: self.onTransformRotationValueChanged(name))
            

        # Add buttons and scroll area to main layout
        #self.main_layout.addWidget(self.rand_button)
        #self.main_layout.addWidget(self.ctr_button)

        # Set the layout of the central widget
        self.main_layout = QVBoxLayout()
        # Add path type combo box to main layout
        self.main_layout.addWidget(self.combo_box_path_type)
        # Add scroll area to main layout
        self.main_layout.addWidget(self.path_parameters_sliders)
        # Add base frame combo box to main layout
        self.main_layout.addWidget(self.combo_box_base_frame)
        # Add transform sliders to main layout
        self.main_layout.addWidget(self.transform_sliders)
        # Add export button to main layout
        self.main_layout.addWidget(self.export_button)

        # Set the central widget of the Window. This is the root widget that holds all other widgets
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        


        #self.jsp = jsp
        #self.jsp.set_source_update_cb(self.sliderUpdateCb)
        #self.jsp.set_robot_description_update_cb(self.initializeCb)

        self.running = True
        
        # The slider dictionary is used to keep track of the sliders that are currently being displayed
        self.sliders = {}

        # Setup signal for initializing the window
        #self.initialize.connect(self.initializeSliders)
        # Set up a signal for updating the sliders based on external joint info
        #self.sliderUpdateTrigger.connect(self.updateSliders)

        # Tell self to draw sliders in case the JointStatePublisher already has a robot_description
        #self.initialize.emit()
        
        # Initialize the path generator to the first path type
        self.onPathTypeChanged(self.combo_box_path_type.currentText())
        # Set the size of the window
        self.updateWindowSize()

    def onPathTypeChanged(self, path_type):
        """
        @brief This method is called when the path type is changed. It updates the path generator and the sliders.
        @param path_type: The new path type
        """
        self.ouroboros_node.path_generator_name = path_type
        self.path_generator = self.ouroboros_node.path_generator # Get the path generator from the node
        
        # Remove all sliders from the GUI
        self.path_parameters_sliders.removeAllSliders()
        self.sliders = {}
        
        for parameter in self.path_generator.path_parameters.values():

            # if parameter.lower_bound >= parameter.upper_bound:
            #     continue
            slider = Slider(parameter.name)
            
            # We keep a map of the sliders of each parameter so that we can easily access them later
            self.path_parameters_map[parameter.name] = {'display': slider.display, 'slider': slider.slider, 'parameter': parameter}

            # Add the slider to the GUI
            self.path_parameters_sliders.addSlider(slider)
            
            # Connect to the signal provided by QSignal such that we can update the PathParameter object when the slider value changes
            slider.slider.valueChanged.connect(lambda event, name = parameter.name: self.onSliderValueChangedOne(name))

            # Keep a reference to the slider so that we can remove it later
            self.sliders[slider] = slider
            
        # Set the slider values to the current values of the PathParameter objects
        self.updateSliders()

        self.onTransformOffsetValueChanged(None)
        self.onTransformRotationValueChanged(None)
        
    def onBaseFrameChanged(self, base_frame):
        """
        @brief This method is called when the base frame is changed. It updates the base frame of the path generator.
        @param base_frame: The new base frame
        """
        self.ouroboros_node.draw_base_frame = base_frame
        #self.ouroboros_node.update_tf()
    
    def onSliderValueChangedOne(self, name):
        """
        @brief This method is called when a slider value was changed and we need to update the corresponding PathParameter object.
        @param name: The name of the slider that was changed
        """
        # First we get the slider value and the PathParameter object from the map.
        parameter_info = self.path_parameters_map[name]
        slider_value = parameter_info['slider'].value()
        parameter = parameter_info['parameter']
        
        # Then we update the PathParameter object with the new value.
        parameter.value = OuroborosGui.sliderToValue(slider_value, parameter)
        
        # Finally, we update the display of the PathParameter object.
        parameter_info['display'].setText("%.3f" % parameter.value)
        
        # Set size of min size of window based on number of the currently active sliders.
        self.updateWindowSize()

    def onTransformOffsetValueChanged(self, name):
        """
        @brief This method is called when a transform offset slider value was changed and we need to update the corresponding transform offset.
        @param name: The name of the slider that was changed
        """
        if self.path_generator is None:
            return
        x_slider_value = self.transform_parameters_map['x']['slider'].value()
        y_slider_value = self.transform_parameters_map['y']['slider'].value()
        z_slider_value = self.transform_parameters_map['z']['slider'].value()
        x_value = OuroborosGui.sliderToValue(x_slider_value, self.transform_parameters_map['x']['parameter'])
        y_value = OuroborosGui.sliderToValue(y_slider_value, self.transform_parameters_map['y']['parameter'])
        z_value = OuroborosGui.sliderToValue(z_slider_value, self.transform_parameters_map['z']['parameter'])
        self.transform_parameters_map['x']['display'].setText("%.3f" % x_value)
        self.transform_parameters_map['y']['display'].setText("%.3f" % y_value)
        self.transform_parameters_map['z']['display'].setText("%.3f" % z_value)
        self.path_generator._offset = np.array([x_value, y_value, z_value])
        #self.ouroboros_node.update_tf()

    def onTransformRotationValueChanged(self, name):
        """
        @brief This method is called when a tranform rotation slider value was changed and we need to update the corresponding transform rotation.
        @param name: The name of the slider that was changed
        """
        if self.path_generator is None:
            return
        roll_slider_value = self.transform_parameters_map['roll']['slider'].value()
        pitch_slider_value = self.transform_parameters_map['pitch']['slider'].value()
        yaw_slider_value = self.transform_parameters_map['yaw']['slider'].value()
        roll_value = OuroborosGui.sliderToValue(roll_slider_value, self.transform_parameters_map['roll']['parameter'])
        pitch_value = OuroborosGui.sliderToValue(pitch_slider_value, self.transform_parameters_map['pitch']['parameter'])
        yaw_value = OuroborosGui.sliderToValue(yaw_slider_value, self.transform_parameters_map['yaw']['parameter'])
        self.transform_parameters_map['roll']['display'].setText("%.3f" % roll_value)
        self.transform_parameters_map['pitch']['display'].setText("%.3f" % pitch_value)
        self.transform_parameters_map['yaw']['display'].setText("%.3f" % yaw_value)
        self.path_generator._rotation_rpy = np.array([roll_value, pitch_value, yaw_value])
        #self.ouroboros_node.update_tf()

    def updateSliders(self):
        """
        @brief This method is called when we need to update the sliders based on the PathParameter objects.
        It is the inverse of the onSliderValueChangedOne method.
        """
        for parameter_info in self.path_parameters_map.values():
            parameter = parameter_info['parameter']
            slider_value = self.valueToSlider(parameter.value, parameter)
            parameter_info['slider'].setValue(slider_value)
    
    @staticmethod
    def sliderToValue(slider, parameter):
        """
        @brief This method is used to convert a slider value to a PathParameter value.
        @param slider: The slider value
        """
        slider_percentage = slider / float(RANGE)
        return parameter.lower_bound + (parameter.upper_bound - parameter.lower_bound) * slider_percentage

    @staticmethod
    def valueToSlider(value, parameter):
        """
        @brief This method is used to convert a PathParameter value to a slider value.
        @param value: The PathParameter value
        """
        value_percentage = (value - parameter.lower_bound) / (parameter.upper_bound - parameter.lower_bound)
        return int(value_percentage * float(RANGE))

    def updateWindowSize(self):
        """
        @brief This method is used to update the size of the window based on the number of active sliders.
        """
        if len(self.sliders) >= INIT_NUM_SLIDERS:  # Limits min size to show INIT_NUM_SLIDERS
            num_sliders = INIT_NUM_SLIDERS
        else:
            num_sliders = len(self.sliders)
        scroll_layout_height = num_sliders * DEFAULT_SLIDER_HEIGHT
        scroll_layout_height += (num_sliders + 1) * DEFAULT_CHILD_MARGIN
        self.setMinimumSize(MIN_WIDTH, scroll_layout_height + MIN_HEIGHT)

    def updateAvailableFrames(self):
        """
        @brief This method is used to update the available frames in the base frame combo box.
        """
        avaiable_frames = yaml.load(self.ouroboros_node.all_frames_yaml, Loader=yaml.BaseLoader)
        if type(avaiable_frames) is not dict:
            return
        avaiable_frames.pop('cartesian_path_frame') # Remove the cartesian_path_frame from the list of available frames
        for frame, frame_info in avaiable_frames.items():
            if frame not in self._avaiable_frames or frame_info['parent'] not in self._avaiable_frames:
                self._avaiable_frames = dict.fromkeys(avaiable_frames, None)
                self._avaiable_frames.update({x['parent']: None for x in avaiable_frames.values()})
                self.combo_box_base_frame.clear()
                self.combo_box_base_frame.addItems(self._avaiable_frames)
                discovered_frames = ', '.join([x for x,y in self._avaiable_frames.items()])
                self.ouroboros_node.get_logger().info("Discovered new frames: %s" % discovered_frames)
                break
        
    def loop(self):
        """
        @brief This method is used to keep the GUI running.
        """
        while self.running:
            rclpy.spin_once(self.ouroboros_node, timeout_sec=0.1)

    def exportEvent(self, event):
        """
        @brief This method is called when the export button is clicked. It exports the path to a bagfile.
        @param event: The event that triggered the method
        """
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        path, _ = QFileDialog.getSaveFileName(self, 'Export path', '', 'ROS2 bagfile (*.bag)', options=options)
        if path:
            if path[-4:] != '.bag':
                path += '.bag'
            self.ouroboros_node.export_path(path)




def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    app = QApplication(sys.argv)
    ouroboros_node = Ouroboros()
    ouroboros_gui = OuroborosGui('Ouroboros', ouroboros_node)

    ouroboros_gui.show()

    threading.Thread(target=ouroboros_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
