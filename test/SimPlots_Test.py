#!/usr/bin/python
import unittest, logging, sys, os, time, subprocess, signal
import rospy, rostest, rospkg, rosbag_pandas
from controller_manager_msgs.srv import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from bobble_controllers.msg import ControlCommands
PKG = 'bobble_controllers'
NAME = 'SimPlots_Test'

class SimPlots_Test(unittest.TestCase):

    def setUp(self):
        self._Logger = logging.getLogger("sim_plots_test_log")
        self.using_gui = rospy.get_param('~using_gui', "False")
        # Give time for GUI to pop up before running tests
        if self.using_gui:
            time.sleep(20.0)
        try:
            self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            self.reset_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.apply_impulse_force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            self.set_link_properties = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
        except rospy.ServiceException, e:
            self._Logger.error("Service call failed: %s"%e)

    def resetState(self):
        control_cmd_publisher = rospy.Publisher('/bobble/bobble_balance_controller/bb_cmd', ControlCommands, queue_size=1)
        balance_cmd_msg = ControlCommands()
        balance_cmd_msg.IdleCmd = True
        balance_cmd_msg.StartupCmd = False
        balance_cmd_msg.DiagnosticCmd = False
        control_cmd_publisher.publish(balance_cmd_msg)
        return

    def activateController(self):
        self._Logger.warn("Commanding controller active.")
        control_cmd_publisher = rospy.Publisher('/bobble/bobble_balance_controller/bb_cmd', ControlCommands, queue_size=1)
        balance_cmd_msg = ControlCommands()
        balance_cmd_msg.StartupCmd = True
        balance_cmd_msg.IdleCmd = False
        control_cmd_publisher.publish(balance_cmd_msg)

    def startDataRecord(self):
        rospack = rospkg.RosPack()
        ros_bag_data_home = os.path.join(rospack.get_path("bobble_controllers"), "test", "bag_files")
        self.rosbag_process = subprocess.Popen('rosbag record -e /bobble/bobble_balance_controller/bb_controller_status -j -o {} --duration {}'.format("tilt_response", "10.0"), stdin=subprocess.PIPE, shell=True, cwd=ros_bag_data_home)

    def applyImpulse(self, forceX, cg=[0.0, 0.0, 0.180]):
        try:
            self.apply_impulse_force(
                                     body_name = "bobblebot::bobble_chassis_link",
                                     reference_frame = "bobblebot::bobble_chassis_link",
                                     reference_point = geometry_msgs.msg.Point(x=0.15, y=0.0, z=0.0),
                                     wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3(forceX, 0.0, 0.0),
                                                                       torque = geometry_msgs.msg.Vector3(0.0, 0.0, 0.0)),
                                     start_time = rospy.Time.from_sec(1.0),
                                     duration = rospy.Time.from_sec(0.001)
                                     )
        except rospy.ServiceException, e:
            self._Logger.error("Service call failed: %s"%e)

    def setMassProperties(self, com=[]):
        try:
            self.set_link_properties(
                                     link_name = "bobblebot",
                                     com = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(x=com[0], y=com[1], z=com[2]),
                                                                   orientation = geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
                                     mass = 2.043,
                                     ixx = 0.03,
                                     ixy = 0.0,
                                     ixz = 0.0,
                                     iyy = 0.03,
                                     iyz = 0.0,
                                     izz = 0.03
                                    )
        except rospy.ServiceException, e:
            self._Logger.error("Service call failed: %s"%e)

    def runImpulseForceTest(self, forceX=100.0):
        self._Logger.warn("Starting the sim dynamics.")
        time.sleep(3.0)
        self.resetState()
        time.sleep(3.0)
        self._Logger.warn("Activating controller.")
        self.activateController()
        time.sleep(0.25)
        self._Logger.warn("Starting data recording.")
        self.startDataRecord()
        time.sleep(1.0)
        self._Logger.warn("Applying impulse force in x direction of : " + str(forceX) + " Newtons.")
        self.applyImpulse(forceX)
        time.sleep(11.0)

    def test_ApplyImpulse(self):
        self.unpause_physics_client()
        num_runs = 1
        impulse_forces_x = [-700.0]
        for run in range(1, num_runs+1):
            impulse_force_x =impulse_forces_x[run-1]
            self._Logger.warn("Running impulse sim run : " + str(run))
            self.runImpulseForceTest(impulse_force_x)
            self.resetState()

if __name__ == '__main__':
    log = logging.getLogger("sim_plots_test_log")
    log.setLevel(logging.DEBUG)
    rospy.init_node('SimPlots_TestNode', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, NAME, SimPlots_Test, sys.argv)

