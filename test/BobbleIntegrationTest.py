#!/usr/bin/python
import unittest, time, sys
import rospy, rostest
PKG = 'bobble_controllers'
NAME = 'BobbleIntegrationTest'

class BobbleIntegrationTest(unittest.TestCase):

    def setUp(self):
        run_length_s = rospy.get_param('~run_length_s', "30.0")
        time.sleep(run_length_s)

    def test_IntegrationTestResults(self):
        # For now there's nothing to do. The other nodes and rosbag play/record
        # do all the heavy lifting of the integration tests. This is just here
        # to let the CI know the test passed if it makes it this far w/o error.
        # In future, we could generate plots from in here.
        self.assertTrue(True)

if __name__ == '__main__':
    rospy.init_node('BobbleIntegrationTest_Node', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, NAME, BobbleIntegrationTest, sys.argv)

