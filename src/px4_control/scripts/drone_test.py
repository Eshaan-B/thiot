#!/usr/bin/env python3
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
import cws
import random
import itertools
import re
from collections import OrderedDict
trees_dict = OrderedDict()

def distance (city1, city2):
    return int(math.sqrt(math.pow(city1[0] - city2[0], 2) + math.pow(city1[1] - city2[1], 2)))

'''
def random_customer (id):
    #print (random.randint(0, 100), random.randint(0, 100))
    return Customer(id, (random.randint(0, 100), random.randint(0, 100)))
'''
def random_customer(id):
    #print (trees_dict[id], trees_dict[id][0], trees_dict[id][1])
    return Customer(id, (trees_dict[id][0], trees_dict[id][1]))

def create_dict():
    with open ("/home/ebhardy/quad_ws/src/px4_control/scripts/location.txt") as f:
        for line in f:
            line = line.strip("\n").strip(";")
            word = line.split()
            val_x = re.findall('-?\d+', word[1])
            val_y = re.findall('-?\d+', word[2])
            #print (re.findall("\d+\.\d+", word[1]))
            x_coord = int(val_x[0])
            y_coord = int(val_y[0])

            trees_dict[word[0]]  = [x_coord, y_coord]
            #print (word)
    f.close()

    trees_dict['depot'] = [0 , 0]
    #print (len(trees_dict))
    '''
    for key, value in trees_dict.items():
        print(key, value)
    '''



def get_streets (customers):
    streets = []
    for i, j in itertools.combinations(customers, 2):
        saving = i.nd_edge.cost + j.dn_edge.cost - distance(i.city, j.city)
        cost = distance(i.city, j.city)
        s = Street(i, j, saving, cost)
        s_inverse = Street(j, i, saving, cost)
        s.inverse, s_inverse.inverse = s_inverse, s
        streets.append(s)
    return tuple(streets)


depot = (0, 0)


class Street (cws.Edge):
    pass


class Customer (cws.Node, object):
    def __init__(self, id, city):
        self.city = city
        dn_edge = Street("depot", self, 0, cost=distance(depot, city))
        nd_edge = Street(self, "depot", 0, cost=distance(city, depot))
        dn_edge.inverse = nd_edge
        nd_edge.inverse = dn_edge
        super(Customer, self).__init__(id, dn_edge, nd_edge)


create_dict()
#print ("here", len(trees_dict))
customers = tuple(random_customer(i) for i in trees_dict)
#print ("here", customers)


streets = get_streets(customers)
#print (streets, '\n')

class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.radius = 0.25

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))

        rospy.loginfo("euclidean distance: {0}, offset: {1}, distance < offset: {2}".format (np.linalg.norm(desired - pos), offset, np.linalg.norm(desired - pos) < offset))
        #rospy.loginfo(np.linalg.norm(desired - pos))
        #rospy.loginfo(offset)
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False

        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("\n\n\nposition reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        print("Program...")
        '''
        config = cws.CWSConfiguration(
            biased = True,
            reverse = True,
            metaheuristic = True,
            start = None,
            maxiter = 1000,
            maxnoimp = 500,
            maxcost = float("inf"),
            minroutes = 1, # was 5
        )
        '''
        solver = cws.ClarkeWrightSavings(nodes=customers, edges=streets)
        sol, cost = solver.__call__()
        print("done")

        #print ("Routes: ", sol, "\n")
        # getting keys/ best edges
        str_val = sol
        str_val = str_val.split(",")

        start_customers = []
        end_customers = []
        for r in range(len(str_val)):
            value_str = str_val[r].lstrip()
            value_str = value_str.split(" ")
            start_customers.append(str(value_str[0]).replace('[','').replace("(",""))
            end_customers.append(str(value_str[2]).replace(']','').replace(")",""))

        # keys
        #start_customers.append(end_customers[-1])

        positions = []
        height = 2.5
        # get values
        for r in range(len(start_customers)):
            #print (start_customers[r], trees_dict.get(start_customers[r]), trees_dict.get(start_customers[r])[0], trees_dict.get(start_customers[r])[1])
            positions.append(tuple([trees_dict.get(start_customers[r])[0], trees_dict.get(start_customers[r])[1], height]))

        positions = tuple(positions)

        rospy.loginfo("\n\nRoutes(ClarkeWrightSavings):\n")
        rospy.loginfo(positions)
        rospy.loginfo("\n\n\n")


        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 60) # was 30

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
