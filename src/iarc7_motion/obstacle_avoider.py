#!/usr/bin/env python

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
import threading

from iarc7_msgs.msg import OdometryArray
from geometry_msgs.msg import PointStamped

class ObstacleAvoider(object):
    def __init__(self):
        self._lock = threading.Lock()

        with self._lock:
            self._last_obst_msg = None
            self._obstacle_sub = rospy.Subscriber('/obstacles', OdometryArray, self._obst_callback)
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

            self._obst_avoidance_radius = rospy.get_param('~obst_avoidance_radius')
            self._obst_avoidance_time = rospy.get_param('~obst_avoidance_time')
            self._startup_timeout = rospy.Duration(rospy.get_param('~startup_timeout'))
            self._timeout = rospy.Duration(rospy.get_param('~timeout'))

    def get_safe_velocity(self, preferred_velocity):
        with self._lock:
            rospy.logwarn('Safe vel: {}'.format(preferred_velocity))
            return preferred_velocity

    def is_safe(self):
        ''' Return true if the current position is safe '''
        assert self._last_obst_msg is not None

        with self._lock:
            try:
                transform = self._tf_buffer.lookup_transform('map',
                                                             'level_quad',
                                                             rospy.Time.now(),
                                                             self._timeout)
            except Exception as ex:
                rospy.logerr('Unable to fetch transform in obstacle_detector.is_safe')
                rospy.logerr(ex)
                return False
            else:
                position = PointStamped()
                position = tf2_geometry_msgs.do_transform_point(position, transform)

                for obst in self._last_obst_msg.data:
                    if (math.hypot(position.point.x - obst.pose.pose.position.x,
                                   position.point.y - obst.pose.pose.position.y)
                            < self._obst_avoidance_radius):
                        return False
                return True

    def _obst_callback(self, msg):
        with self._lock:
            self._last_obst_msg = msg

    def wait_until_ready(self):
        time = rospy.Time.now()
        while (not rospy.is_shutdown()
           and rospy.Time.now() - time < self._startup_timeout
           and self._last_obst_msg is None):
            pass
        return self._last_obst_msg is not None
