#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray


class pwmTestPythonPub(object):
    def __init__(self):
        # create publisher
        self._int32_array_pub = rospy.Publisher(
            "/command", Int32MultiArray, queue_size=10, latch=True)
        # create the msg
        self._int32_array_msg = Int32MultiArray()
        # fill the message with size -1
        for i in range(16):
            self._int32_array_msg.data.append(-1)

        # create rate 50 hz
        self._freq = 50
        self._rate = rospy.Rate(self._freq)

        self._start_range = 1000
        self._end_range = 8000
        self._step = 100  # each second
        self._dir = 1

        # choose num of the servo in the board
        self._pin = 5
        self._int32_array_msg.data[self._pin] = self._start_range

    def run(self):
        while not rospy.is_shutdown():
            # publish message
            self._int32_array_pub.publish(self._int32_array_msg)
            # check if we are over the maximum range reverse direction
            if self._int32_array_msg.data[self._pin] > self._end_range:
                self._int32_array_msg.data[self._pin] = self._end_range
                self._dir *= -1
            # check if we are lower the minimum range reverse direction
            if self._int32_array_msg.data[self._pin] < self._start_range:
                self._int32_array_msg.data[self._pin] = self._start_range
                self._dir *= -1
            # modify message
            if self._dir == 1:
                self._int32_array_msg.data[self._pin] += self._step / self._freq
            if self._dir == -1:
                self._int32_array_msg.data[self._pin] -= self._step / self._freq
            # sleep
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pwm_test_python_pub_node')
    rospy.loginfo("pwm_test_python_pub_node started ...")
    # Instantiate object
    pwm_python_pub_obj = pwmTestPythonPub()
    # run object
    pwm_python_pub_obj.run()
    # spin the node
    rospy.spin()
