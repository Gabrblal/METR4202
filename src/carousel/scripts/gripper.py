#! /usr/bin/python3

import sys
import rospy as ros
import pigpio

from std_msgs.msg import Float32

class CarouselGripper:

    def __init__(
            self,
            minimum : int = 1280,
            maximum : int = 1900,
            pin : int = 18,
            gripper_topic : str = 'carousel_gripper'
        ):
        """Instantiate the carousel gripper from a provided servo motor
        configuration and topic to receive commands on.

        Args:
            minimum: The pwm corresponding to the gripper being closed.
            maximum: The pwm corresponding to the gripper being open.
            pin: The pin to send the pwm signal on.
            gripper_topic: The topic to subscribe to for gripper commands.
                Expects a percentage between 0 and 1 inclusive.
        """
        self._minimum_pwm = minimum
        self._maximum_pwm = maximum
        self._pin = pin

        # Setup pwn on the provided pin.
        self._pi = pigpio.pi()
        self._pi.set_mode(self._pin, pigpio.OUTPUT)
        self._pi.set_servo_pulsewidth(pin, maximum) 

        # Subscribe to gripper commands.
        self._box_sub = ros.Subscriber(gripper_topic, Float32, self._callback)

    def _open(self, percentage, /):
        """Opens the gripper to maximum extension .

        Args:
            percentage: If not None then the percentage to open the gripper
                between 0 and 1 inclusive."""
        pulsewidth = self._minimum_pwm + (self._maximum_pwm - self._minimum_pwm)*percentage
        self._pi.set_servo_pulsewidth(self._pin, pulsewidth)

        ros.loginfo(f"Gripper percentage = {percentage} -> pwm = {pulsewidth}")

    def _callback(self, message : Float32):
        """Callback function for commands to the gripper, gets the message,
         checks it is valid, passes the percentage to _open.

        Args:
            message: The percentage to set the gripper to.
        """
        percent = message.data
        if percent < 0:
            percent = 0
        elif percent > 1:
            percent = 1

        self._open(percent)

    def main(self):
        ros.spin()

if __name__ == '__main__':

    # TODO: Parse commandline arguments for minimum, maximum pwm and raspberry
    # pi pin number.

    n = len(sys.argv)
    ros.init_node('CarouselGripperNode')
    CarouselGripper().main()
