import sys
import rospy as ros
import pigpio

from std_msgs.msg import Float32

class CarouselGripper:

    def __init__(
            self,
            minimum : int = 1000,
            maximum : int = 2000,
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
        self._pi.set_servo_pulsewidth(pin,1000) 

        # Subscribe to gripper commands.
        self._box_sub = ros.Subscriber(gripper_topic, Float32, self._callback)

    def _open(self, percentage = None, /):
        """Opens the gripper to maximum extension.

        Args:
            percentage: If not None then the percentage to open the gripper
                between 0 and 1 inclusive."""
        pass

    def _callback(self, message : Float32):
        """Callback function for commands to the gripper.
        
        Args:
            message: The percentage to set the gripper to.
        """
        pass

    def main(self):
        ros.spin()

if __name__ == '__main__':

    # TODO: Parse commandline arguments for minimum, maximum pwm and raspberry
    # pi pin number.

    n = len(sys.argv)

    CarouselGripper().main()
