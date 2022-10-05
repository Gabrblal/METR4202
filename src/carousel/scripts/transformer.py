import rospy as ros

from topics import Topics

class CarouselTransformer:
    """The carousel input transformation node.
    
    Responsible for cleaning input from the camera and joint state sources
    to put into the coordinate reference frame of the robot.
    
    Responsible for determining the maximum colour the camera is registering.
    """

    def __init__(self):
        """Create a new carousel transformation node."""
        self._box_pub = Topics.box.publisher()
        self._max_colour_pub = Topics.max_colour.publisher()

    def main(self):
        ros.spin()

if __name__ == '__main__':

    # TODO: Handle commandline arguments from launch file.
    CarouselTransformer().main()
