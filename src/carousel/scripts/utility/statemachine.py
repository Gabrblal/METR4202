import rospy as ros

class StateMachine:

    def __init__(self, first):
        """Initialises the sate machine with the first state"""
        self._state = first

    def spin(self):
        """Main loop of the state machine that iterates through states"""

        while not ros.is_shutdown():
            setattr(self._state, "machine", self)

            self._state.on_entry()
            next_state = self._state.main()
            self._state.on_exit()

            if next_state is None:
                break

            self._state = next_state

class State:

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(State, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit(self):
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass
