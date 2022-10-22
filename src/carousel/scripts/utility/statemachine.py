import rospy as ros

class StateMachine:
    """An abstract state machine that loops through states, with each state
    returning the next state."""

    def __init__(self, first):
        """Initialises the sate machine with the first state."""
        self._state = first

    def spin(self):
        """Main loop of the state machine that iterates through states.

        Sets the machine attribute of the state, calls the State.on_entry(),
        State.main() and State.on_exit() in sequence, then goes to the state
        returned by State.main().

        Returns when State.main() returns None.
        """

        while not ros.is_shutdown():

            # Required to let states access persistent data across different
            # states.
            setattr(self._state, "machine", self)

            self._state.on_entry()
            next_state = self._state.main()
            self._state.on_exit()

            # If the next state does not exist then return.
            if next_state is None:
                break

            self._state = next_state

class State:

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to reference each other out of
        definition order with State.state.
        """
        setattr(State, cls.__name__, cls)

    def on_entry(self):
        """Optional method to call when a state is entered"""
        pass

    def on_exit(self):
        """Optional method to call when a state is exited"""
        pass

    def main(self):
        """Main method of the state.

        Returns:
            An instance of the next state to transition to.
        """
        pass
