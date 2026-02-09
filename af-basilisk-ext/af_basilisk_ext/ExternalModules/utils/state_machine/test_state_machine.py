from ExternalModules.utils.state_machine.state_machine_base import *


def test_declared_transition():
    """
    Tests whether the state machine sucessfully transitions between two states with a declared transition.
    """

    class State1(State):
        def update(self) -> str:
            return "State2"

    class State2(State):
        def update(self) -> str:
            return "State1"

    state1 = State1()
    state2 = State2()

    sm = StateMachine(state1)
    sm.add_state(state1)
    sm.add_state(state2)
    sm.add_transition(state1, state2)
    sm.update()

    assert isinstance(sm.current_state, State2)
    assert not isinstance(sm.current_state, State1)


def test_undeclared_transition():
    """
    Tests whether the state machine fails to transition between two states if their transition has been undeclared.
    """

    class State1(State):
        def update(self) -> str:
            return "State2"

    class State2(State):
        def update(self) -> str:
            return "State1"

    state1 = State1()
    state2 = State2()
    sm = StateMachine(state1)
    sm.add_state(state1)
    sm.add_state(state2)
    sm.update()

    assert not isinstance(sm.current_state, State2)
    assert isinstance(sm.current_state, State1)


def test_transition_to_undeclared_state():
    """
    Tests whether the state machine fails to obey the requested transition from a state
    if its transition has not been added to the state machine.
    """

    class State1(State):
        def update(self) -> str:
            return "State3"

    class State2(State):
        def update(self) -> str:
            return "State1"

    state1 = State1()
    state2 = State2()
    sm = StateMachine(state1)
    sm.add_state(state1)
    sm.add_state(state2)
    sm.add_transition(state1, state2)
    sm.update()

    assert isinstance(sm.current_state, State1)


if __name__ == "__main__":
    import sys

    sys.path.append("..")

    test_declared_transition()
    test_undeclared_transition()
    test_transition_to_undeclared_state()
