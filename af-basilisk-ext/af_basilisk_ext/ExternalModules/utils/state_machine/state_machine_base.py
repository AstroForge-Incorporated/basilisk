"""
Custom State Machine base classes, designed primarily to be extremely ergonomic, and allow state transitions to be extremely clear.
See the Basilisk `executive` folder for an example of how to use these base classes.
"""

from __future__ import annotations
import logging
import abc

from .utils import DefaultDict


class State(abc.ABC):
    __name: str = ""
    _instance = None

    def __new__(cls, *args) -> State:
        """
        @brief: Creates new state object, ensuring state is a singleton
        """
        if cls._instance is None:
            cls._instance = super(State, cls).__new__(cls, *args)
        return cls._instance

    def onEntry(self) -> None:
        pass

    def onExit(self) -> None:
        pass

    ## Decorate as an abstractmethod, to ensure state instances override this method
    @abc.abstractmethod
    def update(self) -> str:
        pass

    @property
    def name(self) -> str:
        return self.__class__.__name__


class StateHolder:
    def __init__(self):
        self.state_dict: dict[str, State] = {}

    def add_state(self, state: State) -> None:
        if state.name in self.state_dict.keys():
            raise Exception(
                "A state with name {} has already been added to the state machine."
                "States should have unique names.".format(state.name)
            )
        else:
            self.state_dict[state.name] = state

    def __getitem__(self, key: str) -> State:
        if key not in self.state_dict.keys():
            raise Exception(
                "Attempted to pull {} from state dict, but did not find a key with this name.".format(
                    key
                )
            )
        return self.state_dict[key]

    def __contains__(self, key: str) -> bool:
        return key in self.state_dict.keys()

    def __repr__(self):
        return repr(self.state_dict)


class StateMachine:
    def __init__(self, default_state: State):
        self.valid_states: StateHolder = StateHolder()
        ## Note: We probably don't need the custom DefaultDict class here; defaultdict(lambda: []) should do the trick.
        self.transitions: DefaultDict[State, list[State]] = DefaultDict([])
        self.current_state: State = default_state

    def update(self):
        requested_state_name: str = self.current_state.update()
        ## Do not transition if requested state is the same as current state
        if requested_state_name == self.current_state.name:
            return
        else:
            ## Ignore if requested state name not in self.valid_states
            if requested_state_name not in self.valid_states:
                logging.warning(
                    "State {} requested transition \
                                to an undeclared state {}. Ignoring.".format(
                        self.current_state.name, requested_state_name
                    )
                )

                return
            ## Grab requested state from valid_states
            requested_state: State = self.valid_states[requested_state_name]

            ## Check if is valid transition
            ## If invalid, warn, and return
            if requested_state not in self.transitions[self.current_state]:
                logging.warning(
                    "State {} requested transition to State {}, "
                    "which is not a valid transition. Ignoring.".format(
                        self.current_state.name, requested_state_name
                    )
                )
                return

            ## Trigger current state onExit()
            self.current_state.onExit()

            ## Switch states
            self.current_state = requested_state

            ## Trigger new state onEntry()
            self.current_state.onEntry()
            return

    def add_state(self, state: State) -> None:
        if state.name in self.valid_states:
            logging.warning(
                "State {} already in state machine. Ignoring.".format(state.name)
            )
            return

        self.valid_states.add_state(state)

    def add_transition(self, state_1: State, state_2: State) -> None:
        """
        @brief: Add transition from state_1 to state_2
        """
        if (
            state_1.name not in self.valid_states
            or state_2.name not in self.valid_states
        ):
            raise Exception(
                "Cannot add transition between states {}"
                "and {} as either state {} or state {}"
                "is has not been added to the  state machine."
                "Ignoring.".format(
                    state_1.name, state_2.name, state_1.name, state_2.name
                )
            )

        self.transitions[state_1].append(state_2)

    def set_state(self, state: State) -> None:
        """
        @brief: Sets state of state machine. Fails if state
        has not been added to the state machine.
        """
        if state.name not in self.valid_states:
            raise Exception(
                "Cannot set state: {}, as state has not \
                            been added to the state machine".format(
                    state.name
                )
            )
        self.current_state = state
