from abc import ABC, abstractmethod
from typing import Tuple, Iterable, Generic, TypeVar
from lle import World, Action, WorldState


T = TypeVar("T")


class SearchProblem(ABC, Generic[T]):
    """
    A Search Problem is a problem that can be solved by a search algorithm.

    The generic parameter T is the type of the problem state, which must inherit from WorldState.
    """

    def __init__(self, world: World):
        self.world = world
        world.reset()
        self.initial_state = world.get_state()
        self.nodes_expanded = 0

    @abstractmethod
    def is_goal_state(self, problem_state: T) -> bool:
        """Whether the given state is the goal state"""

    @abstractmethod
    def get_successors(self, state: T) -> Iterable[Tuple[T, Tuple[Action, ...], float]]:
        """
        Yield all possible states that can be reached from the given world state.
        Returns
            - the new problem state
            - the joint action that was taken to reach it
            - the cost of taking the action
        """

    def heuristic(self, problem_state: T) -> float:
        return 0.0


class SimpleSearchProblem(SearchProblem[WorldState]):

    def is_goal_state(self, state: WorldState) -> bool:
        exit_pos = set(self.world.exit_pos)
        return (set(state.agents_positions) | exit_pos) == exit_pos 

    def get_successors(self, state: WorldState) -> Iterable[Tuple[WorldState, Tuple[Action, ...], float]]:
        # - N'oubliez pas de jeter un oeil aux méthodes de la classe World (set_state, done, step, available_actions, ...)
        # - Vous aurez aussi peut-être besoin de `from itertools import product`        
        sol = [] # [(State1, (action,), 1.0), (State2, (action,), 1.0), ...)]
        
        print(state.agents_positions)
        print(self.world.available_actions())

        for actions in self.world.available_actions():
            partial_sol = []
            for action in actions:
                self.world.set_state(state)
                self.world.step([action])
                partial_sol.append((self.world.get_state(), actions, 1.0))
            sol.append(partial_sol)
        
        print(sol)

        self.nodes_expanded += 1
        return sol


    def heuristic(self, state: WorldState) -> float:
        """Manhattan distance for each agent to its goal"""
        raise NotImplementedError()


class CornerProblemState:
    ...


class CornerSearchProblem(SearchProblem[CornerProblemState]):
    def __init__(self, world: World):
        super().__init__(world)
        self.corners = [(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)]
        self.initial_state = ...

    def is_goal_state(self, state: CornerProblemState) -> bool:
        raise NotImplementedError()

    def heuristic(self, problem_state: CornerProblemState) -> float:
        raise NotImplementedError()

    def get_successors(self, state: CornerProblemState) -> Iterable[Tuple[CornerProblemState, Action, float]]:
        self.nodes_expanded += 1
        raise NotImplementedError()


class GemProblemState:
    ...


class GemSearchProblem(SearchProblem[GemProblemState]):
    def __init__(self, world: World):
        super().__init__(world)
        self.initial_state = ...

    def is_goal_state(self, state: GemProblemState) -> bool:
        raise NotImplementedError()

    def heuristic(self, state: GemProblemState) -> float:
        """The number of uncollected gems"""
        raise NotImplementedError()

    def get_successors(self, state: GemProblemState) -> Iterable[Tuple[GemProblemState, Action, float]]:
        self.nodes_expanded += 1
        raise NotImplementedError()
