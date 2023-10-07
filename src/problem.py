from abc import ABC, abstractmethod
from typing import Tuple, Iterable, Generic, TypeVar
from lle import World, Action, WorldState
from frontier import override, Node

from itertools import product

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

	@staticmethod
	def _manhattan_distance(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
		return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

	def f(self, problem_state: T, cost: float):
		return self.g(problem_state, cost) + self.heuristic(problem_state)

	def g(self, problem_state: T, cost: float) -> float:
		"""The cost of reaching the given state"""
		return 1 - cost

	def heuristic(self, state: T) -> float:
		"""Manhattan distance for each agent to the closest exit"""
		return max(min(self._manhattan_distance(agent, exit) for exit in self.world.exit_pos) for agent in state.agents_positions)


class SimpleSearchProblem(SearchProblem[WorldState]):

	@override(SearchProblem)
	def is_goal_state(self, state: WorldState) -> bool:
		self.world.set_state(state)
		return self.world.exit_rate == 1.0

	@override(SearchProblem)
	def get_successors(self, state: WorldState) -> Iterable[Tuple[WorldState, Tuple[Action, ...], float]]:
		self.world.set_state(state)
		if self.world.done: return []
		self.nodes_expanded += 1
		self.world.set_state(state)
		for action in product(*self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (new_state, action, cost)
			self.world.set_state(state)

class ProblemState(ABC):
	"""A problem state is a state that can be used by a search algorithm"""

	def __init__(self, world_state: WorldState):
		self._world_state = world_state

	@property
	def world_state(self) -> WorldState:
		return self._world_state

	@property
	def agents_positions(self) -> Tuple[Tuple[int, int], ...]:
		return self._world_state.agents_positions

	def __key(self):
		return (self._world_state)

	def __eq__(self, other):
		return isinstance(other, ProblemState) and self.__key() == other.__key()

	def __hash__(self):
		return hash(self.__key())

	def __repr__(self):
		return f"<ProblemState {self._world_state}>"


class CornerProblemState(ProblemState)
	def __init__(self, world_state: WorldState, corners: list[bool, bool, bool, bool]=[False,False,False,False]):
		super().__init__(world_state)
		self._corners = corners

	@property
	def corners_rate(self) -> float:
		return sum(self._corners)/4

	@property
	def corners_done(self) -> bool:
		return all(self._corners)

	def corner_done(self, idx: int):
		return self._corners[idx]

	def __key(self):
		return (self._world_state, tuple(self._corners))

	def __eq__(self, other):
		return isinstance(other, CornerProblemState) and self.__key() == other.__key()

	def __hash__(self):
		return hash(self.__key())

	def __repr__(self):
		return f"<CornerProblemState {self._world_state} {self._corners}>"

	def get_new_state(self, new_world_state, corners_pos):
		new_corners = self._corners.copy()
		for i in range(len(corners_pos)):
			if corners_pos[i] in self._world_state.agents_positions:
				new_corners[i] = True
		return CornerProblemState(new_world_state, new_corners)

class CornerSearchProblem(SearchProblem[CornerProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.corners = [(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)]
		self.initial_state = CornerProblemState(world.get_state())

	@override(SearchProblem)
	def is_goal_state(self, state: CornerProblemState) -> bool:
		return state.corners_rate == 1.0 and SimpleSearchProblem(self.world).is_goal_state(state.world_state)

	@override(SearchProblem)
	def get_successors(self, state: CornerProblemState) -> Iterable[Tuple[CornerProblemState, Action, float]]:
		self.world.set_state(state.world_state)
		if self.world.done: return []
		self.nodes_expanded += 1
		for action in product(*self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (state.get_new_state(new_state, self.corners), action, cost)
			self.world.set_state(state.world_state)

	@override(SearchProblem)
	def heuristic(self, state: CornerProblemState) -> float:
		return max(min(self._manhattan_distance(agent, self.corners[i]) if not state.corner_done(i) else float('inf') for i in range(len(self.corners))) for agent in state.agents_positions) if not state.corners_done else super().heuristic(state)

class GemProblemState(ProblemState):
	
	def __init__(self, world_state: WorldState):
		super().__init__(world_state)

	@property
	def gems_done(self) -> float:
		return all(self._world_state.gems_collected)

	@property
	def gems_remaining(self):
		return len(self._world_state.gems_collected) - sum(self._world_state.gems_collected)

	def __key(self):
		return self._world_state

	def __hash__(self):
		return hash(self.__key())

	def __eq__(self, other):
		return isinstance(other, GemProblemState) and self.__key() == other.__key()

	def __repr__(self):
		return f"<GemProblemState {self._world_state}>"

class GemSearchProblem(SearchProblem[GemProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.initial_state = GemProblemState(world.get_state())

	@override(SearchProblem)
	def is_goal_state(self, state: GemProblemState) -> bool:
		return state.gems_done and SimpleSearchProblem(self.world).is_goal_state(state.world_state)

	@override(SearchProblem)
	def get_successors(self, state: GemProblemState) -> Iterable[Tuple[GemProblemState, Action, float]]:
		self.world.set_state(state.world_state)
		if self.world.done: return []
		self.nodes_expanded += 1
		for action in product(*self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (GemProblemState(new_state), action, cost)
			self.world.set_state(state.world_state)

	@override(SearchProblem)
	def heuristic(self, state: GemProblemState) -> float:
		return 10 * state.gems_remaining if not state.gems_done else super().heuristic(state)
