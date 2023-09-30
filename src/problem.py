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

	@staticmethod
	def _get_all_actions(actions: Iterable[Tuple[Action, ...]]) -> Iterable[Tuple[Action, ...]]:
		"""Yield all possible joint actions from the given list of actions"""
		if not actions:
			yield ()
		else:
			for action in actions[0]:
				for next_actions in SimpleSearchProblem._get_all_actions(actions[1:]):
					yield (action,) + next_actions

	@abstractmethod
	def get_successors(self, state: T) -> Iterable[Tuple[T, Tuple[Action, ...], float]]:
		"""
		Yield all possible states that can be reached from the given world state.
		Returns
			- the new problem state
			- the joint action that was taken to reach it
			- the cost of taking the action
		"""

	def _manhattan_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
		return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

	def f(self, problem_state: T, *args):
		return self.g(problem_state, *args) + self.heuristic(problem_state)

	@abstractmethod
	def g(self, problem_state: T, *args) -> float:
		"""The cost of reaching the given state"""

	def heuristic(self, problem_state: T) -> float:
		return 0.0


class SimpleSearchProblem(SearchProblem[WorldState]):

	def is_goal_state(self, state: WorldState) -> bool:
		tmp = self.world.get_state()
		self.world.set_state(state)
		is_goal = self.world.exit_rate == 1.0
		self.world.set_state(tmp)
		return is_goal

	def get_successors(self, state: WorldState) -> Iterable[Tuple[WorldState, Tuple[Action, ...], float]]:
		tmp_state = self.world.get_state()
		self.world.set_state(state)
		if self.world.done: return []
		#for action in product(*self.world.available_actions()):
		for action in self._get_all_actions(self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (new_state, action, cost)
			self.world.set_state(state)
		self.world.set_state(tmp_state)
		self.nodes_expanded += 1

	@override(SearchProblem)
	def g(self, state: WorldState, actions: tuple[Action,...], cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		step = 0
		for action in actions: 
			if action != Action.STAY: step += 1
		return win + 10 * cost + 1 * step

	@override(SearchProblem)
	def heuristic(self, state: WorldState) -> float:
		"""Manhattan distance for each agent to the closest exit"""
		return sum(min(self._manhattan_distance(agent, exit) for exit in self.world.exit_pos) for agent in state.agents_positions)



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


class CornerProblemState(ProblemState):
	def __init__(self, world_state: WorldState, corners: list[bool, bool, bool, bool]=[False,False,False,False]):
		super().__init__(world_state)
		self._corners = corners
		self._on_corner = False

	@property
	def corners_rate(self) -> float:
		return sum(self._corners)/4

	@property
	def on_corner(self):
		return self._on_corner

	def __key(self):
		return (self._world_state, tuple(self._corners))

	def __eq__(self, other):
		return isinstance(other, CornerProblemState) and self.__key() == other.__key()

	def __hash__(self):
		return hash(self.__key())

	def __repr__(self):
		return f"<CornerProblemState {self._world_state} {self._corners}>"

	def get_new_state(self, new_world_state, corners_pos):
		new_corners = self._corners
		for i in range(len(corners_pos)):
			if corners_pos[i] in self._world_state.agents_positions:
				if new_corners[i] == False: self._on_corner = True
				new_corners[i] = True
		return CornerProblemState(new_world_state, new_corners.copy())


class CornerSearchProblem(SearchProblem[CornerProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.corners = [(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)]
		self.initial_state = CornerProblemState(world.get_state())

	def is_goal_state(self, state: CornerProblemState) -> bool:
		return state.corners_rate == 1.0 and SimpleSearchProblem(self.world).is_goal_state(state.world_state)

	def get_successors(self, state: CornerProblemState) -> Iterable[Tuple[CornerProblemState, Action, float]]:
		tmp_state = self.world.get_state()
		self.world.set_state(state.world_state)
		if self.world.done: return []
		for action in self._get_all_actions(self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (state.get_new_state(new_state, self.corners), action, cost)
			self.world.set_state(state.world_state)
		self.world.set_state(tmp_state)
		self.nodes_expanded += 1

	def g(self, state: CornerProblemState, actions: tuple[Action, ...], cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		step = 0
		for action in actions: 
			if action != Action.STAY: step += 1
		return win - 10 * state.on_corner + 1 * step

	def heuristic(self, state: CornerProblemState) -> float:
		return sum(min(self._manhattan_distance(agent, corner) for corner in self.corners) for agent in state.agents_positions) \
				if state.corners_rate < 1.0 else \
				sum(min(self._manhattan_distance(agent, exit) for exit in self.world.exit_pos) for agent in state.agents_positions)


class GemProblemState:
	...


class GemSearchProblem(SearchProblem[GemProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.initial_state = ...

	def is_goal_state(self, state: GemProblemState) -> bool:
		raise NotImplementedError()

	def get_successors(self, state: GemProblemState) -> Iterable[Tuple[GemProblemState, Action, float]]:
		self.nodes_expanded += 1
		raise NotImplementedError()

	def heuristic(self, state: GemProblemState) -> float:
		"""The number of uncollected gems"""
		raise NotImplementedError()
