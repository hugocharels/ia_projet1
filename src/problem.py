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
	def is_useless(actions: tuple[Action, ...]) -> bool:
		for action in actions:
			if action != Action.STAY: return False
		return True

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
		self.world.set_state(state)
		is_goal = self.world.exit_rate == 1.0
		return is_goal

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

	@override(SearchProblem)
	def g(self, state: WorldState, actions: tuple[Action,...], cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		step = 0
		for action in actions: 
			if action != Action.STAY: step += 1
		#return win + 10 * cost + 1 * step
		return (win - 1 * cost + 1 * step ) * 1

	@override(SearchProblem)
	def heuristic(self, state: WorldState) -> float:
		"""Manhattan distance for each agent to the closest exit"""
		return max(min(self._manhattan_distance(agent, exit) for exit in self.world.exit_pos) for agent in state.agents_positions)



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
		self.world.set_state(state.world_state)
		if self.world.done: return []
		self.nodes_expanded += 1
		for action in product(*self.world.available_actions()):
			if self.is_useless(action): continue
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (state.get_new_state(new_state, self.corners), action, cost)
			self.world.set_state(state.world_state)
		
	def g(self, state: CornerProblemState, actions: tuple[Action, ...], cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		step = 0
		for action in actions: 
			if action != Action.STAY: step += 1
		return win - 10 * state.on_corner + 1 * step

	def heuristic(self, state: CornerProblemState) -> float:
		return max(min(self._manhattan_distance(agent, corner) for corner in self.corners) for agent in state.agents_positions) \
				if state.corners_rate < 1.0 else \
				max(min(self._manhattan_distance(agent, exit) for exit in self.world.exit_pos) for agent in state.agents_positions)


class GemProblemState(ProblemState):
	
	def __init__(self, world_state: WorldState, old_state: 'GemProblemState'=None):
		super().__init__(world_state)
		if old_state is not None: self._gems_got = old_state.gems_remaining - self.gems_remaining

	@property
	def gems_got(self) -> float:
		return self._gems_got

	@property
	def gems_rate(self) -> float:
		return sum(self._world_state.gems_collected)/len(self._world_state.gems_collected)

	@property
	def gems_remaining(self):
		return len(self._world_state.gems_collected) - sum(self._world_state.gems_collected)

	def __hash__(self):
		return super.__hash__(self)

	def __eq__(self):
		return super.__eq__(self)

	def __repr__(self):
		return f"<GemProblemState {self._world_state}>"


class GemSearchProblem(SearchProblem[GemProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.initial_state = GemProblemState(world.get_state())

	def is_goal_state(self, state: GemProblemState) -> bool:
		return state.gems_rate == 1.0 and SimpleSearchProblem(self.world).is_goal_state(state.world_state)

	def get_successors(self, state: GemProblemState) -> Iterable[Tuple[GemProblemState, Action, float]]:
		self.world.set_state(state.world_state)
		if self.world.done: return []
		self.nodes_expanded += 1
		for action in product(*self.world.available_actions()):
			if self.is_useless(action): continue
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (GemProblemState(new_state, state), action, cost)
			self.world.set_state(state.world_state)

	def g(self, state: CornerProblemState, actions: tuple[Action, ...], cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		step = 0
		for action in actions: 
			if action != Action.STAY: step += 1
		return win - 10 * state.gems_got + 1 * step

	def heuristic(self, state: GemProblemState) -> float:
		"""The number of uncollected gems"""
		return state.gems_remaining
