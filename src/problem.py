from abc import ABC, abstractmethod
from typing import List, Tuple, Iterable, Generic, TypeVar
from lle import World, Action, WorldState, Position


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

	@staticmethod
	def _manhattan_distance(pos1, pos2):
		return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

	def heuristic(self, problem_state: T) -> float:
		return 0.0

class SimpleSearchProblem(SearchProblem[WorldState]):

	def is_goal_state(self, state: WorldState) -> bool:
		tmp = self.world.get_state()
		self.world.set_state(state)
		ret = self.world.exit_rate == 1.0
		self.world.set_state(tmp)
		return ret

	def get_successors(self, state: WorldState) -> Iterable[Tuple[WorldState, Tuple[Action, ...], float]]:
		tmp = self.world.get_state()
		self.world.set_state(state)
		for possible_action in self._get_all_actions(self.world.available_actions()):
			if self.world.done: continue
			reward = self.world.step(possible_action)
			#print(reward, self.world.exit_rate)
			#yield (self.world.get_state(), possible_action, (1 - self.world.exit_rate) * 2 + (2 - reward) * 1)
			yield (self.world.get_state(), possible_action, 1 - self.world.exit_rate)
			self.world.set_state(state)
		self.world.set_state(tmp)
		self.nodes_expanded += 1

	def heuristic(self, state: WorldState) -> float:
		"""Manhattan distance for each agent to its goal"""
		exit_pos = self.world.exit_pos
		agents_pos = state.agents_positions
		return sum(self._manhattan_distance(agents_pos[i], exit_pos[i]) for i in range(len(exit_pos)-1))/len(agents_pos)


class CornerProblemState:
	
	def __init__(self, agents_positions: List[Position], corners_done: List[Position]):
		self._agents_positions = agents_positions
		self._corners_done = corners_done

	@property
	def agents_positions(self) -> List[Position]:
		return self._agents_positions

	@property
	def corners_done(self) -> List[Position]:
		return self._corners_done

	@property
	def corner_rate(self) -> float:
		return 1/(5-len(self._corners_done)) if len(self._corners_done) > 0 else 0.0

	def _update_corners(self, corners: List[Position]) -> None:
		for agent_pos in self._agents_positions: 
			if agent_pos in corners and agent_pos not in self._corners_done: self._corners_done.append(agent_pos)

	def update(self, world_state: WorldState, corners: List[Position]) -> None:
		self._agents_positions = world_state.agents_positions
		self._update_corners(corners)

	def get_world_state(self) -> WorldState:
		return WorldState(self._agents_positions, [])

	def __key(self):
		return (tuple(self._agents_positions), tuple(self._corners_done))

	def __hash__(self) -> int:
		return hash(self.__key())

	def __eq__(self, other) -> bool:
		return self.__key() == other.__key()


class CornerSearchProblem(SearchProblem[CornerProblemState]):
	def __init__(self, world: World):
		super().__init__(world)
		self.corners = [(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)]
		self.initial_state = CornerProblemState(world.agents_positions, [])

	def is_goal_state(self, state: CornerProblemState) -> bool:
		return set(state.corners_done) == self.corners and set(state.agents_positions) == set(self.world.exit_pos)

	def get_successors(self, state: CornerProblemState) -> Iterable[Tuple[CornerProblemState, Action, float]]:
		tmp = self.world.get_state()
		world_state = state.get_world_state()
		self.world.set_state(world_state)
		for possible_action in self._get_all_actions(self.world.available_actions()):
			if self.world.done: continue
			reward = self.world.step(possible_action)
			state.update(self.world.get_state(), self.corners)
			yield (state, possible_action, (1 - state.corner_rate) * 2 + 1 - self.world.exit_rate)
			self.world.set_state(world_state)
		self.world.set_state(tmp)
		self.nodes_expanded += 1

	def heuristic(self, problem_state: CornerProblemState) -> float:
		"""Better then Manhattan distance"""
		return min(self._manhattan_distance(agent_pos, corner) for corner in self.corners for agent_pos in problem_state.agents_positions)
		

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
