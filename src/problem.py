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
		tmp = self.world.get_state()
		self.world.set_state(state)
		ret = self.world.exit_rate == 1.0
		self.world.set_state(tmp)
		return ret

	def _get_all_actions(self, actions: Iterable[Tuple[Action, ...]]) -> Iterable[Tuple[Action, ...]]:
		"""Yield all possible joint actions from the given list of actions"""
		if not actions:
			yield ()
		else:
			for action in actions[0]:
				for next_actions in self._get_all_actions(actions[1:]):
					yield (action,) + next_actions

	def get_successors(self, state: WorldState) -> Iterable[Tuple[WorldState, Tuple[Action, ...], float]]:
		tmp = self.world.get_state()
		self.world.set_state(state)
		for possible_action in self._get_all_actions(self.world.available_actions()):
			if not self.world.done:
				reward = self.world.step(possible_action)
				yield (self.world.get_state(), possible_action, reward)
				self.world.set_state(state)
		self.world.set_state(tmp)
		self.nodes_expanded += 1

	def heuristic(self, state: WorldState) -> float:
		"""Manhattan distance for each agent to its goal"""
		exit_pos = self.world.exit_pos
		agent_pos = state.agents_positions
		return sum(abs(agent[0] - exit[0]) + abs(agent[1] - exit[1]) for agent, exit in zip(agent_pos, exit_pos))


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
