from abc import ABC, abstractmethod
from typing import Tuple, Iterable, Generic, TypeVar
from lle import World, Action, WorldState
from frontier import override, Node

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
		for action in self._get_all_actions(self.world.available_actions()):
			cost = self.world.step(action)
			new_state = self.world.get_state()
			yield (new_state, action, cost)
			self.world.set_state(state)
		self.world.set_state(tmp_state)
		self.nodes_expanded += 1

	@override(SearchProblem)
	def g(self, state: WorldState, cost: float) -> float:
		win = -500 if self.is_goal_state(state) else 0
		return win + 100 * (1 - self.world.exit_rate) + cost

	@override(SearchProblem)
	def heuristic(self, state: WorldState) -> float:
		"""Manhattan distance for each agent to the closest exit"""
		return 0.0


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
