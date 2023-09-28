from dataclasses import dataclass
from typing import Optional
from lle import Action
from problem import SearchProblem
from typing import Tuple, Iterable, Generic, TypeVar
from queue import Queue, LifoQueue
from priority_queue import PriorityQueue


@dataclass
class Solution:
	actions: list[Tuple[Action,...]]
	reward: float = 0

	@property
	def n_steps(self) -> int:
		return len(self.actions)


def are_opposites(action1: Action, action2: Action) -> bool:
	return action1 == Action.NORTH and action2 == Action.SOUTH or \
		   action1 == Action.SOUTH and action2 == Action.NORTH or \
		   action1 == Action.WEST and action2 == Action.EAST or \
		   action1 == Action.EAST and action2 == Action.WEST

def is_useless(moves: Tuple[Action, ...], last_move: Tuple[Action, ...]) -> bool:
	for i in range(len(moves)):
		if moves[i] == Action.STAY: continue
		if not are_opposites(moves[i], last_move[i]): return False
	return True


def search(problem: SearchProblem, Frontier: Generic) -> Optional[Solution]:
	frontier = Frontier()
	frontier.put((problem.initial_state, [], 0.0))
	all_state = set()
	while not frontier.empty():
		state, actions, reward = frontier.get()
		if problem.is_goal_state(state):
			return Solution(actions=actions, reward=reward)
		if state in all_state: continue
		else: all_state.add(state)
		for next_state, next_actions, next_reward in problem.get_successors(state):
			#if next_state in all_state or (len(actions) > 0 and is_useless(next_actions, actions[-1])): continue
			if next_state in all_state: continue
			frontier.put((next_state, actions + [next_actions], next_reward + reward))
	return None

def dfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, LifoQueue)

def bfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Queue)

def astar(problem: SearchProblem) -> Optional[Solution]:
	heap = PriorityQueue()
	heap.push((problem.initial_state, [], 0.0), 0.0)
	all_state = set()
	while not heap.isEmpty():
		state, actions, reward = heap.pop()
		if state in all_state: continue
		else: all_state.add(state)
		print("----------------------")
		print(state, actions, reward)
		if problem.is_goal_state(state):
			return Solution(actions=actions, reward=reward)
		for next_state, next_actions, next_reward in problem.get_successors(state):
			#if len(actions) > 0 and is_useless(next_actions, actions[-1]): continue
			#if next_state in all_state: continue
			heap.push((next_state, actions + [next_actions], next_reward + reward),
					  next_reward + reward + problem.heuristic(next_state))
			#		  next_reward + reward)
			#		  reward + problem.heuristic(next_state))
	return None
