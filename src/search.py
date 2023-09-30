from dataclasses import dataclass
from typing import Optional
from lle import Action

from problem import SearchProblem

from frontier import Stack, Queue, Heap, Node


@dataclass
class Solution:
	actions: list[tuple[Action]]

	@property
	def n_steps(self) -> int:
		return len(self.actions)


def global_is_useless(_, actions : tuple[Action]) -> bool:
	for action in actions:
		if action != Action.STAY: return False
	return True

def is_opposite_move(prev_action: Action, next_action: Action) -> bool:
	return prev_action == Action.NORTH and next_action == Action.SOUTH or \
	   prev_action == Action.SOUTH and next_action == Action.NORTH or \
	   prev_action == Action.WEST and next_action == Action.EAST or \
	   prev_action == Action.EAST and next_action == Action.WEST

def are_opposite_move(prev_actions: tuple[Action, ...], next_actions: tuple[Action, ...]) -> bool:
	if prev_actions is None or next_actions is None: return False
	for i in range(len(next_actions)):
		if not is_opposite_move(prev_actions[i], next_actions[i]): return False
	return True


def search(problem: SearchProblem, Frontier: type[Stack, Queue, Heap], is_useless=global_is_useless) -> Optional[Solution]:
	frontier = Frontier()
	frontier.push(Node(None, problem.initial_state, None, 0))
	visited = set()
	while not frontier.is_empty():
		node = frontier.pop()
		if node in visited: continue
		visited.add(node)
		if problem.is_goal_state(node.state):
			return Solution(actions=node.get_actions())
		for state, action, cost in problem.get_successors(node.state):
			#if is_useless(node.action, action): continue
			next_node = Node(node, state, action, node.cost + problem.g(state, action, cost), node.cost + problem.f(state, action, cost))
			frontier.push(next_node)
	return None


def dfs(problem: SearchProblem) -> Optional[Solution]:
	#return search(problem, Stack, lambda prev_a, next_a : False)
	return search(problem, Stack, lambda prev_a, next_a : global_is_useless(prev_a, next_a) or are_opposite_move(prev_a, next_a) )


def bfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Queue)


def astar(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Heap)
