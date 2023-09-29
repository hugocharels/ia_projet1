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


def global_is_useless(actions : tuple[Action]) -> bool:
	return all(actions, lambda action : action == Action.STAY)


def search(problem: SearchProblem, Frontier: type[Stack, Queue, Heap], is_useless=global_is_useless) -> Optional[Solution]:
	frontier = Frontier()
	frontier.push(Node(None, problem.initial_state, 0, 0))
	visited = set()
	while not frontier.is_empty():
		node = frontier.pop()
		if node in visited: continue
		visited.add(node)
		if problem.is_goal_state(node.state):
			return Solution(actions=node.get_actions())
		for state, action, cost in problem.get_successors(node.state):
			next_node = Node(node, state, action, node.cost + cost, node.cost + cost + problem.heuristic(state))
			frontier.push(next_node)
	return None


def dfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Stack)


def bfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Queue)


def astar(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Heap)
