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


def search(problem: SearchProblem, Frontier: type[Stack, Queue, Heap]) -> Optional[Solution]:
	frontier = Frontier()
	frontier.push(Node(None, problem.initial_state, None, 0))
	visited = {problem.initial_state}
	while not frontier.is_empty():
		node = frontier.pop()
		if problem.is_goal_state(node.state):
			return Solution(actions=node.get_actions())
		for state, action, cost in problem.get_successors(node.state):
			if state in visited: continue
			visited.add(state)
			next_node = Node(node, state, action, node.cost + problem.g(state, cost), node.cost + problem.f(state, cost))
			frontier.push(next_node)
	return None


def dfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Stack)

def bfs(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Queue)

def astar(problem: SearchProblem) -> Optional[Solution]:
	return search(problem, Heap)
