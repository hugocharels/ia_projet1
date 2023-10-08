from dataclasses import dataclass
from typing import Optional, Generic, TypeVar
from lle import Action
from abc import ABC, abstractmethod
from queue import Queue as qQueue, LifoQueue
from priority_queue import PriorityQueue

from problem import SearchProblem, override



#################### frontier.py ####################

T = TypeVar("T")

class Frontier(ABC, Generic[T]):

	@abstractmethod
	def push(node: T) -> None:
		""" Add a node to the frontier. """

	@abstractmethod
	def pop() -> T:
		""" Remove and return the next node from the frontier. """

	@abstractmethod
	def is_empty() -> bool:
		""" Return True if the frontier is empty. """


class Stack(Frontier[T]):

	def __init__(self):
		self.queue = LifoQueue()

	@override(Frontier)
	def push(self, node: T) -> None:
		self.queue.put(node)

	@override(Frontier)
	def pop(self) -> T:
		return self.queue.get()

	@override(Frontier)
	def is_empty(self) -> bool:
		return self.queue.empty()


class Queue(Frontier[T]):

	def __init__(self):
		self.queue = qQueue()

	@override(Frontier)
	def push(self, node: T) -> None:
		self.queue.put(node)

	@override(Frontier)
	def pop(self) -> T:
		return self.queue.get()

	@override(Frontier)
	def is_empty(self) -> bool:
		return self.queue.empty()


class Heap(Frontier[T]):

	def __init__(self):
		self.queue = PriorityQueue()

	@override(Frontier)
	def push(self, node: T) -> None:
		self.queue.push(node, node.priority)

	@override(Frontier)
	def pop(self) -> T:
		return self.queue.pop()

	@override(Frontier)
	def is_empty(self) -> bool:
		return self.queue.is_empty()



class Node:

	def __init__(self, parent, state, action, cost, priority=0):
		self.parent = parent
		self.state = state
		self.action = action
		self.cost = cost
		self.priority = priority

	def __repr__(self):
		return f"<Node {self.state}, {self.action}>"

	def __key(self):
		return (self.state)

	def __eq__(self, other):
		return isinstance(other, Node) and self.__key() == other.__key()

	def __hash__(self):
		return hash(self.__key())

	def get_actions(self):
		actions = []
		node = self
		while node.parent is not None:
			actions.append(node.action)
			node = node.parent
			print(f"g(n)={node.cost}, h(n)= {node.priority - node.cost}")
		return list(reversed(actions))


#################### search.py ####################


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
