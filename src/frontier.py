from abc import ABC, abstractmethod
from queue import Queue as qQueue, LifoQueue
from priority_queue import PriorityQueue
from typing import Generic, TypeVar

T = TypeVar("T")

def override(abstract_class):
	def overrider(method):
		assert(method.__name__ in dir(abstract_class))
		return method
	return overrider


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
		return list(reversed(actions))
