from dataclasses import dataclass
from typing import Optional
from lle import Action
from problem import SearchProblem

from queue import Queue


@dataclass
class Solution:
    actions: list[list[Action]]
    reward: float = 0

    @property
    def n_steps(self) -> int:
        return len(self.actions)

    ...


def dfs(problem: SearchProblem) -> Optional[Solution]:
    

    sol = Solution(actions=[])
    return sol



def bfs(problem: SearchProblem) -> Optional[Solution]:
    queue = Queue()
    queue.put((problem.initial_state, [], 0))
    while not queue.empty():
        state, actions, reward = queue.get()
        if problem.is_goal_state(state):
            print(actions)
            return Solution(actions=actions, reward=reward)
        for successor in problem.get_successors(state):
            queue.put((successor[0], actions + list(successor[1]), reward + successor[2]))
    return None



def astar(problem: SearchProblem) -> Optional[Solution]:
    ...
