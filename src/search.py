from dataclasses import dataclass
from typing import Optional
from lle import Action
from problem import SearchProblem
from typing import Tuple, Iterable, Generic, TypeVar
from queue import Queue


@dataclass
class Solution:
    actions: list[Tuple[Action,...]]
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
    queue.put((problem.initial_state, [], 0.0))
    while not queue.empty():
        state, actions, reward = queue.get()
        if problem.is_goal_state(state):
            return Solution(actions=actions, reward=reward)
        # print("-----------------")
        # print("bfs : ", state, actions, reward)
        for next_state, next_actions, next_reward in problem.get_successors(state):
            for e in next_actions:
                if e != Action.STAY: break
            else: continue
            queue.put((next_state, actions + [next_actions], reward + next_reward))
    return None



def astar(problem: SearchProblem) -> Optional[Solution]:
    ...
