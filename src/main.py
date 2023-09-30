from lle import World
from problem import SimpleSearchProblem
import search


w = World.from_file("cartes/2_agents/zigzag")
#w = World.from_file("level3")
#w = World.from_file("level5")
#w = World.from_file("level6")

#w = World.from_file("cartes/1_agent/vide")

print(w.world_string)

problem = SimpleSearchProblem(w)
solution = search.astar(problem)

if solution is None:
    print("No solution found")
else:
    print(solution.n_steps)
    print(solution.actions)
print(f"{problem.nodes_expanded} nodes expanded")