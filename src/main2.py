
from lle import World
from problem import SimpleSearchProblem, CornerSearchProblem, GemSearchProblem
import search


w = World.from_file("cartes/gems")


problem = GemSearchProblem(w)
solution = search.astar(problem)

if solution is None:
    print("No solution found")
else:
    print(solution.n_steps)
    print(solution.actions)
print(f"{problem.nodes_expanded} nodes expanded")

