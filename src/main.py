from lle import World
from problem import SimpleSearchProblem, CornerSearchProblem, GemSearchProblem
from search import dfs, bfs, astar

w = World.from_file("level3")


algos = [(dfs, "dfs"), (bfs, "bfs"), (astar, "astar")]

for algo, name in algos:
	problem = GemSearchProblem(w)
	solution = algo(problem)
	if solution is None:
		print(f"{name}: No solution found")
	else:
		#print(solution.actions)
		print(f"{name}: {len(solution.actions)} d'actions, {problem.nodes_expanded} nodes expanded")
		print()
