from lle import World
from problem import SimpleSearchProblem, CornerSearchProblem, GemSearchProblem
from search import dfs, bfs, astar
import cv2

from time import time

"""
import sys

def select_problem(world, choice):
	if choice == '1':
		return SimpleSearchProblem(world)
	elif choice == '2':
		return CornerSearchProblem(world)
	elif choice == '3':
		return GemSearchProblem(world)
	else:
		print("Choix non reconnu!")
		sys.exit()

if __name__ == "__main__":
	carte = "level3"
	#carte = "cartes/2_agents/zigzag"
	w = World.from_file(carte)
	
	print("Choix problème :")
	print("1 = Simple")
	print("2 = Corner")
	print("3 = Gems")
	choice = input("Entrez votre choix: ")

	algos = [(dfs, "dfs"), (bfs, "bfs"), (astar, "astar")]
	algos = [(astar, "astar")]

	#print("\033c")
	for algo, name in algos:
		problem = select_problem(w, choice)
		debut = time()
		solution = algo(problem)
		fin = time()
		
		problem_name = ""
		if choice == '1':
			problem_name = "SimpleSearchProblem"
		elif choice == '2':
			problem_name = "CornerSearchProblem"
		elif choice == '3':
			problem_name = "GemSearchProblem"
		
		if solution is None:
			print(f"{name}: No solution found")
		else:
			print("Carte : " + carte)
			print(f"Problem : {problem_name}")
			print(f"{name}: {len(solution.actions)} d'actions, {problem.nodes_expanded} nodes expanded en {fin - debut} secondes")
			print()

"""


maps = ["level1", "level2", "level3", "level4", "level5", "level6"]

algos = [(astar, "astar")]

problems = [(SimpleSearchProblem, "SimpleSearchProblem"), (CornerSearchProblem, "CornerSearchProblem"), (GemSearchProblem, "GemSearchProblem")]

#problems = [(GemSearchProblem, "GemSearchProblem")]

for map in maps:
	w = World.from_file(map)
	print("Carte : " + map)
	#img = w.get_image()
	#cv2.imwrite(map + '.png', img)
	print()
	for algo, name in algos:
		for problem, problem_name in problems:
			problem = problem(w)
			debut = time()
			solution = algo(problem)
			fin = time()
			if solution is None:
				print(f"{name}: No solution found")
			else:
				print(f"Problem : {problem_name}")
				print(f"{name}: {len(solution.actions)} actions, {problem.nodes_expanded} nodes expanded en {fin - debut} secondes")
				#print(solution.actions)
				print()
