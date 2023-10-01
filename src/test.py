import cv2
from lle import World
from problem import SimpleSearchProblem, CornerSearchProblem, GemSearchProblem
from search import dfs, astar, dfs_matiass


#world = World.from_file("cartes/corners")
world = World.from_file("cartes/2_agents/zigzag")

img = world.get_image()
#cv2.imwrite('world.png', img)

#problem = CornerSearchProblem(world)
#problem = GemSearchProblem(world)
problem = SimpleSearchProblem(world)

solution = dfs(problem)
world.reset()
corners = set([(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)])
for action in solution.actions:
	world.step(action)
	agent_pos = world.agents_positions[0]
	if agent_pos in corners:
		corners.remove(agent_pos)
print(solution)

solution = dfs_matiass(problem)
world.reset()
corners = set([(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)])
for action in solution.actions:
	world.step(action)
	agent_pos = world.agents_positions[0]
	if agent_pos in corners:
		corners.remove(agent_pos)
print(solution)
