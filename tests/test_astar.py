from lle import World, Action
from search import ProblemState, astar
from problem import SimpleSearchProblem

from .utils import check_world_done


def test_1_agent_empty():
    world = World.from_file("cartes/1_agent/vide")
    problem = SimpleSearchProblem(world)
    solution = astar(problem)
    assert solution.n_steps == 8
    assert solution.actions.count((Action.EAST,)) == 6
    assert solution.actions.count((Action.SOUTH,)) == 2
    check_world_done(problem, solution)


def test_1_agent_zigzag():
    world = World.from_file("cartes/1_agent/zigzag")
    problem = SimpleSearchProblem(world)
    solution = astar(problem)
    assert solution.n_steps == 19
    assert solution.actions == [
        (Action.NORTH,),
        (Action.NORTH,),
        (Action.EAST,),
        (Action.EAST,),
        (Action.SOUTH,),
        (Action.SOUTH,),
        (Action.EAST,),
        (Action.EAST,),
        (Action.NORTH,),
        (Action.NORTH,),
        (Action.EAST,),
        (Action.EAST,),
        (Action.SOUTH,),
        (Action.SOUTH,),
        (Action.EAST,),
        (Action.EAST,),
        (Action.NORTH,),
        (Action.NORTH,),
        (Action.EAST,),
    ]
    check_world_done(problem, solution)


def test_1_agent_impossible():
    world = World.from_file("cartes/1_agent/impossible")
    problem = SimpleSearchProblem(world)
    assert astar(problem) is None


def test_2_agents_empty():
    world = World.from_file("cartes/2_agents/vide")
    problem = SimpleSearchProblem(world)
    solution = astar(problem)
    assert solution.n_steps == 8
    check_world_done(problem, solution)


def test_2_agents_zigzag():
    world = World.from_file("cartes/2_agents/zigzag")
    world.reset()
    problem = SimpleSearchProblem(world)
    solution = astar(problem)
    assert solution.n_steps == 15
    check_world_done(problem, solution)


def test_2_agents_impossible():
    world = World.from_file("cartes/2_agents/impossible")
    problem = SimpleSearchProblem(world)
    assert astar(problem) is None


def test_level3():
    world = World.from_file("level3")
    problem = SimpleSearchProblem(world)
    solution = astar(problem)
    check_world_done(problem, solution)
