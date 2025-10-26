from enum import Enum


class Direction(int, Enum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]

#increase turn factor if robot struggles with precise turn
TURN_FACTOR = 1 #weight constant that controls how expensive turning is compared to moving straight (used in A*)
TURN_RADIUS = 1 #in grid cell that the robot occupy when turning

#how many cells of buffer space to keep around each obstacle and around the robot 
#inflates obstacle by 1, so if obstacle at (5,5) then cells at (4,5), (6,5), (5,4), (5,6) are considered unsafe
EXPANDED_CELL = 1 # for both agent and obstacles

#arena size 20x20 cells - each cell 10cm
WIDTH = 20
HEIGHT = 20

#limit on number of iterations for path finding algorithm
ITERATIONS = 2000

#penalty added to path cost if a move risks touching obstacle or enters near danger zone
SAFE_COST = 1000 # the cost for the turn in case there is a chance that the robot is touch some obstacle

#a small cost for taking a screenshot to prevent overprioritizing photo positions
SCREENSHOT_COST = 50 # the cost for the place where the picture is taken