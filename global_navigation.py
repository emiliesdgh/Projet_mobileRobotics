import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


class globalNavigation:
    
    # Define the type of movement the Thymio will follow (at the moment is 4N but we'll see)

    def _get_movements_4n(self):
        """
        Get all possible 4-connectivity movements (up, down, left right).
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0)]

    def _get_movements_8n(self):
        """
        Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
        (up, down, left, right and the 4 diagonals).
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        s2 = math.sqrt(2)
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, s2),
                (-1, 1, s2),
                (-1, -1, s2),
                (1, -1, s2)]
    
    
    # Implementation of A* algorithm

    def reconstruct_path(self, cameFrom, current):
        """
        Recurrently reconstructs the path from start node to the current node
        :param cameFrom: map (dictionary) containing for each node n the node immediately 
                         preceding it on the cheapest path from start to n 
                         currently known.
        :param current: current node (y, x)
        :return: list of nodes from start to current node
        """
        total_path = [current]
        while current in cameFrom.keys():
            # Add where the current node came from to the start of the list
            total_path.insert(0, cameFrom[current]) 
            current=cameFrom[current]
        return total_path

    def A_Star(self, start, goal, occupancy_grid, movement_type="4N"):
        """
        A* for 2D occupancy grid. Finds a path from start to goal.
        h is the heuristic function. h(n) estimates the cost to reach goal from node n.
        :param start: start node (y , x) with inverted coordinates
        :param goal_m: goal node (y, x) with inverted coordinates
        :param occupancy_grid: the grid map
        :param movement: select between 4-connectivity ('4N', default) and 8-connectivity ('8N')
        :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
        """
        # --------------------------------------------------------------------------------------------
        # Initial coords and heuristic
        # --------------------------------------------------------------------------------------------

        # Set a list of all coords
        coords = [(y,x) for y in range(occupancy_grid.shape[0]) for x in range(occupancy_grid.shape[1])]

        # Define the heuristic, here = distance to goal ignoring obstacles
        h = np.linalg.norm(np.array(coords) - goal, axis=-1)
        # h *= 1.333 for optimization
        h = dict(zip(coords, h))

        # --------------------------------------------------------------------------------------------
        # Checks to make the code robust to potential errors
        # --------------------------------------------------------------------------------------------

        # Check if the start and goal are within the boundaries of the map (may be needed if there's some issue with Vision)
        for point in [start, goal]:
            for coord in point:
                assert coord>=0 and coord<occupancy_grid.shape[1], "start or end goal not contained in the map"

        # Check if start and goal nodes correspond to free spaces (again, if there is issues with Vision)
        if occupancy_grid[start[0], start[1]]:
            raise Exception('Start node is not traversable')

        if occupancy_grid[goal[0], goal[1]]:
            raise Exception('Goal node is not traversable')

        # get the possible movements corresponding to the selected connectivity (THIS IS NEEDED for the project)
        if movement_type == '4N':
            movements = self._get_movements_4n()
        elif movement_type == '8N':
            movements = self._get_movements_8n()
        else:
            raise ValueError('Unknown movement')

        # --------------------------------------------------------------------------------------------
        # A* Algorithm implementation
        # --------------------------------------------------------------------------------------------

        # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
        # Initially, only the start node is known.
        openSet = [start]

        # The set of visited nodes that no longer need to be expanded.
        closedSet = []

        # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
        cameFrom = dict()

        # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
        gScore[start] = 0

        # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
        fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
        fScore[start] = h[start]

        # while there are still elements to investigate
        while openSet != []:

            #the node in openSet having the lowest fScore[] value
            fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}

            current = min(fScore_openSet, key=fScore_openSet.get)

            del fScore_openSet

            #If the goal is reached, reconstruct and return the obtained path
            if current == goal:
                # return in form of array in (x,y) format for better handling
                return np.array(self.reconstruct_path(cameFrom, current))[:, [1, 0]], np.array(closedSet)[:, [1, 0]]

            openSet.remove(current)
            closedSet.append(current)

            #for each neighbor of current:
            for dx, dy, deltacost in movements:
                neighbor = (current[0]+dx, current[1]+dy)

                # if the node is not in the map, skip
                if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                    continue

                # if the node is occupied or has already been visited, skip
                if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                    continue

                # d(current,neighbor) is the weight of the edge from current to neighbor
                # tentative_gScore is the distance from start to the neighbor through current
                tentative_gScore = gScore[current] + deltacost

                if neighbor not in openSet:
                    openSet.append(neighbor)

                if tentative_gScore < gScore[neighbor]:
                    # This path to neighbor is better than any previous one. Record it!
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = gScore[neighbor] + h[neighbor]


        # Open set is empty but goal was never reached
        print("No path found to goal")
        return [], closedSet