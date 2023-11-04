# A* Search Algorithm with Manhattan Heuristic

This repository contains my implementation of the A* search algorithm with the Manhattan heuristic as part of my homework assignment for the Artificial Intelligence course (BMEVIMIAC10) at Budapest University of Technology and Economics.

## Overview

The A* search algorithm is a popular and powerful pathfinding and graph traversal algorithm that is widely used in AI for finding the shortest path between nodes in a graph. The algorithm combines features of uniform-cost search and greedy best-first search by considering both the actual cost to reach a node (g) and the estimated cost to get from that node to the goal (h).

For this assignment, I implemented the A* search algorithm along with the Manhattan distance heuristic. The Manhattan distance is a heuristic that computes the total number of steps moved horizontally and vertically to reach the target node from the current node, assuming that paths may only follow the grid layout.

## Implementation Details

- `aStarSearch`: This function implements the A* search algorithm. It maintains a priority queue where nodes are prioritized by the lowest combined cost and heuristic first. The function returns the actions (path) to the goal, the nodes visited, and the corresponding costs.
- `manhattanHeuristic`: This function calculates the Manhattan distance between a given state and the goal state as an estimate of the cost to reach the goal.

## Files

- `search.py`: Contains the implementation of the A* search algorithm.
               Contains the Manhattan heuristic function along with other supporting functions.

## Usage

This code was designed to work within the Pacman AI framework used in the course. To execute the different search strategies, use the following commands:

### Breadth-First Search (BFS)
Breadth-First Search is an algorithm that searches a tree (or graph) by exploring neighbors before children. To try out the medium maze with BFS, run the command:
```shell
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
```
This command initializes the Pacman game with a medium-sized maze, using a SearchAgent that implements BFS to find the path through the maze.

###Depth-First Search (DFS)
Depth-First Search is an algorithm that searches by exploring as far along each branch as possible before backtracking. To experience the medium maze with DFS, execute:
```shell
python pacman.py -l mediumMaze -p SearchAgent -a fn=dfs
```
Running this command starts the game where the SearchAgent uses DFS to navigate through the maze.

###A* Search with Euclidean Heuristic
A* Search is a best-first search that uses heuristics to improve speed. The Euclidean heuristic uses the straight-line distance to the goal. To initiate the medium maze with the Euclidean heuristic, use:
```shell
python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,heuristic=euclideanHeuristic
```
With this command, the Pacman agent will attempt to solve the maze by calculating distances using the Euclidean method.

###A* Search with Manhattan Heuristic
The Manhattan heuristic calculates the distance between two points in a grid based on the sum of the absolute differences of their Cartesian coordinates. To run the A* search with the Manhattan heuristic on a medium maze, the following command is used:
```shell
python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```
Executing this will start the Pacman game where the agent uses your implemented A* search algorithm with the Manhattan distance as a heuristic to find the shortest path.

Each of these commands will showcase the behavior of Pacman using different search strategies in a medium complexity maze. You can observe the path taken by Pacman and how quickly and efficiently it reaches the goal depending on the search strategy used.
```python
actions, visitedNodes, visitedCosts = aStarSearch(problem, manhattanHeuristic)
```

# Acknowledgments

I extend my heartfelt gratitude to the faculty and instructors of the BMEVIMIAC10 Artificial Intelligence course. Their expert guidance and steadfast support have been instrumental in the completion of this project. Their insights and suggestions have enriched my understanding of search algorithms and their applications in artificial intelligence.

## Academic Integrity Notice
Please note that the code and associated documentation provided in this repository were developed as part of a homework assignment in the BMEVIMIAC10 Artificial Intelligence course. This work is intended solely for educational purposes and should not be used for any other purpose without proper attribution to the course and its instructors. It is essential to uphold the principles of academic integrity and to give credit to the educational structures that facilitate our learning.

