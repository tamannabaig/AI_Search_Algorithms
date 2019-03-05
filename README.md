# AI_Search_Algorithms

This project was created as part of coursework for my Artificial Intelligence course.
The task is to implement a number of search algorithms to guide an agent through a grid-based world.
For simplicity, we'll assume that the state space is 2-dimensional, consisting of the grid location of the agent.

The python file search.py consists implementation for 3 of the main search algorithms:
<b> 1. Depth-First Search: </b> The depth_first_search function in search.py implements the depth-first search algorithm. We have employed the graph search version of DFS which allows a state to be expanded only once. To evaluate the correctness of our code we have tested it on different 2D mas produced using the utils.

<b> 2. Uniform Cost Search: </b> Depth-first search cannot find optimal paths, unless it gets lucky. To address this issue, we have used the uniform_cost_search function in search.py to implement the uniform cost search algorithm. Again, we wrote a graph search algorithm that allows a state to be expanded only once. The algorithm returns the optimal path from start to goal.


<b> 3. A* Search: </b> The astar_search function in search.py is to implement A* graph search. As A* needs a heuristic function to work, we have also provided a heuristic function accordingly to return the Manhattan distance between a given state and the goal state.
