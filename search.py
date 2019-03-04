    # Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Authors: Pei Xu (peix@g.clemson.edu) and Ioannis Karamouzas (ioannis@g.clemson.edu)
#

"""
 In this assignment, the task is to implement different search algorithms to find 
 a path from a given start cell to the goal cell in a 2D grid map.

 To complete the assignment, you must finish three functions:
   depth_first_search (line 148), uniform_cost_search (line 224), 
   and astar_search (line 264).

 During the search, a cell is represented using a tuple of its location
 coordinates.
 For example:
   the cell at the left-top corner is (0, 0);
   the cell at the first row and second column is (0, 1);
   the cell at the second row and first column is (1, 0).
 You need put these tuples into the open set or/and closed set properly
 during searching.
"""

# ACTIONS defines how to reach an adjacent cell from a given cell
# Important: please check that a cell within the bounds of the grid
# when try to access it.
ACTIONS = (
    (-1,  0), # go up
    ( 0, -1), # go left
    ( 1,  0), # go down
    ( 0,  1)  # go right
)

from utils.search_app import OrderedSet, Stack, Queue, PriorityQueue
"""
 Four different structures are provided as the containers of the open set
 and closed set.

 OrderedSet is an ordered collections of unique elements.

 Stack is an LIFO container whose `pop()` method always pops out the last
 added element. 

 Queue is an FIFO container whose `pop()` method always pops out the first
 added element.

 PriorityQueue is a key-value container whose `pop()` method always pops out
 the element whose value has the highest priority.

 All of these containers are iterable but not all of them are ordered. Use
 their pop() methods to ensure elements are popped out as expected.


 Common operations of OrderedSet, Stack, Queue, PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s


 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s;
             nothing will be done if x is already in s
   s.pop(): return and remove the LAST added element in s;
            raise IndexError if s is empty 
   s.pop(last=False): return and remove the FIRST added element in s;
            raise IndexError if s is empty 
 Example:
   s = Set()
   s.add((1,2))    # add a tuple element (1,2) into the set
   s.remove((1,2)) # remove the tuple element (1,2) from the set
   s.add((1,1))
   s.add((2,2))
   s.add((3,3))
   x = s.pop()
   assert(x == (3,3))
   assert((1,1) in s and (2,2) in s)
   assert((3,3) not in s)
   x = s.pop(last=False)
   assert(x == (1,1))
   assert((2,2) in s)
   assert((1,1) not in s)
   

 Unique operations of Stack:
   s.append(x): add the element x into the back of the stack s
   s.pop(): return and remove the LAST added element in the stack s;
            raise IndexError if s is empty
 Example:
   s = Stack()
   s.add((1,1))
   s.add((2,2))
   x = s.pop()
   assert(x == (2,2))
   assert((1,1) in s)
   assert((2,2) not in s)


 Unique operations of Queue:
   s.append(x): add the element x into the back of the queue s
   s.pop(): return and remove the FIRST added element in the queue s;
            raise IndexError if s is empty
 Example:
   s = Queue()
   s.add((1,1))
   s.add((2,2))
   x = s.pop()
   assert(x == (1,1))
   assert((2,2) in s)
   assert((1,1) not in s)


 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="max", f=lambda v: abs(v))
   s.put((1,1), -1)
   s.put((2,2), -20)
   s.put((3,3), 10)
   x, v = s.pop()  # the element with maximum value of abs(v) will be popped
   assert(x == (2,2) and v == -20)
   assert(x not in s)
   assert(x.get((1,1)) == -1)
   assert(x.get((3,3)) == 10)
"""


# use math library if needed
import math

# define neighbor function to generate child node

def negh(r,c,nrows,ncols,obstacles):
    t=()
    m={}
    t1=list(t)
    if r-1>=0 and (r-1,c) not in obstacles:
        t1.append((r-1,c))
        m.update({(r-1,c):(-1,0)})
    if c-1>=0 and (r,c-1) not in obstacles:
        t1.append((r,c-1))
        m.update({(r,c-1):(0,-1)})
    if r+1<nrows and (r+1,c) not in obstacles:
        t1.append((r+1,c))
        m.update({(r+1,c):(1,0)})
    if c+1<ncols and (r,c+1) not in obstacles:
        t1.append((r,c+1))
        m.update({(r,c+1):(0,1)})
    return t1,m

#funcation to reverse tuple

def Reverse(tuples): 
    new_tup = () 
    for k in reversed(tuples): 
        new_tup = new_tup + (k,) 
    return new_tup


def depth_first_search(grid_size, start, goal, obstacles, costFn, logger):
    """
    DFS algorithm finds the path from the start cell to the
    goal cell in the 2D grid world.
    
    Parameters
    ----------
    grid_size: tuple, (n_rows, n_cols)
        (number of rows of the grid, number of cols of the grid)
    start: tuple, (row, col)
        location of the start cell;
        row and col are counted from 0, i.e. the 1st row is 0
    goal: tuple, (row, col)
        location of the goal cell
    obstacles: tuple, ((row, col), (row, col), ...)
        locations of obstacles in the grid
        the cells where obstacles are located are not allowed to access 
    costFn: a function that returns the cost of landing to a cell (x,y)
         after taking an action. The default cost is 1, regardless of the
         action and the landing cell, i.e. every time the agent moves NSEW
         it costs one unit. 
    logger: a logger to visualize the search process.
         Do not do anything to it.

   

    Returns
    -------
    movement along the path from the start to goal cell: list of actions
        The first returned value is the movement list found by the search
        algorithm from the start cell to the end cell.
        The movement list should be a list object who is composed of actions
        that should made moving from the start to goal cell along the path
        found the algorithm.
        For example, if nodes in the path from the start to end cell are:
            (0, 0) (start) -> (0, 1) -> (1, 1) -> (1, 0) (goal)
        then, the returned movement list should be
            [(0,1), (1,0), (0, -1)]
        which means: move right, down, left.

        Return an EMPTY list if the search algorithm fails finding any
        available path.
        
    closed set: list of location tuple (row, col)
        The second returned value is the closed set, namely, the cells are expanded during search.
    """
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    ##########################################
    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = Stack()
    closed_set = OrderedSet()
    ##########################################

    ##########################################
    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    ##########################################

    movement = []
    # ----------------------------------------
    # finish the code below
    # ----------------------------------------
    
    # dictionary to store neighbor and action 
    temp_movement={}
    movement_rev=[]
    
    #put start node into the open_set
    
    open_set.add((start_row, start_col))
    
    #looping until the open_set is empty 
   
    while len(open_set)!=0:
        node=open_set.pop()
        closed_set.add(node)
        current_row , current_col = node
    #call neighbor funcation to get child/neighbor and actions of current node 
        list_neg,list_mom=negh(current_row,current_col,n_rows,n_cols,obstacles)
        for x in range(len(list_neg)):
            if list_neg[x] not in closed_set and list_neg[x] not in open_set: 
                temp_movement.update({list_neg[x]:list_mom[list_neg[x]]}) 
                if list_neg[x]==goal:
                    temp=goal
    # code to traceback path form the goal to start
                    while temp!=start:
                        temp_row,temp_col=temp
                        temp1_row,temp1_col=temp_movement[temp]
                        temp_row=temp_row-(temp1_row)
                        temp_col=temp_col-(temp1_col)
                        movement_rev.append(temp_movement[temp])
                        temp=temp_row,temp_col
                        movement=Reverse(movement_rev)
                    return movement, closed_set
                else:
                    open_set.add(list_neg[x])
#######################################################################

#############################################################################
    return movement, closed_set

def uniform_cost_search(grid_size, start, goal, obstacles, costFn, logger):
    """
    Uniform-cost search algorithm finds the optimal path from 
    the start cell to the goal cell in the 2D grid world. 
    
    After expanding a node, to retrieve the cost of a child node at location (x,y), 
    please call costFn((x,y)). In all of the grid maps, the cost is always 1.

    See depth_first_search() for details.
    """
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    ##########################################
    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = PriorityQueue()
    closed_set = OrderedSet()
    ##########################################

    ##########################################
    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    ##########################################

    movement = []
    
    # ----------------------------------------
    # finish the code below
    # ----------------------------------------
    
    # dictionary to store neighbor and action 
    
    temp_movement={}
    movement_rev=[]
    
    #put start node into the open_set 
    
    open_set.put((start_row, start_col),0)
    
    #looping until open_set is empty 
   
    while(len(open_set))!=0:
        node=open_set.pop()
        noderc , node_val = node
        current_row,current_col=noderc
        if noderc==goal:
            temp=goal
    # code to traceback path form the goal to start
    
            while temp!=start:
                temp_row,temp_col=temp
                temp1_row,temp1_col=temp_movement[temp]
                temp_row=temp_row-(temp1_row)
                temp_col=temp_col-(temp1_col)
                movement_rev.append(temp_movement[temp])
                temp=temp_row,temp_col
                movement=Reverse(movement_rev)
            return movement,closed_set
        closed_set.add(noderc)
    
    #call neighbor function to get child/neighbor and actions of current node 
    
        list_neg,list_mom=negh(current_row,current_col,n_rows,n_cols,obstacles)
        for x in range(len(list_neg)):
            child_row, child_col=list_neg[x]
            if list_neg[x] not in open_set and list_neg[x] not in closed_set: 
                open_set.put(list_neg[x],costFn(list_neg[x]))
                temp_movement.update({list_neg[x]:list_mom[list_neg[x]]}) 
            elif list_neg[x] in open_set and costFn((child_row,child_col))>node_val:
                noderc=list_neg[x]
                   
#############################################################################

#############################################################################
    return movement, closed_set

def astar_search(grid_size, start, goal, obstacles, costFn, logger):
    """
    A* search algorithm finds the optimal path from the start cell to the
    goal cell in the 2D grid world.

    After expanding a node, to retrieve the cost of a child node at location (x,y), 
    please call costFn((x,y)). In all of the grid maps, the cost is always 1.  

    See depth_first_search() for details.    
    """
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    ##########################################
    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = PriorityQueue()
    closed_set = OrderedSet()
    ##########################################

    ##########################################
    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    ##########################################

  
    # ----------------------------------------
    # finish the code below to implement a Manhattan distance heuristic
    # ----------------------------------------
    def heuristic(row, col):
        return abs(row - goal_row) + abs(col - goal_col)

    
   
    movement = []
    temp_movement={}
    movement_rev=[]

    #put start node into the open_set with fval 0
    open_set.put((start_row,start_col),0)
    
    #looping until open_set is empty 
    while len(open_set) != 0:
        node = open_set.pop()

        curr_node, node_fval = node

        curr_row, curr_col = curr_node

        if curr_node == goal:
            temp = goal
            
    # code to traceback path form the goal to start
    
            while temp!=start:
                temp_row,temp_col=temp
                temp1_row,temp1_col=temp_movement[temp]
                temp_row=temp_row-(temp1_row)
                temp_col=temp_col-(temp1_col)
                movement_rev.append(temp_movement[temp])
                temp=temp_row,temp_col
                movement=Reverse(movement_rev)
            return movement,closed_set
        
        closed_set.add(curr_node)

        neighbors,my_move = negh(curr_row, curr_col, n_rows, n_cols, obstacles)

        
        for next_node in range(len(neighbors)):
            total_cost = 0
            child_row, child_col = neighbors[next_node]

            #to calculate the F-value: for A* F(node) = g(node) + h(node)
            #cost_g is the sum of path cost between current node and child_node + path_cost of current node
            #and h(node) is heuristics value between child_node and goal node
            
            cost_g = costFn((child_row, child_col))+ (node_fval - heuristic(curr_row, curr_col))
            cost_h = heuristic(child_row, child_col)
            total_cost = cost_g + cost_h

            if neighbors[next_node] not in open_set and neighbors[next_node] not in closed_set:
                #insert child_node and f_value into openset
                open_set.put(neighbors[next_node], total_cost)
                temp_movement.update({neighbors[next_node]:my_move[neighbors[next_node]]})
                
            elif neighbors[next_node] in open_set and total_cost < node_fval+heuristic(curr_row,curr_col):
                curr_node = neighbors[next_node]
                
    return movement, closed_set

if __name__ == "__main__":
    # make sure actions and cost are defined correctly
    from utils.search_app import App
    assert(ACTIONS == App.ACTIONS)

    import tkinter as tk

    algs = {
        "Depth-First": depth_first_search,
        "Uniform Cost Search": uniform_cost_search,
        "A*": astar_search
    }

    root = tk.Tk()
    App(algs, root)
    root.mainloop()

