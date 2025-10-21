# Comparison of Different Path Planning Algorithms

🧭
An intiutive MATLAB based implementation of popular traditional grid-based and search-based path planning algorithms 
🧭

📍 This work has been developed during Autonomous Vehicles master's course in PoliMi under the guidance of Francesco Braghin. For any questions and/or contributions feel free to get in touch! 

## Dijkstra's Algorithm

📍Evaluates shortest distance to the start amongst every neighbour at each iteration (cost-to-go):

$$
\math{q = min[C(q)]}
$$

where $C(q)$ is the cost to arrive at node $q$ at each iteration.

![Translational Dijkstra](image-1.png)

![Diagonal Dijkstra](image-2.png)

## A*

📍Evaluates shortest distance with heuristic additional amongst every neighbout at each iteration (cost-to-go + cost-to-arrive estimation)

$$
\math{q = min[C(q)]}
$$

where $C(q)$ is the cost to arrive at node $q$ at each iteration.

![Simple A*](image-3.png)

![Semi-Heuristic A*](image-4.png)

![Greedy Search](image-5.png)

## RRT

📍Evaluates admissable space configurations on a random sampling basis, hence the name rapidly exploring random trees - RRT.

![RRT](image-6.png)

![RRT - 2nd Search](image-7.png)

## RRT*

📍Evaluates and updates on each iteration possible links based on an heuristic optimization rule allowing for the stochaisticity to be minimalized.

![RRT*](image-9.png)

![RRT* - Extended Search](image-9.png)

# Scripts

The results presented are obtained on a modern CPU. Any Intel/AMD configuration that is from the last 10 years should suffice.

Apart from the proprietary software (MATLAB), there are no other software requirements.

# Maps

It is possible to create custom maps. For this purpose any Microsoft Paint based implementation should suffice where the initial (green), final (red) and obstacle (black) points are clearly identifiable.

Note that the exact pixel locations may not be perfectly identified due to the blurring effects in MS Paint and the implemented color mask.

# Commands

It is possible to run each path finding algorithm referring to its main file and running it through the MATLAB compiler. For example, in order to find sub-optimal paths on a given map using A*, it is enough to navigate to the script

XX

and compile.