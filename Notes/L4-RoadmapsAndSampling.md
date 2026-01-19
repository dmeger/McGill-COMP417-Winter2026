# Roadmaps and Sampling Planners

So far we've applied standard CS algorithms that work on any graph planning problem, from playing board games to scheduling jobs on a CPU. That is, they were not very "robotic". This week we'll deviate and mention methods that take specific advantage of the continuous, geometric nature of the robot planning problem. Things to notice:
- In Euclidean spaces, the shortest path through free space is easy to find and is optimal. We might always start planning by checking a straight line start-goal, and many planning instances can be solved immediately.
- Of course, obstacles will fall along this path in all interesting cases. However, the same reasoning extends to sub-problems when navigating obstacles:
    - The best way to pass from vertex $v_1$ to $v_2$ is also a straight line. In 2D, the only way to go "around" an obstacle is by touching the vertices that form its **convex hull**. 
    - In higher dimensions, the shortest path might pass over the edges or vertices of obstacles, but regardless, these entities are our key inputs to planning. The description of obstacles and their boundaries can be viewed as at least equally important to planning as their dual, the free space.

A problem with extending the discrete, anonymous graph algorithms of the previous sections to robotics problems is that we must discretize the naturally continuous, and potentially high dimensional state and action spaces of our robot. Discretization simply means dividing continuous space up into finitely many, usually regularly spaced locations and connecting them up based on the kinematics and obstacle information of our planning problem. This discrete graph grows exponentially in the planning dimensions because there are more and more factors to divide evenly. A unit 
line in 1D can be broken into K even line segments with length $1/K$. A unit plane takes $K^{2}$ small squares of equal side length and a unit cube $K^{3}$ small cubes. We very quickly want to avoid this data structure and almost never use a 6-D even grid to consider the motions of our Kinova arms, for example.

Instead, planners that are aware of being run in metric spaces can use various elements of geometry: simplifications, known invariant properties, useful data structures and more in order to find more efficient representations.

## Algorithm 1: Voronoi Graph

A Voronoi Diagram/Graph (VG) is the data structure formed by edges and curves that are equidistant from at least two elements of the free space boundary. Examples are shown on the slides and easy to obtain. The VG can be thought of as a skeleton, lying at the centre of the free space. It's an important data structure for many elements of geometry processing. It is used in robotic navigation, especially when we want robots to be maximally safe. The idea is to build the VG, then connect every element of free space up to its network. We move from the start to a VG vertex or curve (vertices are easier to identify in a simple algorith, but curves are usually closer). Then along the VG, until we apply the inverse operation to reach the goal.

## Algorithm 2: Visibility Graph

Notes to be continued for Wednesday.




