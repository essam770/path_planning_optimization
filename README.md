# path_planning_optimization
The problem of AGV path optimization is usually described
as: In a warehouse, workshop, or factory. There are AGVs (n)
and shelves/production lines (m). the number of shelves shall
be greater than the number of AGVs transporting. m ¿ n. The
AGVs shall receive orders from the FMS and select the best
path in terms of the shortest distance to be covered by the
AGV in order to transport material from/to shelves/production
lines. This makes the objective function for us is to minimize
the distance covered by each AGV and the time taken by
each AGV.
In order to formulate our problem correctly. The Optimization problem must have several assumptions made first in order
to start executing algorithms These assumptions are:
• AGV speed is consistent during operation
• All equipment and machines are available
• The distance between all shelves is equal
• AGV does not need to return to the starting point after
performing tasks, but directly executes the next task in
the queue, and does not return to the warehouse until all
tasks are complete
• Maps of factories which contain all distances and paths
are pre-given in our problem which assumed to be handed
by the factory itself
• no recharging at stations
• All AGVs are reliable and unbreakable
• All AGVs are bidirectional
• Time of Loading/unloading will be considered constant
Our Optimization algorithm will consist of key factors which
will be considered as the main elements for our study and our
analysis.
• Cost: is what it takes to perform a certain task and is
not necessarily dependent on money. It could be time,
distance, fuel consumption or battery usage, etc.
• Fitness: is how well a certain task could be done by a
certain candidate, this could be how well a certain robot
or machine can perform a task
• Priority: is the necessity or urgency of doing a particular
task
In our specific problem. Simulated annealing algorithm, Genetic Algorithm, Discrete particle swarm oprimization and
African Buffalo optimization were all implemented to test
and see the results and which optimization technique yielded
the best results in terms of cost function which means
min(distance,time). Also, 3 other performnace metrics were
applied to test and see the effect of each optimization technique on our problem.
