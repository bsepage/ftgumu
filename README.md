# ftgumu
Far Target Geolocation Using Multiple UAVs

This thesis addresses a target tracking problem based on video geolocation performed by multiple Unmanned Aerial Vehicles (UAVs). Targets are ground vehicles moving at constant speed and altitude, while UAVs are modelled as Dubins vehicles, i.e. in a 2D plan, with the heading as their unique control
input. They are flying significantly faster than their targets and do not have hovering capabilities. Thus the problem is to design a controller that allows UAVs to compute a trajectory which results in optimal target coverage over time. Two approaches are considered: a deterministic approach implemented
with the dynamic programming algorithm, and a stochastic approach implemented with both the genetic algorithm and the particle-swarm optimiser. The deterministic approach consists of an exhaustive search forward in time causing the controller to use backward induction to find the optimal control
policy. The stochastic approach generates an optimal solution online which results in the design of one-step ahead controllers. The outcome of this thesis consists of Matlab simulations that will compare the performances of each algorithm.

See "semal_msc_thesis.pdf" for further details. Chapter 3 provides a formulation of the state-space and cost function, and describes their discretisation in the scope of the Matlab implementation. Chapter 4 addresses the dynamic programming approach to the target tracking problem, by providing several solutions that result in a reduced computational power. Chapter 5 describes the genetic algorithm and the particle-swarm optimizer implementation. A comparison is provided using the Rastrigin’s function. Several fitness functions are then suggested which can be coupled with nonlinear constraints. Chapter 6 outlines the simulation settings and an analysis of the results.

# How to plot existing results:

- Open the src/getPlots.m file in Matlab
- Choose a simulation file from the sim/ folder
- Drag and drop the chosen simulation file into the Matlab workspace
- Run the src/getPlots.m file
