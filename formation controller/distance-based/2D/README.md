# 2D Distance-Based Formation Control

Run `distance_based_formation_2D.m` from MATLAB. The script simulates a regular N-gon in the plane for four graph topologies:

- complete
- minimally globally rigid
- minimally rigid
- non-rigid

The undirected edge counts are complete: `n(n-1)/2`, minimally globally rigid: `3n-6`, minimally rigid: `2n-3`, and non-rigid: `2n-4`.

Edit the configuration section at the top of the script to change `nAgents`, the polygon radius, simulation parameters, or `initialPositions`. Use an `nAgents`-by-2 matrix for custom initial conditions. Leave it as `[]` to use the built-in deterministic initialization.

Every run deletes and recreates the `result` directory. Each graph case receives its own folder containing:

- `desired_shape.fig` and `desired_shape.png`
- `trajectories.fig` and `trajectories.png`
- `distance_error_convergence.fig` and `distance_error_convergence.png`
- `animation.mp4`

The trajectory figure shows colored agent paths and the final converged graph with annotated vertices. The animation is a compact H.264 MP4 compatible with GitHub and modern PowerPoint versions. It is slowed and includes trails, graph edges, vertex annotations, and an agent legend. The animation and convergence figure stop at the first time all future edge errors remain below `0.001`. The convergence figure plots `||p_i-p_j||^2-d_ij^2` for every graph edge.

The controller uses the squared-distance gradient law

`u_i = -k sum_j (||p_i-p_j||^2-d_ij^2)(p_i-p_j)`.