# Distance-Based Formation Control

MATLAB simulation for N-agent distance-based formation control in 2D. The target formation is a regular polygon, and the controller is evaluated with complete, minimally globally rigid, minimally rigid, and non-rigid graph topologies.

## Table of Contents

- [Quick Start](#quick-start)
- [Controller](#controller)
- [Results](#results)
   - [Complete Graph](#complete-graph)
   - [Minimally Globally Rigid Graph](#minimally-globally-rigid-graph)
   - [Minimally Rigid Graph](#minimally-rigid-graph)
   - [Non-Rigid Graph](#non-rigid-graph)
- [Output Files](#output-files)
- [Project Layout](#project-layout)

## Quick Start

1. Open MATLAB.
2. Open [`formation controller/distance-based/2D`](formation%20controller/distance-based/2D).
3. Run [`distance_based_formation_2D.m`](formation%20controller/distance-based/2D/distance_based_formation_2D.m).
4. Edit the configuration section at the top of the script to set the number of agents, polygon radius, initial positions, time step, duration, gain, and convergence threshold.

Each run replaces the complete 2D [`result`](formation%20controller/distance-based/2D/result) directory. Error plots and animations stop once every edge error remains below `0.001`.

## Controller

For every measured graph edge, the gradient controller uses

$$
u_i=-k\sum_j\left(\lVert p_i-p_j\rVert^2-d_{ij}^2\right)(p_i-p_j).
$$

The per-edge convergence plots show

$$
e_{ij}=\lVert p_i-p_j\rVert^2-d_{ij}^2.
$$

## Results

The default run uses five agents. Each result includes the desired graph, agent trajectories with the converged graph, per-edge error convergence, and a compact H.264 MP4 animation.

### Complete Graph

The complete graph uses all $n(n-1)/2$ edges, giving 10 constraints for the default five-agent case.

[Open result folder](formation%20controller/distance-based/2D/result/complete) | [Play or download animation](formation%20controller/distance-based/2D/result/complete/animation.mp4)

| Desired shape and graph | Agent trajectories and converged graph |
| --- | --- |
| ![Complete graph desired shape](formation%20controller/distance-based/2D/result/complete/desired_shape.png) | ![Complete graph trajectories](formation%20controller/distance-based/2D/result/complete/trajectories.png) |

![Complete graph edge-error convergence](formation%20controller/distance-based/2D/result/complete/distance_error_convergence.png)

### Minimally Globally Rigid Graph

This case uses $3n-6$ edges, giving 9 constraints for the default five-agent case.

[Open result folder](formation%20controller/distance-based/2D/result/minimally_globally_rigid) | [Play or download animation](formation%20controller/distance-based/2D/result/minimally_globally_rigid/animation.mp4)

| Desired shape and graph | Agent trajectories and converged graph |
| --- | --- |
| ![Minimally globally rigid desired shape](formation%20controller/distance-based/2D/result/minimally_globally_rigid/desired_shape.png) | ![Minimally globally rigid trajectories](formation%20controller/distance-based/2D/result/minimally_globally_rigid/trajectories.png) |

![Minimally globally rigid edge-error convergence](formation%20controller/distance-based/2D/result/minimally_globally_rigid/distance_error_convergence.png)

### Minimally Rigid Graph

This case uses $2n-3$ edges, giving 7 constraints for the default five-agent case.

[Open result folder](formation%20controller/distance-based/2D/result/minimally_rigid) | [Play or download animation](formation%20controller/distance-based/2D/result/minimally_rigid/animation.mp4)

| Desired shape and graph | Agent trajectories and converged graph |
| --- | --- |
| ![Minimally rigid desired shape](formation%20controller/distance-based/2D/result/minimally_rigid/desired_shape.png) | ![Minimally rigid trajectories](formation%20controller/distance-based/2D/result/minimally_rigid/trajectories.png) |

![Minimally rigid edge-error convergence](formation%20controller/distance-based/2D/result/minimally_rigid/distance_error_convergence.png)

### Non-Rigid Graph

The non-rigid comparison removes one edge from the minimally rigid count, leaving $2n-4$ edges, or 6 constraints for the default case. Its measured edge errors converge, but those constraints do not uniquely determine the complete desired shape.

[Open result folder](formation%20controller/distance-based/2D/result/non_rigid) | [Play or download animation](formation%20controller/distance-based/2D/result/non_rigid/animation.mp4)

| Desired shape and graph | Agent trajectories and converged graph |
| --- | --- |
| ![Non-rigid desired shape](formation%20controller/distance-based/2D/result/non_rigid/desired_shape.png) | ![Non-rigid trajectories](formation%20controller/distance-based/2D/result/non_rigid/trajectories.png) |

![Non-rigid edge-error convergence](formation%20controller/distance-based/2D/result/non_rigid/distance_error_convergence.png)

## Output Files

Each graph result folder contains:

- `desired_shape.fig` and `desired_shape.png`: desired regular polygon with graph edges and vertex labels.
- `trajectories.fig` and `trajectories.png`: colored agent paths and the final converged graph.
- `distance_error_convergence.fig` and `distance_error_convergence.png`: signed squared-distance error for every measured edge.
- `animation.mp4`: compact animation with trails, graph edges, vertex labels, and an agent legend.

## Project Layout

```text
formation controller/
   distance-based/
      2D/
         distance_based_formation_2D.m
         README.md
         result/
```

The 3D implementation is deferred and excluded through `.gitignore` while development focuses on the planar controller.
