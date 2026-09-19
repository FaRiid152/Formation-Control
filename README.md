# Distance-Based Formation Control

MATLAB simulation for N-agent distance-based formation control in 2D. The target formation is a regular polygon, and the controller is evaluated with complete, minimally globally rigid, minimally rigid, and non-rigid graph topologies.

## Project layout

```text
formation controller/
   distance-based/
      2D/
         distance_based_formation_2D.m
         README.md
         result/
```

   The 3D implementation is deferred and excluded through `.gitignore` while development focuses on the planar controller.

## How to run

1. Open MATLAB.
2. Open `formation controller/distance-based/2D`.
3. Run `distance_based_formation_2D`.
4. Edit the configuration section at the top of the script to set the number of agents, polygon radius, initial positions, time step, duration, and gain.

Each run overwrites the matching `result` directory. Every graph case produces a desired-shape figure, a trajectory figure, a per-edge distance-error convergence figure, and a compact H.264 MP4 animation for GitHub and PowerPoint. Error plots and animations stop once every edge error remains below `0.001`. The trajectory figure shows the final graph over the converged formation; the animation includes agent trails, vertex annotations, graph edges, and an agent legend.

## Controller

For every measured graph edge, the gradient controller uses

`u_i = -k sum_j (||p_i-p_j||^2-d_ij^2)(p_i-p_j)`.

The root prototype has been removed; the dimension-specific scripts are the supported entry points.
