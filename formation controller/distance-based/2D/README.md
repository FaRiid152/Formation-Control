# 2D Distance-Based Formation Control

[Back to project](../../../README.md) | [Controller theory and stability](../README.md)

This directory contains the planar graph construction, simulation, and result-generation logic. The general control law, rigidity definitions, and Lyapunov analysis are documented in the [distance-based controller README](../README.md).

## Contents

- [Graph Cases](#graph-cases)
	- [Complete](#complete)
	- [Minimally Globally Rigid](#minimally-globally-rigid)
	- [Minimally Rigid](#minimally-rigid)
	- [Non-Rigid](#non-rigid)
- [Configuration](#configuration)
- [Run](#run)
- [Results](#results)
- [Output Convention](#output-convention)

## Graph Cases

The script evaluates four graph topologies:

| Case | Number of edges | Default with $n=5$ |
| --- | ---: | ---: |
| Complete | $n(n-1)/2$ | 10 |
| Minimally globally rigid | $3n-6$ | 9 |
| Minimally rigid | $2n-3$ | 7 |
| Non-rigid | $2n-4$ | 6 |

The desired distances are calculated from a regular $n$-gon. The current implementation deterministically selects the requested number of edges from the complete graph and validates the count before simulation. Edge count is a comparison convention here; formal rigidity additionally depends on graph structure and generic framework rank.

### Complete

The complete graph $K_n$ measures every pairwise distance. It is highly redundant and supplies the strongest constraint set, but its $O(n^2)$ edge count also creates the highest sensing and computation cost.

[View complete-graph results](result/complete/README.md)

### Minimally Globally Rigid

This project uses the requested $3n-6$ edge count. For the default five-agent case, this is $K_5$ with one edge removed. The graph has fewer constraints than the complete graph while retaining substantial redundancy.

The label describes this project case. For arbitrary $n$, the edge count alone is not a global-rigidity certificate; 3-connectivity and redundant rigidity must also be checked.

[View minimally globally rigid results](result/minimally_globally_rigid/README.md)

### Minimally Rigid

The $2n-3$ count is necessary for a generic minimally rigid planar framework. A graph with this count is minimally rigid only when it also satisfies the Laman sparsity conditions and the realized rigidity matrix has rank $2n-3$.

[View minimally rigid results](result/minimally_rigid/README.md)

### Non-Rigid

This case uses $2n-4$, one edge fewer than the minimally rigid count. The available distances can converge while the full geometry remains underconstrained. This case highlights the distinction between controlling measured distances and uniquely recovering the desired formation.

[View non-rigid results](result/non_rigid/README.md)

## Configuration

Edit the first section of [`distance_based_formation_2D.m`](distance_based_formation_2D.m) to configure:

- Number of agents and polygon radius.
- Initial positions, simulation duration, integration step, and gain.
- Convergence threshold used to truncate exported transients.
- Animation frame spacing, playback rate, quality, and visual styling.

Set `initialPositions = []` to use the deterministic default, or provide an `nAgents`-by-2 matrix.

## Run

Open this directory in MATLAB and run:

```matlab
distance_based_formation_2D
```

The script refreshes generated media while preserving the explanatory README in each result directory.

## Results

- [Complete graph](result/complete/README.md)
- [Minimally globally rigid graph](result/minimally_globally_rigid/README.md)
- [Minimally rigid graph](result/minimally_rigid/README.md)
- [Non-rigid graph](result/non_rigid/README.md)

## Output Convention

Each result directory contains:

- `README.md`: interpretation and embedded media.
- `desired_shape.fig` and `desired_shape.png`: desired polygon and graph.
- `trajectories.fig` and `trajectories.png`: agent paths and converged graph.
- `distance_error_convergence.fig` and `.png`: per-edge signed squared-distance errors.
- `animation.gif`: autoplay preview for GitHub documentation.
- `animation.mp4`: compact H.264 animation for PowerPoint and download.

The convergence plot and animation end when all future measured-edge errors remain below `0.001`.