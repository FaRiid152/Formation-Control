# Distance-Based Formation Control

MATLAB project for studying gradient-based formation control under different sensing-graph constraints. The active implementation controls an arbitrary number of planar agents toward a regular polygon using desired inter-agent distances.

## Contents

- [Project Structure](#project-structure)
- [Active Implementation](#active-implementation)
- [Results](#results)
- [Status](#status)

## Project Structure

```text
formation controller/
   distance-based/
         README.md
      2D/
         distance_based_formation_2D.m
         README.md
         result/
            complete/
            minimally_globally_rigid/
            minimally_rigid/
            non_rigid/
```

## Active Implementation

Read the documentation in order of detail:

1. [Distance-based controller theory, rigidity, and Lyapunov stability](formation%20controller/distance-based/README.md)
2. [2D graph cases, configuration, and run instructions](formation%20controller/distance-based/2D/README.md)
3. The self-contained result reports linked below

## Results

Each result directory is a self-contained report with its own explanation, desired graph, trajectories, edge-error convergence, and animation:

- [Complete graph](formation%20controller/distance-based/2D/result/complete/README.md)
- [Minimally globally rigid graph](formation%20controller/distance-based/2D/result/minimally_globally_rigid/README.md)
- [Minimally rigid graph](formation%20controller/distance-based/2D/result/minimally_rigid/README.md)
- [Non-rigid graph](formation%20controller/distance-based/2D/result/non_rigid/README.md)

## Status

Development currently focuses on the planar controller. The 3D implementation is deferred and excluded through `.gitignore`.
