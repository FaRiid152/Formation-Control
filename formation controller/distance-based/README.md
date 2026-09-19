# Distance-Based Formation Controller

[Back to project](../../README.md) | [Open the 2D implementation](2D/README.md)

This directory contains formation controllers that regulate inter-agent distances over a sensing graph. The active implementation is planar; the 3D extension is deferred.

## Contents

- [Problem Formulation](#problem-formulation)
- [Gradient Controller](#gradient-controller)
- [Graph Rigidity](#graph-rigidity)
- [Lyapunov Stability](#lyapunov-stability)
- [What the Proof Guarantees](#what-the-proof-guarantees)
- [Implementations](#implementations)

## Problem Formulation

Let $G=(\mathcal{V},\mathcal{E})$ be an undirected graph with $n$ agents. Agent $i$ has position $p_i\in\mathbb{R}^d$. For every measured edge $(i,j)\in\mathcal{E}$, prescribe a desired distance $d_{ij}>0$ and define

$$
e_{ij}=\lVert p_i-p_j\rVert^2-d_{ij}^2.
$$

Stack the positions into $p\in\mathbb{R}^{dn}$ and the $m=|\mathcal{E}|$ edge errors into $e(p)\in\mathbb{R}^m$. The control objective is

$$
e(p)\rightarrow 0,
$$

which means that every graph edge reaches its desired length. Absolute position and orientation are intentionally unspecified, so translated, rotated, and reflected realizations have the same distance constraints.

## Gradient Controller

Use the nonnegative potential

$$
V(p)=\frac{1}{4}\sum_{(i,j)\in\mathcal{E}}e_{ij}^2
     =\frac{1}{4}e(p)^\mathsf{T}e(p).
$$

The contribution of edge $(i,j)$ to agent $i$ is

$$
\frac{\partial V}{\partial p_i}
=e_{ij}(p_i-p_j).
$$

The negative-gradient controller is therefore

$$
\dot p_i=-k\sum_{j\in\mathcal{N}_i}
 e_{ij}(p_i-p_j),\qquad k>0.
$$

This is distributed with respect to the graph: agent $i$ only requires relative positions and desired distances for its neighbors.

## Graph Rigidity

The rigidity matrix $R(p)$ is the Jacobian of the half squared-edge-length map. With this convention,

$$
\nabla_p V=R(p)^\mathsf{T}e(p).
$$

A planar framework is infinitesimally rigid when

$$
\operatorname{rank}R(p)=2n-3.
$$

The three missing rank directions correspond to two translations and one rotation. A generic minimally rigid planar graph has exactly $2n-3$ edges and satisfies the Laman sparsity conditions. Adding redundant edges can improve robustness and may support global rigidity.

For generic planar frameworks with at least four vertices, global rigidity requires more than an edge count: the graph must be 3-connected and redundantly rigid. Consequently, labels such as "minimally globally rigid" in this project identify the requested graph family and edge count; the count $3n-6$ alone is not a general proof of global rigidity. The generated framework should be checked through graph structure and rigidity-matrix rank when a formal certificate is required.

Rigidity has the following practical interpretation:

- A rigid graph locally preserves the complete shape when its measured distances are fixed.
- A globally rigid graph determines a unique realization up to rigid transformations and reflection.
- A non-rigid graph permits continuous shape deformations while preserving all measured edge lengths.

## Lyapunov Stability

The closed-loop stacked dynamics are

$$
\dot p=-kR(p)^\mathsf{T}e(p).
$$

Differentiating the potential along a trajectory gives

$$
\begin{aligned}
\dot V
&=\nabla_p V^\mathsf{T}\dot p\\
&=\left(R^\mathsf{T}e\right)^\mathsf{T}
   \left(-kR^\mathsf{T}e\right)\\
&=-k\left\lVert R^\mathsf{T}e\right\rVert^2\leq 0.
\end{aligned}
$$

Therefore, $V$ is nonincreasing and the trajectories approach the largest invariant subset of

$$
\left\{p:R(p)^\mathsf{T}e(p)=0\right\}.
$$

At a desired formation, $e=0$, so $V=0$ and the formation is an equilibrium. The edge-error dynamics are

$$
\dot e=2R\dot p=-2kRR^\mathsf{T}e.
$$

Near a correct, generic, minimally rigid framework, $R$ has full row rank. Hence $RR^\mathsf{T}$ is positive definite, and the linearized error system is exponentially stable:

$$
\lVert e(t)\rVert
\leq
\exp\!\left(-2k\lambda_{\min}(RR^\mathsf{T})t\right)
\lVert e(0)\rVert.
$$

The formation thus converges locally and exponentially in edge-error coordinates, modulo translation, rotation, and reflection.

## What the Proof Guarantees

The Lyapunov argument proves monotonic decrease of the distance-error potential. It does not by itself establish global convergence from every initial condition because undesired critical points can satisfy $R^\mathsf{T}e=0$ with $e\neq0$. Degenerate or collocated initial configurations can also violate the local rigidity assumptions.

The simulations report convergence of measured edges. In the non-rigid case, zero measured-edge error does not imply that unmeasured pairwise distances or the complete polygon shape are correct.

## Implementations

- [2D controller, graph cases, and run instructions](2D/README.md)
- 3D controller: deferred and currently excluded from version control
