# Tension Distribution for a Planar Four-Cable Robot

MATLAB implementations of tension distribution for a planar four-cable robot. The proposed **Direct Method (DM)** minimizes the cable-tension L2 norm subject to force equilibrium and tension bounds. It applies to feasible force requests, with four counterclockwise-ordered anchors and an end-effector position strictly inside their convex frame.

## Quick start

Download and extract the repository, then set MATLAB's **Current Folder** to the folder containing this README. Run:

```matlab
addpath('examples');
result = run_example;
```

The example requires **MATLAB only**. It displays and returns the requested force, cable tensions, achieved force, equilibrium error, and tension-bound checks.

Positions are expressed in **mm**; forces and tensions are in **N**. The example uses a feasible force request and tension bounds of **3.8–25 N**. The expected equilibrium error is below `1e-8` N.

## Method

DM expresses cable tensions as a particular solution plus a two-dimensional null-space contribution. The tension bounds define feasible scalar intervals. At most two clipped scalar candidates are evaluated, and the minimum-norm tension vector is returned in the original cable order.

The [implementation](src/wt_direct.m) follows Algorithm 1 and the corresponding equations in the manuscript.

## Comparison methods

The six-method example requires **MATLAB and Optimization Toolbox**:

```matlab
addpath('examples');
comparison = run_example('all');
```

| Method | Implementation |
| --- | --- |
| Direct Method (DM) | [`wt_direct.m`](src/wt_direct.m) |
| Improved closed-form method (ICFM) | [`wt_pott.m`](src/wt_pott.m) |
| Bounded quadratic programming (QP) | [`wt_qp_bounded.m`](src/wt_qp_bounded.m) |
| Vertex-based VTDA-L2 | [`wt_gouttefarde.m`](src/wt_gouttefarde.m) |
| Analytic-centre ACTA | [`wt_acta.m`](src/wt_acta.m) |
| Extended X-ACTA | [`wt_xacta.m`](src/wt_xacta.m) |

The first five methods receive a common wrench-exertion-capability (WEC) target; X-ACTA receives the original force. WEC leaves the feasible example request unchanged. ACTA and X-ACTA minimize analytic-centre objectives and are implemented from the published equations. Equivalence to the original authors' software has not been established.

Bounded QP uses MATLAB's **active-set** algorithm, initialized at the minimum tension for each cable: `x0 = 3.8*ones(4,1)` in this example. A feasibility phase is performed when the initial vector does not satisfy force equilibrium.

## Experimental forward kinematics

The MATLAB-only example reconstructs five known positions from cable lengths:

```matlab
addpath('examples');
position_result = run_forward_kinematics_example;
```

Cable lengths, calibrated motor displacements, positions, and errors are expressed in **mm**. The [forward-kinematics reference](docs/FUNCTION_REFERENCE.md#experimental-forward-kinematics) defines the coordinates, cable numbering, and fixed adjacent circle pairs. Standalone use requires positive cable lengths with real intersections and a real, finite returned position.

Forward kinematics estimates position separately; each tension distribution function receives position as an input. Forward kinematics is excluded from the reported tension distribution execution times.

## Documentation

- [Function reference](docs/FUNCTION_REFERENCE.md): inputs, units, solver settings, and status definitions.
- [Method sources](docs/METHOD_SOURCES.md): mathematical references and implementation scope.
- [Notices](NOTICES.md): attribution and licensing.
