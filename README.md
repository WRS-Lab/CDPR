# Tension Distribution for a Planar Four-Cable Robot

MATLAB implementations and runnable examples for computing cable tensions subject to force equilibrium and lower and upper tension bounds.

The proposed **Direct Method (DM)** minimizes the cable-tension L2 norm for a feasible force request by evaluating at most two scalar candidates. It assumes four counterclockwise-ordered anchors and an end-effector position strictly inside their convex frame.

## Quick start

1. Download and extract the repository ZIP, or clone the repository.
2. Open MATLAB and set its **Current Folder** to the folder containing this README.
3. Run:

```matlab
addpath('examples');
result = run_example;
```

This example requires **MATLAB only**. It displays the requested force, four computed cable tensions, achieved force, equilibrium error, and whether the tension bounds are satisfied. These values are also returned in `result`.

The example uses a known feasible request, positions in **mm**, forces and tensions in **N**, and cable bounds of **3.8–25 N**. All example checks should pass, with equilibrium error below `1e-8` N.

## How the Direct Method works

Given an end-effector position and a feasible force request, the method:

1. Forms the cable-direction matrix and a particular tension solution.
2. Expresses the remaining freedom using a two-dimensional null-space basis.
3. Converts the cable-tension bounds into feasible scalar intervals.
4. Evaluates at most two clipped scalar candidates and returns the minimum-norm tension vector in the original cable order.

See [`src/wt_direct.m`](src/wt_direct.m) for the implementation and references to the manuscript's equations and Algorithm 1. The [function reference](docs/FUNCTION_REFERENCE.md) explains inputs, outputs, units, assumptions, and status fields.

## Try the other methods

With **Optimization Toolbox** installed, run:

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

The first five methods receive the same wrench-exertion-capability (WEC) target; X-ACTA receives the original force. WEC leaves this example's feasible request unchanged. ACTA and X-ACTA use analytic-centre objectives, which differ from DM's tension L2 objective. They are equation-based MATLAB implementations; equivalence to the original authors' software has not been established. The [method sources](docs/METHOD_SOURCES.md) provide attribution.

Bounded QP uses MATLAB's **active-set** algorithm, starting every cable at its minimum tension: `x0 = 3.8*ones(4,1)` in this example. The solver performs a feasibility phase when this initial vector does not satisfy force equilibrium.

## Experimental forward kinematics

Reconstruct five known positions from cable lengths with this MATLAB-only example:

```matlab
addpath('examples');
position_result = run_forward_kinematics_example;
```

It displays cable lengths, calibrated motor displacements, reconstructed positions, and position errors, all in **mm**. The [function help](src/forward_kinematics.m) documents the controller's cable numbering, coordinate system, fixed adjacent circle pairs, and invalid-intersection handling. The example uses positive lengths with real circle intersections; check that the returned position is real and finite for other inputs.

Forward kinematics supplies a position estimate separately. Each tension distribution function receives the end-effector position as an input.

## Repository guide

| Location | Contents |
| --- | --- |
| [`src/`](src/) | Tension distribution, WEC, and forward-kinematics functions |
| [`examples/`](examples/) | Small runnable examples with equilibrium, bound, and position checks |
| [Function reference](docs/FUNCTION_REFERENCE.md) | Dimensions, units, cable ordering, solver settings, and returned statuses |
| [Method sources](docs/METHOD_SOURCES.md) | Mathematical references and implementation scope |
| [Notices](NOTICES.md) | Attribution and license information |

Returned tensions follow the anchor-column order. For other inputs, check the returned status, force equilibrium, and tension bounds together. WEC may change the requested force or return no valid target; its interface and assumptions are described in the function reference.
