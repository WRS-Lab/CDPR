# Function reference

The tension distribution functions calculate four cable tensions from a known end-effector position and a requested force. Forward kinematics performs the separate task of estimating position from calibrated motor displacements.

**Start with the [runnable examples](../README.md#quick-start).** This page explains the inputs, returned values, and how to interpret them. Numerical details are grouped under each method.

- [Inputs and coordinates](#inputs-and-coordinates)
- [Outputs and success checks](#outputs-and-success-checks)
- [Direct Method](#direct-method)
- [ICFM](#icfm)
- [Bounded QP](#bounded-qp)
- [VTDA-L2](#vtda-l2)
- [ACTA](#acta)
- [X-ACTA](#x-acta)
- [WEC preprocessing](#wec-preprocessing)
- [Experimental forward kinematics](#experimental-forward-kinematics)

## Inputs and coordinates

The six tension distribution functions share the same calling pattern:

```matlab
output = wt_direct(parameters, position, requested_force);
```

Replace `wt_direct` with the method you want to use. The parameter structure is called `WDM`, the position `P`, and the requested force `F` in the source files. **Only bounded QP needs Optimization Toolbox.** The other functions use base MATLAB; their custom helper functions are included in their source files.

| Input | Shape | Units | Meaning |
| --- | --- | --- | --- |
| `P` | 2 × 1 | mm | Known end-effector position `[x; y]` |
| `F` | 2 × 1 | N | Requested force exerted by the cables |
| `WDM.param.M` | 2 × 4 | mm | Anchor positions, one cable per column |
| `WDM.param.lim_inf` | 4 × 1 | N | Lower tension bounds; `3.8*ones(4,1)` in the example |
| `WDM.param.lim_sup` | 4 × 1 | N | Upper tension bounds; `25*ones(4,1)` in the example |
| `WDM.param.TOLL` | Scalar | Numerical cutoff | Zero/geometric cutoff; `1e-14` in the example |
| `WDM.param.TOLL_WEC` | Scalar | N | WEC inward margin; `1e-9` in the example |
| `WDM.verbose` | Logical scalar | — | Use `false` for ordinary numerical calls |

### Cable numbering

The example uses a frame-centred coordinate system: **+x points right and +y points up**. Cables run counterclockwise, starting at the bottom-left anchor.

| Cable | Anchor | Position `[x, y]` in mm |
| --- | --- | --- |
| 1 | Bottom-left | `[-315, -315]` |
| 2 | Bottom-right | `[315, -315]` |
| 3 | Top-right | `[315, 315]` |
| 4 | Top-left | `[-315, 315]` |

```matlab
WDM.param.M = [-315 315 315 -315; -315 -315 315 315];
```

Each column of the direction matrix `W` is a unit vector pointing from the end effector towards its anchor. Force equilibrium is **`W*T = F`**: `F` is the cable-resultant force, not its negative. Returned tensions keep the original anchor-column order.

The Direct Method assumes a position **strictly inside** the frame. Frame-edge and zero-length-cable cases lie outside its geometric domain. Each tension distribution function receives the position directly; it does not call forward kinematics.

### Bounds and units

The shared four-cable interface assumes equal positive lower bounds and equal upper bounds. Direct, ICFM branching, and WEC read the first bound as a common scalar; their interfaces do not establish support for unequal bounds. X-ACTA additionally validates general full-row-rank `n × m` geometry and scalar or per-cable bounds.

Use the same length unit for `M` and `P`. Changing force or tension units also requires reconsidering dimensional settings such as `delta` and `TOLL_WEC`. `TOLL` is a numerical/geometric cutoff, **not** the force-acceptance tolerance.

<details>
<summary>Passing parameters between calls</summary>

Build each method call from a fresh parameter structure, rather than from another method's returned structure. Returned structures preserve incoming fields. In particular, VTDA-L2 can return without overwriting `T`; a tension vector left over from an earlier call could otherwise be mistaken for a new solution.

No tension distribution method uses a previous solution as a warm start. QP caches only its constant options object. X-ACTA caches only its validated constants/options record. Neither retains a position, force, tension vector, or factorization between calls.

</details>

## Outputs and success checks

| Output | Shape | Units | Meaning |
| --- | --- | --- | --- |
| `T` | 4 × 1 | N | Cable tensions in the original cable order |
| `normT` | Scalar | N | Euclidean norm of the tension vector |
| `errT` | Scalar | N | Force error `norm(W*T-F,2)` |
| `exitflag` | Scalar, when present | — | Solver status; its meaning depends on the method |
| `scale_dir` | Scalar | — | Method-specific diagnostic; see the relevant section below |

**Check the output, the status, and the physics together.** A finite tension vector or a status flag alone does not establish a valid solution. Some routines can raise an error, return nonfinite values, or omit output fields for difficult inputs.

To check that a returned vector reproduces a particular target:

1. Confirm that `T` exists, is numeric, real and finite, and contains four elements.
2. Check that the maximum tension-bound violation is at most **`1e-9` N**.
3. Check that `norm(W*T-F_target,2)` is at most **`1e-8` N**.
4. For QP, ACTA and X-ACTA, also require a positive `exitflag`.

The bound violation is `max([0; lim_inf-T; T-lim_sup])`. These physical checks are separate from each solver's internal stopping rule.

WEC may adjust the requested force, and X-ACTA's extended stage permits force error. Therefore, also inspect **`norm(W*T-F_raw,2)` against the original request**. A bounded, converged output and exact reproduction of the original force are different properties. In particular, X-ACTA can converge without meeting the original-force equality check.

`scale_dir` has no single meaning across methods. A fallback flag is not an optimality certificate. Case numbers, status flags and iteration counts are dimensionless.

## Direct Method

[Source: `wt_direct.m`](../src/wt_direct.m) · [Mathematical reference](METHOD_SOURCES.md#direct-method)

**Purpose:** minimize the actual cable-tension L2 norm for a feasible target in the stated four-cable domain. The method forms a particular solution and null-space basis, chooses a geometric configuration, and evaluates at most two clipped scalar minima. It does not call WEC internally.

The basis and particular solution are placed back in the original cable rows, so the returned `T` needs no inverse permutation.

| Diagnostic | Meaning |
| --- | --- |
| `case = 0` | Numerical `r = 0` |
| `case = 1…4` | Configurations I–IV |
| `case = -1` | Initial double-saturation branch |
| `scale_dir = 1` | Double saturation or an empty candidate interval |
| `scale_dir = 0` | No such diagnostic; still apply the physical checks |

Branches marked by `scale_dir = 1` retain a candidate; they do not invoke a fallback optimizer.

<details>
<summary>Numerical and diagnostic details</summary>

There are no iterative stopping options. Interval consistency uses `1e-12`.

With `verbose=true`, the function also returns `theta_q` and `theta_alphamin` in radians and dimensionless `N1dotN2`. Computing these diagnostics adds a 2 × 2 solve and two `atan2` evaluations. Use `verbose=false` for ordinary calls.

</details>

## ICFM

[Source: `wt_pott.m`](../src/wt_pott.m) · [Mathematical reference](METHOD_SOURCES.md#icfm)

**Purpose:** calculate a bounded tension distribution by projecting the tension midpoint with `pinv`, then clamping and removing violating cables in at most three nested steps. Lower-bound tests precede upper-bound tests. This procedure is not a general L2-optimality certificate.

| Additional output | Meaning |
| --- | --- |
| `scale_dir = 1` | A final bound violation was detected; this flag does not test force equality |
| `active` | Logical cable indicator `T > lim_inf` |
| `cable_n` | Number of cables whose relative lower-bound difference exceeds `1e-6` |
| `case` | `NaN` |
| `param.M_eff` | Copy of the anchor matrix `M` |

No iterative tolerance or iteration budget is exposed. MATLAB's default `pinv` rank tolerance is used.

## Bounded QP

[Source: `wt_qp_bounded.m`](../src/wt_qp_bounded.m) · [Backend reference](METHOD_SOURCES.md#bounded-qp)

**Purpose:** minimize `0.5*T'*T` subject to force equilibrium `W*T=F` and both tension bounds, using MATLAB `quadprog`. Both bounds are retained even when the target is infeasible.

The solver uses **active-set**. Every call starts at `x0 = lim_inf(:)`: `[3.8; 3.8; 3.8; 3.8]` N in the example. This fixed starting vector satisfies the bounds, but may not satisfy equilibrium. `quadprog` performs its feasibility phase when needed.

| Setting | Value |
| --- | --- |
| `Algorithm` | `active-set` |
| `ConstraintTolerance` | `1e-10` |
| `OptimalityTolerance` | `1e-10` |
| `StepTolerance` | `1e-12` |
| `MaxIterations` | `200` |
| `Display` | `off` |

Other options use the installed MATLAB defaults. The documented environment is MATLAB R2024b Update 4 with Optimization Toolbox 24.2; library releases can affect iteration counts and numerical output.

<details>
<summary>Returned diagnostics and failure handling</summary>

| Output | Meaning and units |
| --- | --- |
| `exitflag` | Native `quadprog` status; require a positive value |
| `iter` | Native iteration count |
| `objective` | `0.5*T'*T`, in N² |
| `resid` | Native `firstorderopt`, when available; otherwise `NaN` |
| `primal_resid` | Infinity norm of the force residual, in N |
| `bound_violation` | Maximum tension-bound violation, in N |
| `scale_dir` | `0` only for positive status, bound violation ≤ `1e-9` N, and force error ≤ `1e-8` N; otherwise `-1` |
| `case` | `NaN` |

On failure, `T` may be empty or contain the last iterate. Errors from `quadprog` are not hidden by the wrapper.

</details>

## VTDA-L2

[Source: `wt_gouttefarde.m`](../src/wt_gouttefarde.m) · [Mathematical reference](METHOD_SOURCES.md#vtda-l2)

**Purpose:** search the feasible polygon in the two-dimensional null space for a minimum-norm tension vector. The function computes `null(W)`, forms an explicit generalized inverse, and evaluates polygon vertices and edges.

The polygon walk permits **100 visits** and returns before processing visit 101. It may return without `T` or propagate an error. Always pass fresh parameters and inspect the returned output.

| Additional output | Meaning |
| --- | --- |
| `vertices` | 2 × k numerical null-coordinate vertices, if that stage completes |
| `param.M_eff` | Copy of `M` |
| `fallback = 1` | Minimum-norm feasible vertex among those traced; not a certificate of the overall optimum |
| `scale_dir = 1` | A final bound violation was detected |

There is no rank regularization. Geometric, closure and final bound tests use `TOLL`. Zero fallback or scale flags do not establish convergence or a valid newly computed tension vector.

## ACTA

[Source: `wt_acta.m`](../src/wt_acta.m) · [Mathematical reference](METHOD_SOURCES.md#acta)

**Purpose:** find the analytic centre of the feasible tension set. ACTA minimizes the strict log-barrier objective while enforcing `W*T=F`. It starts at the tension midpoint with `lambda=0` on every call.

**Strict feasibility is required.** A force that can only be produced by tensions on the bounds does not supply an analytic centre.

| `exitflag` | Meaning |
| --- | --- |
| `1` | Converged |
| `0` | Iteration budget reached |
| `-1` | Singular-system safeguard |
| `-3` | Line-search safeguard |

A budget failure does not establish infeasibility. `T` contains the last iterate; `iter` counts accepted Newton updates, and `resid` contains the stopping norm. `scale_dir` is `0` for converged, in-bounds output, `-1` for nonconvergence, and `1` for a bound violation. `case` is `NaN`.

<details>
<summary>Objective, options and convergence rule</summary>

The objective has unit barrier weights:

```text
phi(T) = -sum(log(T-lo) + log(hi-T))
```

Optional fields in `WDM.acta`:

| Field | Default | Meaning |
| --- | --- | --- |
| `tol` | `1e-10` | Absolute combined KKT tolerance |
| `maxit` | `50` | Maximum accepted Newton updates |
| `alpha` | `0.01` | Residual-decrease parameter |
| `beta` | `0.5` | Backtracking reduction factor |
| `maxls` | `60` | Shared maximum step reductions for domain and residual-decrease checks |

The stopping test is:

```text
norm([grad(phi) + W'*lambda; W*T-F], 2) <= tol
```

This combines stationarity and force-equilibrium residuals. It is an **absolute combined KKT test**, not a force-only test or X-ACTA's scaled test. A 2 × 2 Schur determinant safeguard uses `1e-12*S(1,1)*S(2,2)`.

</details>

## X-ACTA

[Source: `wt_xacta.m`](../src/wt_xacta.m) · [Mathematical reference](METHOD_SOURCES.md#x-acta)

**Purpose:** calculate tensions for the original force request, including cases requiring force error to keep tensions strictly within their bounds. It does not require WEC preprocessing.

The relaxed analytic-centre stage (**RAC**) first seeks a solution with exact force equilibrium. If needed, the extended analytic-centre stage (**EAC**) introduces a penalized force-error variable. An unsuccessful RAC stage prevents EAC from running. The final tension vector is never clipped.

| `exitflag` | Meaning |
| --- | --- |
| `1` | Converged |
| `0` | Iteration budget reached |
| `-1` | Rank or factorization safeguard |
| `-2` | Nonfinite state |
| `-3` | Line-search failure |

`stage` is `1` for RAC or `2` for EAC. `scale_dir` is `0` for final flag `1`, otherwise `-1`; `case` is `NaN`. Because EAC allows force error, **native convergence can coexist with nonzero original-force error**.

<details>
<summary>Stages, initialization and options</summary>

RAC minimizes a relaxed log barrier with `W*T=F`, using an affine midpoint projection and null-coordinate Newton iteration. With `k2=1`, the out-of-domain barrier extension is quadratic. Activation is calculated from the RAC solution:

```text
shrunk = clip(T_RAC, lo+delta, hi-delta)
distance = abs(T_RAC-shrunk)
mu = tanh(sum(distance.^(k3+1)))
```

If no distance is positive, RAC is final. Otherwise, EAC minimizes the true log barrier plus `eta*norm(s)^2 + gamma*norm(mu*s)^2`, subject to `W*T + mu*s = F` and strict bounds. EAC starts from the same call's RAC result if it is interior, otherwise from the shrunken projection. Both stages use unit barrier weights.

Optional fields in `WDM.xacta`:

| Field | Default | Meaning |
| --- | --- | --- |
| `delta` | `0.1` N | Interior margin |
| `gamma` | `500` | Force-error penalty parameter |
| `eta` | `1e-5` | Slack penalty parameter |
| `k2`, `k3` | `1`, `1` | Barrier-extension and activation parameters |
| `tol` | `1e-9` | Scaled convergence tolerance |
| `force_tol` | `1e-8` N | Absolute augmented-equality tolerance |
| `maxit` | `100` | Default iteration limit per stage |
| `alpha`, `beta` | `0.01`, `0.5` | Line-search parameters |
| `maxls` | `60` | Maximum line-search index; permits 61 trials, indexed 0–60 |

`rac_maxit` and `eac_maxit` can override `maxit` separately. EAC uses a `0.99` fraction-to-boundary safeguard. Inputs and options are validated: `k2`, `k3` and budgets must be positive integers, and `delta` must be below half the smallest tension range. Unknown options raise errors.

</details>

<details>
<summary>Convergence rule and stationarity allowance</summary>

Let `g=grad(phi)` and `h` be the diagonal Hessian. The stationarity scale and roundoff allowance are:

```text
d = max(1, norm(g,Inf), norm(W'*lambda,Inf))
rho = norm(h.*eps(abs(T)),Inf)/d
stationarity_tolerance = max(tol, min(sqrt(eps), 4*rho))
```

Success requires all three conditions:

```text
norm(g + W'*lambda,Inf)/d <= stationarity_tolerance
norm(rp,Inf)/max(1,norm(F,Inf)) <= tol
norm(rp,Inf) <= force_tol
```

RAC uses `rp=W*T-F`. EAC uses `rp=W*T-F-c*lambda`, where:

```text
a = eta + gamma*mu^2
c = mu^2/(2*a)
s = -mu*lambda/(2*a)
```

EAC's `primal_resid` measures **augmented equilibrium**, not equality to the original force. The Schur formulation avoids division by `mu` near zero. The relative stationarity allowance does not loosen the independent force tests.

</details>

<details>
<summary>Additional returned values</summary>

Here `n` is the number of force coordinates and `m` the number of cables; the example uses `n=2`, `m=4`.

| Output | Shape or meaning |
| --- | --- |
| `achieved_force` | n × 1 vector `W*T`, in N |
| `T_rac` | m × 1 RAC tension vector, in N |
| `rac_lambda`, `lambda` | n × 1 Lagrange-multiplier vectors |
| `s` | n × 1 force-error variable, in N |
| `mu` | Scalar activation |
| `iter` | Total accepted updates |
| `rac_iters`, `eac_iters` | Per-stage accepted updates; also returned as `rac_iter`, `eac_iter` |
| `exitflag`, `rac_exitflag`, `eac_exitflag` | Final and per-stage status |
| `resid` | Maximum of relative stationarity and relative augmented equality |
| `raw_resid` | Unscaled combined KKT Euclidean norm |
| `primal_resid` | `norm(rp,Inf)`, in N |
| `stationarity_resid` | `norm(g+W'*lambda,Inf)` |
| `stationarity_relative`, `stationarity_tolerance`, `roundoff_floor` | Scaled stationarity diagnostics and allowance |
| `rac_resid`, `eac_resid` | Per-stage scaled residuals |
| `barrier_evals`, `rac_barrier_evals`, `eac_barrier_evals` | Full lower/upper barrier evaluation counts |
| `backtracks`, `rac_backtracks`, `eac_backtracks` | Rejected-trial counts, including final failures |

If EAC does not run, its residuals and flag are `NaN` and its work counts are zero.

</details>

## WEC preprocessing

[Source: `WEC.m`](../src/WEC.m) · [Mathematical reference](METHOD_SOURCES.md#wec)

**Purpose:** adjust a requested force using the forces attainable within the cable-tension bounds. WEC returns a force, **not cable tensions**.

```matlab
[F_NEW, IS_SCALED] = WEC(WDM, P, F);
```

`F_NEW` is a 2 × 1 force vector in N. `IS_SCALED` is the routine's scalar change flag; use `norm(F_NEW-F,2)` to measure the actual force change.

For each call, WEC forms the convex hull of the forces at the 16 tension-box vertices. For nonzero `F`, adjustment follows the **signed force line**, with the `TOLL_WEC` inward margin. It can increase the force magnitude or reverse its direction.

| Situation | Behavior |
| --- | --- |
| Unchanged nonzero request | `IS_SCALED` is false |
| Zero request, including one left unchanged | `IS_SCALED` remains true |
| Infeasible zero request | A nonzero fallback is used |
| Force line does not intersect the attainable set | A `NaN` force and a warning are returned |

Finite outputs still need attainable-force verification. Degenerate geometry can raise an error. Keep `verbose=false`: verbose mode opens a diagnostic plot and pauses execution.

## Experimental forward kinematics

[Source: `forward_kinematics.m`](../src/forward_kinematics.m) · [Source information](METHOD_SOURCES.md#experimental-forward-kinematics)

**Purpose:** estimate end-effector position from four calibrated motor displacements, using the fixed adjacent cable pairs from the experimental controller.

```matlab
P = forward_kinematics(M1, M2, M3, M4);
```

| Input or output | Shape | Units | Meaning |
| --- | --- | --- | --- |
| `M1`…`M4` | Four real scalar doubles | mm | Calibrated motor displacements, in controller cable order |
| `P` | 2 × 1 double | mm | Position `[x; y]` relative to the bottom-left corner |

Inputs are assumed to have these types and dimensions; the function does not validate them. Encoder conversion at **150/4000 mm per count** and reference-offset calibration occur upstream in the controller.

### Controller coordinates and cable lengths

The controller uses a **630 mm square**, with its origin at the bottom-left corner, +x right and +y up. Its cable order differs from the tension distribution example:

| Controller cable | Anchor | Position `[x, y]` in mm |
| --- | --- | --- |
| 1 | Bottom-left | `[0, 0]` |
| 2 | Top-left | `[0, 630]` |
| 3 | Top-right | `[630, 630]` |
| 4 | Bottom-right | `[630, 0]` |

With `r0 = 630*sqrt(2)/2`, the absolute cable lengths are:

```matlab
r = [r0-M1; r0+M2; r0-M3; r0+M4];
```

Subtract `[315;315]` from the returned position to obtain frame-centred coordinates. Reorder a controller cable vector with **`[1 4 3 2]`** for use in the tension distribution functions. The motor inputs themselves must retain their controller order and signs.

### Circle intersections and point selection

The function always uses the four adjacent pairs **(1,2), (2,3), (3,4), (4,1)**. In each local frame, the second anchor is at `[0;d]`. Subtracting the circle equations gives:

```text
y = (ri^2 + d^2 - rj^2)/(2*d)
x = sqrt(ri^2 - y^2)
```

The positive root selects the inward-facing intersection. Each point is transformed into the common coordinate system, and the retained points are averaged with equal weights. The intended geometric domain is inside the frame. **There is no quadrant-dependent pair selection or explicit error-propagation weighting.**

### Invalid intersections

Points containing `NaN` are omitted. If none remain, the output is `NaN`. There is no persistent state, previous-position fallback, returned status flag, or explicit input-length validation.

The generated real-valued controller uses real square roots, so a negative argument produces `NaN` and the pair is omitted. Standalone MATLAB can instead return a complex root, which the `NaN` check does not exclude. **For standalone use, supply positive lengths with real adjacent-circle intersections and check that the returned position is real and finite.** This domain check is not a measurement-accuracy guarantee.

The source also calculates diagonal candidates for pairs (1,3) and (2,4), using the constant `892` mm. Neither these candidates nor their selection affect the returned adjacent-pair mean.

Forward kinematics estimates position only; it does not calculate tensions or check tension bounds. Supply its position estimate separately to a tension distribution function. Try the [forward-kinematics example](../README.md#experimental-forward-kinematics).
