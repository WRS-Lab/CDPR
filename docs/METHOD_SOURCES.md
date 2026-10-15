# Method sources

## Implementation scope

The references below identify the mathematical basis of the [MATLAB implementations](../src/). The [function reference](FUNCTION_REFERENCE.md) specifies their interfaces, assumptions, solver settings, and status definitions. Equivalence to the original authors' software has not been established.

## Direct Method

Implementation: [`wt_direct.m`](../src/wt_direct.m) · [Function reference](FUNCTION_REFERENCE.md#direct-method)

The implementation follows Algorithm 1 (p. 6) of *Efficient L2-Optimal Tension Distribution in Planar Translational Cable Driven Parallel Robots with 2 Degrees of Redundancy*:

- Particular solution: Eq. (6).
- Null-space basis: Eq. (7).
- Feasible intervals: Eqs. (11)–(12).
- Reduced scalar objective: Eq. (13).
- Projection: Eq. (14).

The implementation forms `Pm = N'*N`. Configurations II and III select the `alpha2` and `alpha1` axes, respectively. WEC is applied separately.

## ICFM

Implementation: [`wt_pott.m`](../src/wt_pott.m) · [Function reference](FUNCTION_REFERENCE.md#icfm)

- **Manuscript reference [27]:** A. Pott, “An improved force distribution algorithm for over-constrained cable-driven parallel robots,” *Proceedings of the 6th International Workshop on Computational Kinematics (CK2013)*, pp. 139–146, Springer, 2014.
- **Earlier closed-form method, manuscript reference [25]:** A. Pott, T. Bruckmann, and L. Mikelsons, “Closed-form force distribution for parallel wire robots,” *Computational Kinematics*, pp. 25–34, Springer, 2009.

ICFM denotes the improved method used in the manuscript.

## VTDA-L2

Implementation: [`wt_gouttefarde.m`](../src/wt_gouttefarde.m) · [Function reference](FUNCTION_REFERENCE.md#vtda-l2)

- **Manuscript reference [29]:** M. Gouttefarde, J. Lamaury, C. Reichert, and T. Bruckmann, “A versatile tension distribution algorithm for n-dof parallel robots driven by n+2 cables,” *IEEE Transactions on Robotics*, vol. 31, no. 6, pp. 1444–1457, 2015.

The implementation limits the polygon traversal to 100 visits and includes a feasible-vertex fallback. The fallback does not certify L2 optimality.

## Bounded QP

Implementation: [`wt_qp_bounded.m`](../src/wt_qp_bounded.m) · [Function reference](FUNCTION_REFERENCE.md#bounded-qp)

The implementation uses MATLAB `quadprog` with `active-set` in MATLAB R2024b Update 4 and Optimization Toolbox 24.2. It minimizes `0.5*T'*T` subject to force equilibrium and both tension bounds, initialized at each cable's lower bound. Initialization, feasibility handling, and options are specified in the function reference.

- [MATLAB R2024b `quadprog` documentation](https://www.mathworks.com/help/releases/R2024b/optim/ug/quadprog.html).

MATLAB and toolbox source code are not included.

## ACTA

Implementation: [`wt_acta.m`](../src/wt_acta.m) · [Function reference](FUNCTION_REFERENCE.md#acta)

- V. Di Paola, A. Goldsztejn, M. Zoppi, and S. Caro, “Analytic Centre Based Tension Distribution for Cable-Driven Platforms (CDPs),” *Journal of Mechanisms and Robotics*, vol. 16, no. 8, article 081018, 2024. [DOI: 10.1115/1.4065244](https://doi.org/10.1115/1.4065244).
- S. Boyd and L. Vandenberghe, *Convex Optimization*, 2004: Algorithm 10.2, p. 531, and block elimination from Section 10.4.2.

ACTA follows Section 4, Eqs. (12)–(18), and Algorithm 1 of Di Paola et al. (manuscript reference [22]). The infeasible-start Newton method follows Boyd and Vandenberghe.

This equation-based implementation does not use the authors' code. Initialization, iteration limits, line-search limits, and numerical safeguards are specified for this implementation.

## X-ACTA

Implementation: [`wt_xacta.m`](../src/wt_xacta.m) · [Function reference](FUNCTION_REFERENCE.md#x-acta)

- D. Dona', V. Di Paola, A. Trevisani, and M. Zoppi, “X-ACTA: eXtended Analytic Center Tension distribution Algorithm for fixed and mobile cable-driven-parallel-robot,” [arXiv:2607.08265v1](https://arxiv.org/html/2607.08265v1), 2026.

X-ACTA follows Eqs. (14)–(21) (manuscript reference [23]). This equation-based implementation does not use the authors' code. It uses affine-coordinate RAC, the eliminated EAC Schur equations, unit barrier weights, and the numerical safeguards specified in the function reference.

## WEC

Implementation: [`WEC.m`](../src/WEC.m) · [Function reference](FUNCTION_REFERENCE.md#wec-preprocessing)

- **Manuscript reference [34]:** G. Boschetti, C. Passarini, A. Trevisani, and D. Zanotto, “A fast algorithm for wrench exertion capability computation,” *Cable-Driven Parallel Robots*, edited by C. Gosselin, P. Cardou, T. Bruckmann, and A. Pott, pp. 292–303, Springer International Publishing, 2018.

WEC uses signed force-line adjustment and a separate zero-command policy. Their behavior and limitations are specified in the function reference.

## Experimental forward kinematics

Implementation: [`forward_kinematics.m`](../src/forward_kinematics.m) · [Function reference](FUNCTION_REFERENCE.md#experimental-forward-kinematics)

The function was used in the 2024 experimental controller, in the MATLAB Function block “Actual Position/forward kinematics 2” of `CompleteSystem_RL_Final.slx.original`. It averages fixed adjacent circle-pair intersections, excluding points containing `NaN`.

The executable calculation is preserved; the function name and comments are adapted for standalone use. Differences in invalid-intersection handling between MATLAB and the generated real-valued controller are documented in the function reference.

## Attribution limits

Mathematical citations do not establish software authorship or grant a software license. See the [source and license notices](../NOTICES.md).
