# Method sources

## Implementation scope

The [`src/`](../src/) folder contains the tension distribution functions, WEC preprocessing, and the experimental forward-kinematics function. The function help text and [function reference](FUNCTION_REFERENCE.md) describe the interfaces, assumptions, solver settings, and returned statuses.

The references below identify mathematical methods. Equivalence to the original authors' software is not established for these MATLAB implementations.

## Direct Method

Implementation: [`wt_direct.m`](../src/wt_direct.m) · [Function reference](FUNCTION_REFERENCE.md#direct-method)

The manuscript is titled *Efficient L2-Optimal Tension Distribution in Planar Translational Cable Driven Parallel Robots with 2 Degrees of Redundancy*. The implementation follows Algorithm 1 (p. 6):

- Particular solution: Eq. (6).
- Null-space basis: Eq. (7).
- Feasible intervals: Eqs. (11)–(12).
- Reduced scalar objective: Eq. (13).
- Projection: Eq. (14).

The source forms the full `Pm = N'*N`. Its configuration labels II and III select the `alpha2` and `alpha1` axes, respectively. WEC is a separate preprocessing function.

## ICFM

Implementation: [`wt_pott.m`](../src/wt_pott.m) · [Function reference](FUNCTION_REFERENCE.md#icfm)

- **Manuscript reference [27]:** A. Pott, “An improved force distribution algorithm for over-constrained cable-driven parallel robots,” *Proceedings of the 6th International Workshop on Computational Kinematics (CK2013)*, pp. 139–146, Springer, 2014.
- **Earlier closed-form method, manuscript reference [25]:** A. Pott, T. Bruckmann, and L. Mikelsons, “Closed-form force distribution for parallel wire robots,” *Computational Kinematics*, pp. 25–34, Springer, 2009.

The ICFM label identifies the comparison method in the manuscript.

## VTDA-L2

Implementation: [`wt_gouttefarde.m`](../src/wt_gouttefarde.m) · [Function reference](FUNCTION_REFERENCE.md#vtda-l2)

- **Manuscript reference [29]:** M. Gouttefarde, J. Lamaury, C. Reichert, and T. Bruckmann, “A versatile tension distribution algorithm for n-dof parallel robots driven by n+2 cables,” *IEEE Transactions on Robotics*, vol. 31, no. 6, pp. 1444–1457, 2015.

The MATLAB implementation includes a 100-visit safeguard and a feasible-vertex fallback. A feasible fallback result is not a certificate of L2 optimality; see the function reference for its returned statuses.

## Bounded QP

Implementation: [`wt_qp_bounded.m`](../src/wt_qp_bounded.m) · [Function reference](FUNCTION_REFERENCE.md#bounded-qp)

The numerical backend is MATLAB `quadprog`, using `active-set`, Optimization Toolbox 24.2, and MATLAB R2024b Update 4. The local wrapper solves `min 0.5*T'*T` with equality and both box bounds, starting each cable at its lower tension bound. The function reference describes initialization, the feasibility phase, and options.

- [MATLAB R2024b `quadprog` documentation](https://www.mathworks.com/help/releases/R2024b/optim/ug/quadprog.html).

The commercial MATLAB/toolbox implementation is not included here.

## ACTA

Implementation: [`wt_acta.m`](../src/wt_acta.m) · [Function reference](FUNCTION_REFERENCE.md#acta)

- V. Di Paola, A. Goldsztejn, M. Zoppi, and S. Caro, “Analytic Centre Based Tension Distribution for Cable-Driven Platforms (CDPs),” *Journal of Mechanisms and Robotics*, vol. 16, no. 8, article 081018, 2024. [DOI: 10.1115/1.4065244](https://doi.org/10.1115/1.4065244).
- S. Boyd and L. Vandenberghe, *Convex Optimization*, 2004: Algorithm 10.2, p. 531, and block elimination from Section 10.4.2.

The ACTA source scope is Section 4, Eqs. (12)–(18), and Algorithm 1 of Di Paola et al.; this is manuscript reference [22]. The infeasible-start Newton method follows Boyd and Vandenberghe.

This is an equation-based implementation; it does not use the authors' code. Cold starts, finite iteration and line-search limits, and numerical safeguards are implementation choices, not a claim to reproduce the authors' program.

## X-ACTA

Implementation: [`wt_xacta.m`](../src/wt_xacta.m) · [Function reference](FUNCTION_REFERENCE.md#x-acta)

- D. Dona', V. Di Paola, A. Trevisani, and M. Zoppi, “X-ACTA: eXtended Analytic Center Tension distribution Algorithm for fixed and mobile cable-driven-parallel-robot,” [arXiv:2607.08265v1](https://arxiv.org/html/2607.08265v1), 2026.

The source scope is Eqs. (14)–(21); this is manuscript reference [23]. This is an equation-based implementation; it does not use the authors' code. It uses affine-coordinate RAC, the eliminated EAC Schur equations, unit barrier weights, and the numerical safeguards described in the function reference.

## WEC

Implementation: [`WEC.m`](../src/WEC.m) · [Function reference](FUNCTION_REFERENCE.md#wec-preprocessing)

- **Manuscript reference [34]:** G. Boschetti, C. Passarini, A. Trevisani, and D. Zanotto, “A fast algorithm for wrench exertion capability computation,” *Cable-Driven Parallel Robots*, edited by C. Gosselin, P. Cardou, T. Bruckmann, and A. Pott, pp. 292–303, Springer International Publishing, 2018.

This is the manuscript's method reference for WEC. The implementation uses signed force-line adjustment and a separate zero-command policy; their behavior and limitations are described in the function reference.

## Experimental forward kinematics

Implementation: [`forward_kinematics.m`](../src/forward_kinematics.m) · [Function reference](FUNCTION_REFERENCE.md#experimental-forward-kinematics)

This function was used in the 2024 experimental controller, in the MATLAB Function block “Actual Position/forward kinematics 2” of `CompleteSystem_RL_Final.slx.original`. It averages fixed adjacent circle-pair intersections, omitting points containing `NaN`. The standalone MATLAB and real-valued generated-controller domain behavior is documented in its help.

The public function name and comments are adapted for standalone use; the executable calculation is preserved.

## Attribution limits

A mathematical reference is not a software-license grant or an assertion that a file is original-author code. See the [source and license notices](../NOTICES.md).
