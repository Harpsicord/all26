# lib.state

This package represents mechanisms:

- `ModelR1` represents measurement in one dimension. Measurements never include acceleration,
since it is not directly measurable.
- `ControlR1` represents control outputs in one dimension, which _do_ contain acceleration,
which can translate directly into motor voltages using the "kA" factor of the motor models.

In the "state space" representation in control theory, the `ModelR1` is
the `x` variable and `ControlR1` is the `u` variable.

There are groupings for the SE(2) manifold of 2d transformations, `ModelSE2`
and `ControlSE2`.  These treat each dimension independently, not using the
logmap geodesic constant-twist idea.