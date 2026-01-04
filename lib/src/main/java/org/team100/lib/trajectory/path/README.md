# lib.trajectory.path

This package includes `PathSE2`, which describes a path through the SE(2)
manifold, the space that Pose2d lives in, with a list of samples.

The `PathFactorySE2` samples a spline so that the straight parts don't have too
many points, but the curved parts have more.

There are also versions of these for SE(3), the Pose3d manifold.