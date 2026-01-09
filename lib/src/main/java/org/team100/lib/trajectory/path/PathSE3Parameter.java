package org.team100.lib.trajectory.path;

import org.team100.lib.trajectory.spline.SplineSE3;

/** A point on a spline. */
public record PathSE3Parameter(SplineSE3 spline, double s) {
}
