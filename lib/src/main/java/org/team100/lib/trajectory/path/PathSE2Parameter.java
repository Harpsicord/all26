package org.team100.lib.trajectory.path;

import org.team100.lib.trajectory.spline.ISplineSE2;

/** A point on a spline. */
public record PathSE2Parameter(ISplineSE2 spline, double s) {
}