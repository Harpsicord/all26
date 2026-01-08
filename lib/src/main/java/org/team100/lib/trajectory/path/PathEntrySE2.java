package org.team100.lib.trajectory.path;

import org.team100.lib.trajectory.path.spline.SplineSE2;

/**
 * Contains the "key" (spline, s value) and "value" (waypoint, curvature) of a
 * path sample
 */
public record PathEntrySE2(Parameter parameter, PathPointSE2 point) {
    public record Parameter(SplineSE2 spline, double s) {
    }
}
