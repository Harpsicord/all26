package org.team100.lib.trajectory.spline;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.WaypointSE2;

public class SplineSE2Factory {
    /**
     * Make N-1 splines from N waypoint knots.
     */
    public static List<SplineSE2> splinesFromWaypoints(List<WaypointSE2> waypoints) {
        List<SplineSE2> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new SplineSE2(waypoints.get(i - 1), waypoints.get(i)));
        }
        return splines;
    }

}
