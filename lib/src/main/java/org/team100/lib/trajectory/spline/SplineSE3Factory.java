package org.team100.lib.trajectory.spline;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.WaypointSE3;

public class SplineSE3Factory {

    /**
     * Make N-1 splines from N waypoint knots.
     */
    public static List<SplineSE3> splinesFromWaypoints(List<WaypointSE3> waypoints) {
        List<SplineSE3> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new SplineSE3(waypoints.get(i - 1), waypoints.get(i)));
        }
        return splines;
    }

}
