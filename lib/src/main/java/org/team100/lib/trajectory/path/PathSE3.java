package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Metrics;

public class PathSE3 {

    private final List<PathSE3Entry> m_points;
    private final double[] m_distances;

    public PathSE3(final List<PathSE3Entry> states) {
        int n = states.size();
        m_points = new ArrayList<>(n);
        m_distances = new double[n];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i = 1; i < n; ++i) {
            m_points.add(states.get(i));
            PathSE3Point p0 = getEntry(i - 1).point();
            PathSE3Point p1 = getEntry(i).point();
            double dist = Metrics.translationalDistance(p0.waypoint().pose(), p1.waypoint().pose());
            m_distances[i] = m_distances[i - 1] + dist;
        }
    }

    public int length() {
        return m_points.size();
    }

    public PathSE3Entry getEntry(int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }
}
