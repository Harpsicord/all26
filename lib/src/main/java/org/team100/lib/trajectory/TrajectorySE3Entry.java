package org.team100.lib.trajectory;

import org.team100.lib.trajectory.path.PathSE3Point;
import org.team100.lib.trajectory.path.PathUtil;

/**
 * Represents a state within a trajectory in SE(3).
 */
public class TrajectorySE3Entry {
    private static final boolean DEBUG = false;

    private final PathSE3Point m_point;
    private final double m_timeS;
    private final double m_velocityM_S;
    private final double m_accelM_S_S;

    public TrajectorySE3Entry(
            PathSE3Point point,
            double t,
            double velocity,
            double acceleration) {
        m_point = point;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    public PathSE3Point point() {
        return m_point;
    }

    public double getTimeS() {
        return m_timeS;
    }

    public double velocityM_S() {
        return m_velocityM_S;
    }

    public double acceleration() {
        return m_accelM_S_S;
    }

    public TrajectorySE3Entry interpolate(TrajectorySE3Entry other, double delta_t) {
        if (delta_t < 0)
            throw new IllegalArgumentException("delta_t must be non-negative");
        if (DEBUG)
            System.out.println("lerp");
        double tLerp = m_timeS + delta_t;
        double vLerp = m_velocityM_S + m_accelM_S_S * delta_t;
        double pathwiseDistance = m_velocityM_S * delta_t + 0.5 * m_accelM_S_S * delta_t * delta_t;

        double distanceBetween = m_point.distanceCartesian(other.m_point);
        double interpolant = pathwiseDistance / distanceBetween;
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        // TODO: pass t interpolant, not just spatial one
        double s = delta_t / (other.m_timeS - m_timeS);
        if (DEBUG)
            System.out.printf("t0 %f t1 %f delta t %f s %f\n",
                    m_timeS, other.m_timeS, delta_t, s);

        if (DEBUG)
            System.out.printf("tlerp %f\n", tLerp);
        return new TrajectorySE3Entry(
                PathUtil.interpolate(m_point, other.m_point, s),
                tLerp,
                vLerp,
                m_accelM_S_S);
    }

}
