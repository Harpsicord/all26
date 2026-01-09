package org.team100.lib.trajectory;

import org.team100.lib.trajectory.path.PathSE2Entry;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.trajectory.path.PathSE3Entry;
import org.team100.lib.trajectory.path.PathSE3Point;
import org.team100.lib.trajectory.path.PathUtil;

public class TrajectoryUtil {
    private static final boolean DEBUG = false;

    /**
     * Linear interpolation by time.
     * 
     * Velocity of this state is the initial velocity.
     * Acceleration of this state is constant through the whole arc.
     */
    public static TrajectorySE2Entry interpolate(TrajectorySE2Entry a, TrajectorySE2Entry b, double delta_t) {
        if (delta_t < 0)
            throw new IllegalArgumentException("delta_t must be non-negative");
        if (DEBUG)
            System.out.println("lerp");
        double tLerp = a.point().time() + delta_t;
        double vLerp = a.point().velocity() + a.point().accel() * delta_t;
        double pathwiseDistance = a.point().velocity() * delta_t + 0.5 * a.point().accel() * delta_t * delta_t;

        PathSE2Point aPoint = a.point().point();
        PathSE2Point bPoint = b.point().point();
        double distanceBetween = aPoint.distanceCartesian(bPoint);
        double interpolant = pathwiseDistance / distanceBetween;
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        // TODO: pass t interpolant, not just spatial one
        double timeInterp = delta_t / (b.point().time() - a.point().time());
        if (DEBUG)
            System.out.printf("t0 %f t1 %f delta t %f timeInterp %f\n",
                    a.point().time(), b.point().time(), delta_t, timeInterp);

        if (DEBUG)
            System.out.printf("tlerp %f\n", tLerp);
        PathSE2Entry aEntry = new PathSE2Entry(a.parameter(), aPoint);
        PathSE2Entry bEntry = new PathSE2Entry(b.parameter(), bPoint);

        PathSE2Entry entryLerp = PathUtil.interpolate(aEntry, bEntry, timeInterp);
        
        return new TrajectorySE2Entry(
                entryLerp.parameter(),
                new TrajectorySE2Point(entryLerp.point(), tLerp, vLerp, a.point().accel()));
    }

    public static TrajectorySE3Entry interpolate(TrajectorySE3Entry a, TrajectorySE3Entry b, double delta_t) {
        if (delta_t < 0)
            throw new IllegalArgumentException("delta_t must be non-negative");
        if (TrajectorySE3Entry.DEBUG)
            System.out.println("lerp");
        double tLerp = a.point().time() + delta_t;
        double vLerp = a.point().velocity() + a.point().accel() * delta_t;
        double pathwiseDistance = a.point().velocity() * delta_t + 0.5 * a.point().accel() * delta_t * delta_t;
    
        PathSE3Point aPoint = a.point().point();
        PathSE3Point bPoint = b.point().point();
        double distanceBetween = aPoint.distanceCartesian(bPoint);
        double interpolant = pathwiseDistance / distanceBetween;
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }
    
        // TODO: pass t interpolant, not just spatial one
        double timeInterp = delta_t / (b.point().time() - a.point().time());
        if (TrajectorySE3Entry.DEBUG)
            System.out.printf("t0 %f t1 %f delta t %f s %f\n",
                    a.point().time(), b.point().time(), delta_t, timeInterp);
    
        if (TrajectorySE3Entry.DEBUG)
            System.out.printf("tlerp %f\n", tLerp);
    
        PathSE3Entry aEntry = new PathSE3Entry(a.parameter(), aPoint);
        PathSE3Entry bEntry = new PathSE3Entry(b.parameter(), bPoint);
    
        PathSE3Entry entryLerp = PathUtil.interpolate(aEntry, bEntry, timeInterp);
    
        return new TrajectorySE3Entry(
                entryLerp.parameter(),
                new TrajectorySE3Point(entryLerp.point(), tLerp, vLerp, a.point().accel()));
    }

}
