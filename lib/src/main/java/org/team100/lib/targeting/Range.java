package org.team100.lib.targeting;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Provides a firing solution (range and time of flight) by integrating a drag
 * model with initial conditions of elevation and fixed muzzle velocity.
 * 
 * All results are precomputed, and stored in an interpolating map.
 */
public class Range {
    private static final boolean DEBUG = false;
    private static final boolean CACHE = false;

    /** Try elevations this far apart. */
    private static final double ELEVATION_STEP = 0.01;

    private final Drag d;
    private final double v;
    private final double omega;

    /** key = elevation in radians, value = solution */
    private final InterpolatingTreeMap<Double, FiringSolution> m_map;

    /**
     * @param d     drag model
     * @param v     muzzle speed in m/s
     * @param omega spin in rad/s, positive is backspin
     */
    public Range(Drag d, double v, double omega) {
        this.d = d;
        this.v = v;
        this.omega = omega;
        m_map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringSolutionInterpolator());
        if (DEBUG)
            System.out.println("elevation, range, tof");
        for (double elevation = 0; elevation < Math.PI / 2; elevation += ELEVATION_STEP) {
            FiringSolution solution = RangeSolver.getSolution(d, v, omega, elevation);
            if (solution == null) {
                // no solution
                continue;
            }
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n",
                        elevation, solution.range(), solution.tof());
            m_map.put(elevation, solution);
        }
    }

    /**
     * @param elevation in radians
     */
    public FiringSolution get(double elevation) {
        if (!CACHE)
            return RangeSolver.getSolution(d, v, omega, elevation);
        return m_map.get(elevation);
    }

}
