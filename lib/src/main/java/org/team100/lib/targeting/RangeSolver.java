package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

/** Integrates the drag model until position is less than zero. */
public class RangeSolver {
    private static final boolean DEBUG = false;

    /**
     * RK4 integration with this resolution.
     * 
     * See RangeSolverTest for choice of DT
     */
    static final double INTEGRATION_DT = 0.01;

    /**
     * Both range and time-of-flight are always slight underestimates.
     * 
     * @param d         drag model
     * @param v         muzzle speed in m/s
     * @param omega     spin in rad/s, positive is backspin
     * @param elevation in rad
     */
    public static FiringSolution getSolution(
            Drag d, double v, double omega, double elevation) {
        return solveWithDt(d, v, omega, elevation, INTEGRATION_DT);
    }

    static FiringSolution solveWithDt(
            Drag d, double v, double omega, double elevation, double dt) {
        if (dt < 1e-6)
            throw new IllegalArgumentException("must use nonzero dt");
        double vx = v * Math.cos(elevation);
        double vy = v * Math.sin(elevation);
        // state is (x, y, theta, vx, vy, omega)
        Matrix<N6, N1> x = VecBuilder.fill(0, 0, 0, vx, vy, omega);
        Matrix<N6, N1> prevX = x;
        double t = 0;
        for (t = 0; t < 10; t += dt) {
            // this is the x for t+dt.
            x = NumericalIntegration.rk4(d, prevX, dt);
            if (DEBUG)
                System.out.printf("t %f x %f y %f\n",
                        t, x.get(0, 0), x.get(1, 0));
            double height = x.get(1, 0);
            if (height < 0) {
                // interpolate using the floor position
                // more-clever interpolation or integration makes no difference.
                double prevHeight = prevX.get(1, 0);
                double lerp = prevHeight / (prevHeight - height);
                double range = MathUtil.interpolate(prevX.get(0, 0), x.get(0, 0), lerp);
                double tof = t + dt * lerp;
                return new FiringSolution(range, tof);
            }
            prevX = x;
        }
        // if we got to the end, there's no (useful) solution.
        return null;
    }
}
