package org.team100.lib.localization;

import java.util.Random;
import java.util.function.UnaryOperator;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.uncertainty.Uncertainty;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Add noise to the twist computed from wheel deltas.
 * 
 * This noise source is not very realistic, but it's better than independent
 * noise at each wheel.
 */
public class OdometryNoise implements UnaryOperator<Twist2d> {
    private final Random m_rand;

    public OdometryNoise() {
        m_rand = new Random();
    }

    public Twist2d apply(Twist2d x) {
        double distanceM = Metrics.translationalNorm(x);
        double cartesian = Uncertainty.odometryCartesianStdDev(distanceM);
        double rotation = Uncertainty.odometryRotationStdDev(distanceM, x.dtheta);
        return new Twist2d(
                x.dx + cartesian * m_rand.nextGaussian(),
                x.dy + cartesian * m_rand.nextGaussian(),
                x.dtheta + rotation * m_rand.nextGaussian());
    }

}
