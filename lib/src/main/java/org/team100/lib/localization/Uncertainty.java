package org.team100.lib.localization;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Methods governing vision update uncertainties.
 * 
 * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
 * https://docs.google.com/spreadsheets/d/1StMbOyksydpzFmpHbZBL7ICMQtHnOBubrOMeYSx0M6E
 */
public class Uncertainty {
    /** this is the default value which, in hindsight, seems ridiculously high. */
    private static final double[] DEFAULT_STATE_STDDEV = new double[] {
            0.1,
            0.1,
            0.1 };

    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     */
    static final double[] TIGHT_STATE_STDDEV = new double[] {
            0.001,
            0.001,
            0.1 };

    static double[] visionMeasurementStdDevs(double distanceM, double offAxisAngleRad) {
        // these extra 0.01 values are total guesses
        // TODO: remove them?
        return new double[] {
                figure5(distanceM) + 0.01,
                figure5(distanceM) + 0.01,
                figure6(offAxisAngleRad) + 0.01 };
    }

    /**
     * Figure 5 in the Wang paper (below 15 m) indicates a linear relationship
     * between cartesian error and tag distance.
     */
    static double figure5(double distanceM) {
        return 0.03 * distanceM;
    }

    /**
     * Figure 6 in the Wang paper indicates a U-shaped relationship between the "off
     * axis" angle and the error. Note the very large error at zero.
     */
    static double figure6(double offAxisAngleRad) {
        if (offAxisAngleRad < 0)
            throw new IllegalArgumentException("angle must be non-negative");
        // This uses degrees because figure 6 uses degrees.
        double offAxisDegrees = Math.toDegrees(offAxisAngleRad);
        if (offAxisDegrees < 3)
            return Double.MAX_VALUE;
        return 10 / offAxisDegrees + 10 / Math.pow(85 - offAxisDegrees, 1.2);
    }

    static double[] stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return Uncertainty.TIGHT_STATE_STDDEV;
        }
        return Uncertainty.DEFAULT_STATE_STDDEV;
    }

    static Twist2d getScaledTwist(
            double[] stateSigma,
            double[] visionSigma,
            Twist2d twist) {
        // discount the vision update by this factor.
        final double[] K = Uncertainty.getK(stateSigma, visionSigma);
        Twist2d scaledTwist = new Twist2d(
                K[0] * twist.dx,
                K[1] * twist.dy,
                K[2] * twist.dtheta);
        return scaledTwist;
    }

    static double[] getK(double[] stateSigma, double[] visionSigma) {
        return new double[] {
                Uncertainty.mix(Math.pow(stateSigma[0], 2), Math.pow(visionSigma[0], 2)),
                Uncertainty.mix(Math.pow(stateSigma[1], 2), Math.pow(visionSigma[1], 2)),
                Uncertainty.mix(Math.pow(stateSigma[2], 2), Math.pow(visionSigma[2], 2))
        };
    }

    /**
     * Given q and r stddev's, what mixture should that yield?
     * This is the "closed form Kalman gain for continuous Kalman filter with A = 0
     * and C = I. See wpimath/algorithms.md." ... but really it's just a mixer.
     * 
     * @param q state variance
     * @param r vision variance
     */
    static double mix(final double q, final double r) {
        if (q == 0.0)
            return 0.0;
        return q / (q + Math.sqrt(q * r));
    }

}
