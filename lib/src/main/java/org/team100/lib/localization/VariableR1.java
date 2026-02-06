package org.team100.lib.localization;

/** Random variable in one dimension */
public class VariableR1 {
    private final double mean;
    private final double variance;

    public VariableR1(double mean, double variance) {
        this.mean = mean;
        this.variance = variance;
    }

    /**
     * Operands are independent, so means and variances simply add.
     */
    public static VariableR1 add(VariableR1 a, VariableR1 b) {
        return new VariableR1(a.mean + b.mean, a.variance + b.variance);
    }

    /**
     * Uses inverse-variance weighting.
     */
    public static VariableR1 fuse(VariableR1 a, VariableR1 b) {
        double wA = weight(a.variance, b.variance);
        double wB = weight(b.variance, a.variance);
        double mean = wA * a.mean + wB * b.mean;
        // gaussian mixture
        // law of total variance
        // takes mean dispersion into account
        // https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
        // https://stats.stackexchange.com/questions/309622/calculate-moments-of-a-weighted-mixture-of-normal-distributions
        double variance = wA * a.variance + wB * b.variance + wA * wB * Math.pow(a.mean - b.mean, 2);

        return new VariableR1(mean, variance);
    }

    public static VariableR1 fuse2(VariableR1 a, VariableR1 b) {
        // TODO: handle zero
        double wA = 1 / a.variance;
        double wB = 1 / b.variance;
        double mean = (wA * a.mean + wB * b.mean) / (wA + wB);
        // inverse-variance weighting
        // does *not* take mean dispersion into account.
        // but does increase confidence with multiple measurements.
        double variance = 1 / (wA + wB);

        return new VariableR1(mean, variance);
    }

    public static double weight(double varA, double varB) {
        if (varA < 1e-3 && varB < 1e-3) {
            return 0.5;
        }
        if (varA < 1e-3) {
            return 1.0;
        }
        if (varB < 1e-3) {
            return 0.0;
        }
        return (1 / varA) / (1 / varA + 1 / varB);
    }

    public double mean() {
        return mean;
    }

    public double variance() {
        return variance;
    }

}
