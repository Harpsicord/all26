package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Covariance Inflation
 * 
 * Result variance is inverse variance weighting, with two terms for covariance
 * inflation:
 * 
 * * mean dispersion weight: reduce the influence of mean dispersion, but not to
 * zero.
 * * minimum state variance: avoid state variance collapse.
 * 
 * I tuned these terms by eye, in this sheet:
 * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=1604242948#gid=1604242948
 */
public class CovarianceInflation implements Fusor {
    // TODO: make this adjustable?
    private static final double DISPERSION_WEIGHT = 0.02;
    // TODO: make this adjustable?
    private static final double MIN_VARIANCE = 0.000009;

    @Override
    public VariableR1 fuse(VariableR1 a, VariableR1 b) {
        if (a.variance() < 1e-9 && b.variance() < 1e-9)
            return VariableR1.fromVariance((a.mean() + b.mean()) / 2, 0);
        if (a.variance() < 1e-9)
            return VariableR1.fromVariance(a.mean(), 0);
        if (b.variance() < 1e-9)
            return VariableR1.fromVariance(b.mean(), 0);
        double wA = 1 / a.variance();
        double wB = 1 / b.variance();
        double totalWeight = wA + wB;
        double mean = (wA * a.mean() + wB * b.mean()) / totalWeight;

        // Inverse variance weight
        double variance = 1 / totalWeight;
        // Add (a little) mean dispersion, so that when very-different camera estimates
        // arrive, the state listens to them.

        variance += DISPERSION_WEIGHT * wA * Math.pow(a.mean() - mean, 2) / totalWeight
                + DISPERSION_WEIGHT * wB * Math.pow(b.mean() - mean, 2) / totalWeight;
        // Prevent variance collapse, so that the camera influence stays high
        // enough.
        variance = Math.max(variance, MIN_VARIANCE);
        return VariableR1.fromVariance(mean, variance);
    }

}
