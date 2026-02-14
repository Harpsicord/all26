package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Gaussian mixture method.
 * 
 * This fusion method is appropriate when the two operands describe
 * subpopulations. The result is intended to reflect the total population. This
 * method is the same as Covariance Intersection if the weights are chosen to be
 * the inverse variances.
 * 
 * Repeated fusions of the same input do not represent additional "evidence" and
 * so the result confidence does not change.
 * 
 * Uses inverse-variance weights but also includes a covariance term for the
 * dispersion of the means -- this is the "law of total variance."
 * 
 * The variance is never less than the smaller component variance.
 * 
 * This method is useful to avoid "covariance collapse" overconfidence, when
 * you don't know what the correlation between two variables is.
 * 
 * https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
 * https://stats.stackexchange.com/questions/309622/calculate-moments-of-a-weighted-mixture-of-normal-distributions
 */
public class GaussianMixture implements Fusor {

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
        double variance = 2 / totalWeight
                + wA * Math.pow(a.mean() - mean, 2) / totalWeight
                + wB * Math.pow(b.mean() - mean, 2) / totalWeight;
        return VariableR1.fromVariance(mean, variance);
    }

}
