package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Inverse-variance weighting is the maximum-likelihood estimator. Repeated
 * fusions of the same input represent additional "evidence" and so the result
 * becomes progressively more confident.
 * 
 * It tends to become overconfident ("covariance collapse") because the operands
 * are often not as independent as we may think they are.
 * 
 * This method also does not take the dispersion of the means into account: it
 * returns a confident result even when the operand means are very different.
 * The result is that the fusion can easily become very confident about the
 * wrong value.
 * 
 * https://en.wikipedia.org/wiki/Inverse-variance_weighting
 * https://en.wikipedia.org/wiki/Weighted_arithmetic_mean
 * https://www.nist.gov/system/files/documents/2017/05/09/combine-1.pdf
 */
public class InverseVarianceWeighting implements Fusor {

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
        double variance = 1 / totalWeight;
        return VariableR1.fromVariance(mean, variance);
    }

}
