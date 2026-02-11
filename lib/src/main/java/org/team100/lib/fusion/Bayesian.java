package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Bayesian
 * 
 * https://stats.stackexchange.com/questions/237037/bayesian-updating-with-new-data
 * 
 * I think this post has an error in the mean computation, like they meant to
 * write precision (1/var).
 * 
 * https://seor.vse.gmu.edu/~klaskey/SYST664/Bayes_Unit5.pdf
 * 
 * Bayesian updating is identical to inverse variance weighting:
 * it weighs the more confident update more highly
 * it ignores mean dispersion
 */
public class Bayesian implements Fusor {

    @Override
    public VariableR1 fuse(VariableR1 a, VariableR1 b) {
        if (a.variance() < 1e-9 && b.variance() < 1e-9)
            return VariableR1.fromVariance((a.mean() + b.mean()) / 2, 0);
        if (a.variance() < 1e-9)
            return VariableR1.fromVariance(a.mean(), 0);
        if (b.variance() < 1e-9)
            return VariableR1.fromVariance(b.mean(), 0);

        // Bayes "precisions".
        double ap = 1 / a.variance();
        double bp = 1 / b.variance();
        double mean = (ap * a.mean() + bp * b.mean()) / (ap + bp);
        double p = ap + bp;
        double variance = 1 / p;
        return VariableR1.fromVariance(mean, variance);
    }

}
