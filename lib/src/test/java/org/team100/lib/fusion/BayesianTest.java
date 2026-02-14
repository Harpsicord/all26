package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.uncertainty.VariableR1;

public class BayesianTest {
    private static final double DELTA = 0.001;

    @Test
    void testCrisp() {
        VariableR1 a = VariableR1.fromVariance(0, 0);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new Bayesian().fuse(a, b);
        // Crisp variable wins
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0, c.variance(), DELTA);
    }

    @Test
    void testSelf() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(0, 1);
        VariableR1 c = new Bayesian().fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Overconfident.
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testHighVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 100);
        VariableR1 c = new Bayesian().fuse(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        // Adopts the lower variance.
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testEqualVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new Bayesian().fuse(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // Ignores mean dispersion :-(
        assertEquals(0.5, c.variance(), DELTA);
    }

}
